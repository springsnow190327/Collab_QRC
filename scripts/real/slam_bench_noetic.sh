#!/usr/bin/env bash
# slam_bench_noetic.sh — diagnostic companion for FastLIO vs PointLIO A/B comparison.
#
# Run ALONGSIDE onboard_{fastlio,pointlio}_noetic.sh (SLAM must already be up).
# Captures in parallel:
#   odom_hz.log       — rostopic hz /robot/Odometry  (SLAM output rate, RTF proxy)
#   lidar_hz.log      — rostopic hz /livox/lidar      (LiDAR reference, should be ~10 Hz)
#   odom_delay.log    — rostopic delay /robot/Odometry (end-to-end latency: wall_recv − header.stamp)
#   tegrastats.log    — CPU/GPU/MEM every 2s (Jetson Orin)
#   rosout.bag        — /rosout + /rosout_agg (ikd-tree rebuild times, iteration counts)
#   bench_meta.yaml   — slam type, start time, node list, param snapshot
#
# RTF = odom_hz / lidar_hz.  Healthy: both ~10 Hz → RTF ≈ 1.0.
# FastLIO degradation signature: lidar_hz stays 10 Hz, odom_hz drops to 4-5 Hz → RTF ≈ 0.4-0.5.
# PointLIO (iVox O(1)) should stay at RTF ≈ 1.0 throughout.
#
# Usage (on Jetson, same terminal session that has ROS_MASTER_URI set):
#   ./slam_bench_noetic.sh slam=fastlio
#   ./slam_bench_noetic.sh slam=pointlio
#   ./slam_bench_noetic.sh slam=fastlio ns=robot out=/data/bench duration=300
#   ./slam_bench_noetic.sh stop           # kill any running bench
#
# WiFi note: set ROS_MASTER_URI + ROS_IP before calling this script if not
# already exported from the SLAM launcher session.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"

# ── Strip miniconda (same guard as other noetic scripts) ─────────────
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE "(miniconda|conda)" | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

# ── Defaults ─────────────────────────────────────────────────────────
SLAM_TYPE=""
NAMESPACE="robot"
OUT_ROOT="/home/unitree/bags/slam_bench"
DURATION=0       # 0 = run until Ctrl+C
ROS_MASTER_PORT="11311"
HZ_WINDOW=20     # message window for rostopic hz rolling average

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Killing any running slam_bench processes..."
      pkill -f "rostopic hz" 2>/dev/null || true
      pkill -f "rostopic delay" 2>/dev/null || true
      pkill -f "tegrastats" 2>/dev/null || true
      pkill -f "rosbag record.*rosout" 2>/dev/null || true
      echo "Done."
      exit 0
      ;;
    slam=*)      SLAM_TYPE="${arg#slam=}" ;;
    ns=*)        NAMESPACE="${arg#ns=}" ;;
    out=*)       OUT_ROOT="${arg#out=}" ;;
    duration=*)  DURATION="${arg#duration=}" ;;
    port=*)      ROS_MASTER_PORT="${arg#port=}" ;;
    hz_window=*) HZ_WINDOW="${arg#hz_window=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

if [[ -z "$SLAM_TYPE" ]]; then
  echo "ERROR: slam= required.  Use slam=fastlio or slam=pointlio." >&2
  exit 1
fi
case "$SLAM_TYPE" in fastlio|pointlio) ;; *)
  echo "ERROR: slam must be fastlio or pointlio (got '$SLAM_TYPE')" >&2; exit 1 ;;
esac

# ── ROS env ──────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
[[ -f "$WS_ROOT/devel/setup.bash" ]] && source "$WS_ROOT/devel/setup.bash"

# Inherit ROS_MASTER_URI from environment if already set by the SLAM launcher.
# Fall back to localhost.
if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  export ROS_MASTER_URI="http://$(hostname -I | awk '{print $1}'):${ROS_MASTER_PORT}"
  export ROS_IP="$(hostname -I | awk '{print $1}')"
fi

if ! rostopic list &>/dev/null; then
  echo "ERROR: no ROS master at $ROS_MASTER_URI." >&2
  echo "       Start the SLAM stack first, then run this script in the SAME shell" >&2
  echo "       (or export ROS_MASTER_URI=http://<jetson_ip>:11311 first)." >&2
  exit 1
fi

# ── Output directory ─────────────────────────────────────────────────
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_ROOT}/${SLAM_TYPE}_${STAMP}"
mkdir -p "$OUT_DIR"
echo "  Output: $OUT_DIR"

# ── Metadata snapshot ────────────────────────────────────────────────
{
  echo "slam: $SLAM_TYPE"
  echo "namespace: $NAMESPACE"
  echo "start: $STAMP"
  echo "ros_master_uri: $ROS_MASTER_URI"
  echo "hostname: $(hostname)"
  echo "kernel: $(uname -r)"
  echo "hz_window: $HZ_WINDOW"
  echo "duration: $DURATION"
  echo "nodes:"
  rosnode list 2>/dev/null | sed 's/^/  - /'
  echo "topics:"
  rostopic list 2>/dev/null | sed 's/^/  - /'
} > "$OUT_DIR/bench_meta.yaml"

# FastLIO param dump (ikd-tree related params live in the rosparam server)
rosparam dump "$OUT_DIR/rosparam_dump.yaml" 2>/dev/null || true

# ── Background loggers ───────────────────────────────────────────────
PIDS=()

# 1. SLAM odometry rate (the key RTF proxy)
#    -w N: rolling window of N messages for smoother per-second Hz output
rostopic hz "/${NAMESPACE}/Odometry" -w "$HZ_WINDOW" \
  >"$OUT_DIR/odom_hz.log" 2>&1 &
PIDS+=($!)

# 2. LiDAR raw rate (should be rock-steady 10 Hz; drop here = hardware issue)
rostopic hz /livox/lidar -w "$HZ_WINDOW" \
  >"$OUT_DIR/lidar_hz.log" 2>&1 &
PIDS+=($!)

# 3. End-to-end SLAM latency
#    rostopic delay measures: wall_clock_receive_time − header.stamp
#    For FastLIO: header.stamp = LiDAR scan timestamp
#    → delay = true processing latency (should be < 0.1s; spikes = ikd-tree stall)
rostopic delay "/${NAMESPACE}/Odometry" \
  >"$OUT_DIR/odom_delay.log" 2>&1 &
PIDS+=($!)

# 4. Jetson system stats (CPU cores, GPU, MEM — Orin-specific tegrastats)
if command -v tegrastats &>/dev/null; then
  tegrastats --interval 2000 --logfile "$OUT_DIR/tegrastats.log" &
  PIDS+=($!)
else
  # Fallback: /proc stats every 2s
  (while true; do
    echo "=== $(date +%T) ===" >> "$OUT_DIR/proc_stats.log"
    cat /proc/loadavg >> "$OUT_DIR/proc_stats.log"
    grep -E "^(MemTotal|MemAvailable|Buffers|Cached)" /proc/meminfo >> "$OUT_DIR/proc_stats.log"
    sleep 2
  done) &
  PIDS+=($!)
fi

# 5. /rosout bag — FastLIO prints ikd-tree rebuild count + per-iteration ms to rosout
#    Search for "kdtree" or "average" in the extracted log after the run.
rosbag record \
  --output-name "$OUT_DIR/rosout_0.bag" \
  --buffsize 0 \
  /rosout /rosout_agg \
  >/tmp/slam_bench_rosout.log 2>&1 &
PIDS+=($!)

# ── Cleanup ──────────────────────────────────────────────────────────
_cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "  Stopping bench loggers..."
  for pid in "${PIDS[@]}"; do
    kill -INT "$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
  done
  # Let rosbag write its footer
  sleep 3
  pkill -f "tegrastats" 2>/dev/null || true

  echo ""
  echo "  ── Results in $OUT_DIR ────────────────────────────"
  ls -lh "$OUT_DIR/" 2>/dev/null | awk '{print "    " $0}'
  echo ""

  # Quick summary from odom_hz.log (last 10 lines)
  if [[ -s "$OUT_DIR/odom_hz.log" ]]; then
    echo "  ── Odometry Hz (last 10 samples) ───────────────────"
    grep "average rate" "$OUT_DIR/odom_hz.log" | tail -10 | awk '{print "    " $0}'
  fi
  if [[ -s "$OUT_DIR/odom_delay.log" ]]; then
    echo "  ── Odometry delay (last 5 samples) ─────────────────"
    grep -E "mean|min|max" "$OUT_DIR/odom_delay.log" | tail -5 | awk '{print "    " $0}'
  fi

  echo ""
  echo "  Post-process on laptop after rsync:"
  echo "    python3 scripts/real/slam_bench_analyze.py $OUT_DIR/"
  echo ""
  echo "  Extract /rosout ikd-tree logs from bag:"
  echo "    rostopic echo -b $OUT_DIR/rosout_0.bag /rosout | grep -i 'kd\\|average\\|iter'"
  exit 0
}
trap _cleanup INT TERM

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  SLAM Bench — $SLAM_TYPE"
echo "    namespace  : /$NAMESPACE"
echo "    out        : $OUT_DIR"
echo "    loggers    : odom_hz | lidar_hz | odom_delay | tegrastats | rosout.bag"
echo "    duration   : ${DURATION}s  (0 = until Ctrl+C)"
echo ""
echo "  Expected healthy values:"
echo "    odom_hz ≈ 10.0 Hz  (RTF = odom_hz / 10.0 ≈ 1.0)"
echo "    odom_delay ≈ 0.02–0.08 s"
echo "  FastLIO degradation signature:"
echo "    odom_hz drops to 4–5 Hz after 2–3 min walking"
echo "    odom_delay spikes to 0.3–1.0 s"
echo "################################################"
echo ""
echo "  Walk the robot. Press Ctrl+C to stop and finalize."
echo ""

# ── Wait loop ────────────────────────────────────────────────────────
if [[ "$DURATION" -gt 0 ]]; then
  sleep "$DURATION"
  _cleanup
else
  # Check all background PIDs are still alive every 5s
  while true; do
    for pid in "${PIDS[@]}"; do
      if ! kill -0 "$pid" 2>/dev/null; then
        echo "WARN: logger PID=$pid exited unexpectedly. Check logs in $OUT_DIR." >&2
      fi
    done
    ELAPSED=$(( SECONDS ))
    ODOM_RATE=$(grep "average rate" "$OUT_DIR/odom_hz.log" 2>/dev/null | tail -1 | grep -oP '[\d.]+(?= Hz)' || echo "?")
    LIDAR_RATE=$(grep "average rate" "$OUT_DIR/lidar_hz.log" 2>/dev/null | tail -1 | grep -oP '[\d.]+(?= Hz)' || echo "?")
    printf "\r  t=%4ds  odom=%-6s Hz  lidar=%-6s Hz  RTF=%-5s" \
      "$ELAPSED" "$ODOM_RATE" "$LIDAR_RATE" \
      "$(awk "BEGIN{if(\"$ODOM_RATE\"~/[0-9]/ && \"$LIDAR_RATE\"~/[0-9]/) printf \"%.2f\", $ODOM_RATE/$LIDAR_RATE; else print \"?\"}")"
    sleep 5
  done
fi
