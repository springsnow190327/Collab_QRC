#!/usr/bin/env bash
# onboard_record_noetic.sh — record Livox raw + FAST-LIO2 outputs to a ROS 1
# bag on the Jetson. Sister to onboard_fastlio_noetic.sh; assumes that stack
# is ALREADY running (it shares the same roscore via ROS_MASTER_URI).
#
# Why a separate script (not just `rosbag record` inline)?
#   - Same conda-deactivation guard as the launcher (interactive .bashrc on
#     this Jetson auto-activates `conda activate base`, which can shadow
#     rosbag's python deps).
#   - Topic list bundles raw sensor + SLAM output + TF for offline replay.
#   - Bag finalization on Ctrl+C: rosbag is SIGINT'd first (writes footer),
#     then publishers continue — symmetric to scripts/real/onboard_record.sh.
#
# Topics recorded (always):
#   /livox/lidar  /livox/imu                            ← raw sensor (CustomMsg)
# With fastlio=true (default — capture SLAM output too):
#   /robot/Odometry  /robot/cloud_registered  /robot/cloud_registered_body
#   /robot/path  /tf  /tf_static
#
# Output: $BAG_DIR  (.bag, ROS 1 native; sqlite3/mcap aren't used).
#
# Usage (run on Jetson, AFTER onboard_fastlio_noetic.sh):
#   ./onboard_record_noetic.sh
#   ./onboard_record_noetic.sh fastlio=false       # raw lidar+imu only
#   ./onboard_record_noetic.sh tag=corridor_run1
#   ./onboard_record_noetic.sh bag_dir=/data/bags
#   ./onboard_record_noetic.sh ns=robot_b          # namespace match
#   ./onboard_record_noetic.sh split=1024          # split at 1024 MB (default)
#   ./onboard_record_noetic.sh stop                # kill any running rosbag

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"   # /home/unitree/noetic_fastlio_ws

# ── Defaults ─────────────────────────────────────────────────────────
NAMESPACE="robot"
WITH_FASTLIO="true"
TAG=""
BAG_DIR_OVERRIDE=""
BAG_ROOT_DEFAULT="/home/unitree/bags"
SPLIT_MB="1024"      # rosbag's --split + --size is in MB. 1 GB chunks.
ROS_MASTER_PORT="11311"

# ── Cleanup helpers ──────────────────────────────────────────────────
BAG_PID=""
_kill_record() {
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "  → SIGINT rosbag (waiting up to 8s for footer write)"
    kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 1
    done
  fi
  pkill -INT -f "rosbag record" 2>/dev/null || true
  sleep 1
  pkill -9 -f "rosbag record" 2>/dev/null || true
}

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping any running rosbag..."
      _kill_record
      echo "Done."
      exit 0
      ;;
    fastlio=*)        WITH_FASTLIO="${arg#fastlio=}" ;;
    tag=*)            TAG="${arg#tag=}" ;;
    bag_dir=*)        BAG_DIR_OVERRIDE="${arg#bag_dir=}" ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    split=*)          SPLIT_MB="${arg#split=}" ;;
    port=*)           ROS_MASTER_PORT="${arg#port=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$WITH_FASTLIO" in true|false) ;; *) echo "ERROR: fastlio must be true|false" >&2; exit 1 ;; esac

# ── Strip miniconda (same guard as launcher) ─────────────────────────
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  echo "  Stripping miniconda from env..."
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE "(miniconda|conda)" | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

# ── ROS env ──────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
[[ -f "$WS_ROOT/devel/setup.bash" ]] && source "$WS_ROOT/devel/setup.bash"

# Hook to whatever ROS master onboard_fastlio_noetic.sh is using (same host,
# same port — bake the host as the Jetson's own primary IP).
export ROS_MASTER_URI="http://$(hostname -I | awk '{print $1}'):${ROS_MASTER_PORT}"
export ROS_IP="$(hostname -I | awk '{print $1}')"

# Verify master is up — recording without a master silently produces nothing.
if ! rostopic list &>/dev/null; then
  echo "ERROR: no ROS master at $ROS_MASTER_URI." >&2
  echo "       Start the stack first:  $WS_ROOT/scripts/onboard_fastlio_noetic.sh" >&2
  exit 1
fi

# ── Bag output path ──────────────────────────────────────────────────
BAG_ROOT="${BAG_DIR_OVERRIDE:-$BAG_ROOT_DEFAULT}"
mkdir -p "$BAG_ROOT" || { echo "ERROR: cannot mkdir $BAG_ROOT" >&2; exit 1; }
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="onboard_noetic_${STAMP}"
[[ -n "$TAG" ]] && BAG_NAME="${BAG_NAME}_${TAG//[^A-Za-z0-9_-]/_}"
BAG_PREFIX="${BAG_ROOT}/${BAG_NAME}"      # rosbag adds _0.bag, _1.bag, etc.

# ── Topic list ───────────────────────────────────────────────────────
TOPICS=(
  /livox/lidar
  /livox/imu
)
if [[ "$WITH_FASTLIO" == "true" ]]; then
  TOPICS+=(
    /${NAMESPACE}/Odometry
    /${NAMESPACE}/cloud_registered
    /${NAMESPACE}/cloud_registered_body
    /${NAMESPACE}/path
    /tf
    /tf_static
    /rosout        # FastLIO logs ikd-tree rebuild times + iteration counts here
    /rosout_agg
  )
fi

# Verify each topic has a publisher BEFORE recording — rosbag silently
# records empty if topic doesn't exist yet.
# `rostopic info` output for a published topic has a literal " * /node" line
# under "Publishers:". For an unpublished topic it has "Publishers: None".
echo ""
echo "=== Verifying ${#TOPICS[@]} topics ==="
MISSING=()
for t in "${TOPICS[@]}"; do
  INFO=$(rostopic info "$t" 2>/dev/null)
  if echo "$INFO" | grep -q "^Publishers: None"; then
    echo "  ✗ $t  (no publisher — will record empty)"
    MISSING+=("$t")
  elif echo "$INFO" | grep -q "^ \* /"; then
    PUB=$(echo "$INFO" | grep "^ \* /" | head -1 | awk '{print $2}')
    echo "  ✓ $t  → $PUB"
  else
    echo "  ✗ $t  (topic not advertised — will record empty)"
    MISSING+=("$t")
  fi
done
if [[ ${#MISSING[@]} -gt 0 ]]; then
  echo ""
  echo "  WARN: ${#MISSING[@]} topic(s) have no publisher. Most common cause:"
  echo "        onboard_fastlio_noetic.sh isn't running, OR Mid-360 hasn't"
  echo "        started streaming yet (give it ~5s after launcher boot)."
fi
echo ""

# ── Banner ───────────────────────────────────────────────────────────
echo "################################################"
echo "  Noetic onboard RECORD"
echo "    namespace : $NAMESPACE"
echo "    fastlio   : $WITH_FASTLIO"
echo "    topics    : ${#TOPICS[@]}"
echo "    bag       : ${BAG_PREFIX}_*.bag"
echo "    split     : ${SPLIT_MB} MB per chunk"
echo "    tag       : ${TAG:-<none>}"
echo "  Walk the robot via the BT pad."
echo "  Stop with Ctrl+C — bag is finalized cleanly first."
echo "################################################"
echo ""

# ── Trap ─────────────────────────────────────────────────────────────
cleanup_on_signal() {
  trap - INT TERM EXIT
  echo ""
  echo "Caught interrupt — finalizing bag..."
  _kill_record
  echo ""
  echo "  Recorded files:"
  ls -lh "${BAG_PREFIX}"_*.bag "${BAG_PREFIX}"_*.bag.active 2>/dev/null \
    | awk '{print "    " $NF "  " $5}' || echo "    (none)"
  TOTAL=$(du -ch "${BAG_PREFIX}"_*.bag "${BAG_PREFIX}"_*.bag.active 2>/dev/null | tail -1 | cut -f1)
  [[ -n "$TOTAL" ]] && echo "  Total: $TOTAL"
  echo ""
  echo "  Replay (on Jetson or after rsync to laptop):"
  echo "    rosbag play ${BAG_PREFIX}_*.bag --clock"
  exit 0
}
trap cleanup_on_signal INT TERM

# ── Manifest (written next to bag for offline self-description) ──────
MANIFEST="${BAG_PREFIX}.manifest.txt"
{
  echo "kind=onboard_noetic_record"
  echo "fastlio_running_during_record=$WITH_FASTLIO"
  echo "namespace=$NAMESPACE"
  echo "tag=$TAG"
  echo "stamp=$STAMP"
  echo "hostname=$(hostname)"
  echo "user=$(whoami)"
  echo "ros_distro=noetic"
  echo "ros_master_uri=$ROS_MASTER_URI"
  echo "date=$(date -Iseconds)"
  echo "topics=${TOPICS[*]}"
  echo "split_mb=$SPLIT_MB"
  echo "args=$*"
} > "$MANIFEST"

# ── rosbag record ────────────────────────────────────────────────────
# --split + --size enables size-based rolling. --buffsize=0 disables write
# buffering (less risk of losing the tail on crash; minor overhead).
# --output-prefix uses BAG_PREFIX as a base; rosbag adds timestamp + _N.bag.
echo "Starting rosbag record..."
# nohup + disown so the recording survives an SSH session drop (this Jetson's
# USB-C dongle goes NO_CARRIER intermittently — without nohup, the operator's
# rosbag gets SIGHUP'd and the .bag.active is left orphaned, requiring
# `rosbag reindex` to recover the data).
nohup rosbag record \
  --output-name "${BAG_PREFIX}_0.bag" \
  --split --size "$SPLIT_MB" \
  --buffsize 0 \
  "${TOPICS[@]}" \
  >/tmp/onboard_record_noetic.log 2>&1 &
BAG_PID=$!
disown $BAG_PID 2>/dev/null || true

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag died within 2s. Log:" >&2
  sed 's/^/        /' /tmp/onboard_record_noetic.log >&2
  rm -f "$MANIFEST"
  exit 1
fi

echo "  rosbag PID=$BAG_PID  log: /tmp/onboard_record_noetic.log"
echo ""

# ── Status loop ──────────────────────────────────────────────────────
echo "RECORDING.  Walk the robot through the scene."
echo "Press Ctrl+C to stop and finalize."
echo ""
START_T=$SECONDS
while kill -0 "$BAG_PID" 2>/dev/null; do
  ELAPSED=$(( SECONDS - START_T ))
  # Include both finalised (.bag) and in-progress (.bag.active) chunks.
  # Without .active, current chunk is invisible until it splits or stops.
  TOTAL_SIZE=$(du -cb "${BAG_PREFIX}"_*.bag "${BAG_PREFIX}"_*.bag.active 2>/dev/null \
               | tail -1 | cut -f1)
  TOTAL_HUMAN=$(numfmt --to=iec --suffix=B "${TOTAL_SIZE:-0}" 2>/dev/null || echo "?B")
  NUM_DONE=$(ls "${BAG_PREFIX}"_*.bag 2>/dev/null | wc -l)
  NUM_ACTIVE=$(ls "${BAG_PREFIX}"_*.bag.active 2>/dev/null | wc -l)
  printf "\r  t=%4ds  chunks=%d (+%d active)  total=%-9s" \
    "$ELAPSED" "$NUM_DONE" "$NUM_ACTIVE" "$TOTAL_HUMAN"
  sleep 2
done

echo ""
echo "WARN: rosbag exited unexpectedly. Cleaning up..."
cleanup_on_signal
