#!/usr/bin/env bash
# slam_record_session.sh — laptop-side one-shot launcher for onboard SLAM + rosbag.
#
# Laptop requires: sshpass (apt install sshpass).  No ROS needed on laptop.
# Everything runs on the Jetson (Noetic).  Laptop only manages SSH sessions.
#
# What it does:
#   1. Ensures Ethernet link (192.168.123.100 on dongle).
#   2. SSHes into Jetson → starts FastLIO or PointLIO (with its own roscore).
#   3. SSHes into Jetson → starts rosbag recording.
#   4. Optionally SSHes into Jetson → starts slam_bench (RTF + latency + tegrastats).
#   5. On Ctrl+C: stops bag cleanly (SIGINT → footer write), kills SLAM, done.
#
# Usage:
#   JETSON_PASS=123 ./slam_record_session.sh slam=fastlio
#   JETSON_PASS=123 ./slam_record_session.sh slam=pointlio bench=true
#   JETSON_PASS=123 ./slam_record_session.sh slam=fastlio tag=corridor_run1 bench=true
#   JETSON_PASS=123 ./slam_record_session.sh stop
#
# Monitor from laptop (no ROS needed — pure SSH):
#   JETSON_PASS=123 ./slam_record_session.sh monitor

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# ── Config ───────────────────────────────────────────────────────────
JETSON_USER="unitree"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
LAPTOP_IP="${GO2W_HOST_IP:-192.168.123.100}"
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
JETSON_WS="/home/unitree/noetic_fastlio_ws"

SLAM_TYPE=""
TAG=""
BAG_DIR="/home/unitree/bags"
WITH_BENCH="false"
NAMESPACE="robot"

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 -o BatchMode=no)

_require_pass() {
  if [[ -z "$JETSON_PASS" ]]; then
    echo "ERROR: JETSON_PASS not set.  export JETSON_PASS=123" >&2; exit 1
  fi
  command -v sshpass &>/dev/null || {
    echo "ERROR: sshpass not installed.  sudo apt install sshpass" >&2; exit 1
  }
}
_ssh()   { sshpass -p "$JETSON_PASS" ssh  "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "$@"; }
_ssh_bg(){ sshpass -p "$JETSON_PASS" ssh  "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "$@" & }

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      _require_pass
      echo "── Stopping all SLAM/bag/bench on Jetson ────────────────────"
      _ssh "
        pkill -INT -f 'rosbag record' 2>/dev/null || true; sleep 3
        pkill -9  -f 'rosbag record' 2>/dev/null || true
        $JETSON_WS/scripts/slam_bench_noetic.sh stop 2>/dev/null || true
        $JETSON_WS/scripts/onboard_fastlio_noetic.sh stop 2>/dev/null || true
        $JETSON_WS/scripts/onboard_pointlio_noetic.sh stop 2>/dev/null || true
        echo DONE
      "
      exit 0
      ;;
    monitor)
      _require_pass
      echo "── Live monitor (Ctrl+C to quit) ─────────────────────────────"
      echo "   Tip: open multiple terminals and run one line each."
      echo ""
      # Run an interactive SSH that loops rostopic hz
      _ssh "
        source /opt/ros/noetic/setup.bash
        source $JETSON_WS/devel/setup.bash 2>/dev/null || true
        export ROS_MASTER_URI=http://\$(hostname -I | awk '{print \$1}'):11311
        export ROS_IP=\$(hostname -I | awk '{print \$1}')
        echo 'ROS_MASTER_URI='\$ROS_MASTER_URI
        echo ''
        echo '=== /robot/Odometry Hz (Ctrl+C to stop) ==='
        rostopic hz /robot/Odometry -w 20
      "
      exit 0
      ;;
    slam=*)     SLAM_TYPE="${arg#slam=}" ;;
    tag=*)      TAG="${arg#tag=}" ;;
    bag_dir=*)  BAG_DIR="${arg#bag_dir=}" ;;
    bench=*)    WITH_BENCH="${arg#bench=}" ;;
    ns=*)       NAMESPACE="${arg#ns=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

_require_pass

if [[ -z "$SLAM_TYPE" ]]; then
  echo "ERROR: slam= required." >&2
  echo "Usage: JETSON_PASS=123 $0 slam=fastlio|pointlio [tag=X] [bench=true]" >&2
  exit 1
fi
case "$SLAM_TYPE" in fastlio|pointlio) ;;
  *) echo "ERROR: slam must be fastlio or pointlio" >&2; exit 1 ;;
esac
case "$WITH_BENCH" in true|false) ;;
  *) echo "ERROR: bench must be true|false" >&2; exit 1 ;;
esac

# ── Step 1: Ethernet link ─────────────────────────────────────────────
echo ""
echo "── [1/4] Ethernet link ──────────────────────────────────────────"
if ! ip -4 addr show "$ETH_IFACE" 2>/dev/null | grep -q " ${LAPTOP_IP}/"; then
  echo "  Adding ${LAPTOP_IP}/24 to $ETH_IFACE..."
  sudo ip addr add "${LAPTOP_IP}/24" dev "$ETH_IFACE" 2>/dev/null || true
fi
if ip -4 addr show "$ETH_IFACE" 2>/dev/null | grep -q " ${LAPTOP_IP}/"; then
  echo "  $LAPTOP_IP on $ETH_IFACE ✓"
else
  echo "  WARN: could not bind $LAPTOP_IP — cable/dongle issue?" >&2
fi
if ! ping -c 2 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "ERROR: cannot ping Jetson at $JETSON_HOST.  Check cable." >&2; exit 1
fi
echo "  Jetson reachable ✓"

# ── Step 2: Launch SLAM on Jetson (Jetson owns roscore) ───────────────
echo ""
echo "── [2/4] Launching $SLAM_TYPE on Jetson ────────────────────────"
RECORD_TAG="${SLAM_TYPE}${TAG:+_${TAG}}"
# Kill any stale sessions first
_ssh "
  $JETSON_WS/scripts/onboard_fastlio_noetic.sh stop 2>/dev/null || true
  $JETSON_WS/scripts/onboard_pointlio_noetic.sh stop 2>/dev/null || true
  pkill -9 -f 'rosbag record' 2>/dev/null || true
  sleep 1
" 2>/dev/null || true

# Start SLAM in a detached background SSH (nohup + disown on Jetson side)
_ssh "
  nohup bash -c '
    source /opt/ros/noetic/setup.bash
    source $JETSON_WS/devel/setup.bash
    $JETSON_WS/scripts/onboard_${SLAM_TYPE}_noetic.sh
  ' >/tmp/slam_session_${SLAM_TYPE}.log 2>&1 &
  disown \$!
  echo \"SLAM started PID=\$!\"
"
echo "  SLAM launched (log: ssh jetson 'cat /tmp/slam_session_${SLAM_TYPE}.log')"

# Poll until /robot/Odometry appears
echo "  Waiting for /${NAMESPACE}/Odometry to come up..."
for i in $(seq 1 30); do
  ODOM_UP=$(_ssh "
    source /opt/ros/noetic/setup.bash
    source $JETSON_WS/devel/setup.bash 2>/dev/null
    export ROS_MASTER_URI=http://\$(hostname -I | awk '{print \$1}'):11311
    export ROS_IP=\$(hostname -I | awk '{print \$1}')
    rostopic info /${NAMESPACE}/Odometry 2>/dev/null | grep -c 'Publishers:' || echo 0
  " 2>/dev/null | tr -d '\n' || echo 0)
  if [[ "${ODOM_UP}" -ge 1 ]]; then
    echo "  /${NAMESPACE}/Odometry up ✓"
    break
  fi
  printf "\r  waiting... %ds" $(( i * 2 ))
  sleep 2
done

# ── Step 3: rosbag on Jetson ──────────────────────────────────────────
echo ""
echo "── [3/4] Starting rosbag on Jetson ─────────────────────────────"
_ssh "
  nohup bash -c '
    source /opt/ros/noetic/setup.bash
    source $JETSON_WS/devel/setup.bash
    export ROS_MASTER_URI=http://\$(hostname -I | awk \"{print \\\$1}\"):11311
    export ROS_IP=\$(hostname -I | awk \"{print \\\$1}\")
    $JETSON_WS/scripts/onboard_record_noetic.sh tag=${RECORD_TAG} bag_dir=${BAG_DIR} ns=${NAMESPACE}
  ' >/tmp/slam_session_record.log 2>&1 &
  disown \$!
  echo \"rosbag started\"
"
echo "  rosbag started"

# ── Step 4: slam_bench (optional) ─────────────────────────────────────
echo ""
if [[ "$WITH_BENCH" == "true" ]]; then
  echo "── [4/4] Starting slam_bench on Jetson ──────────────────────────"
  _ssh "
    nohup bash -c '
      source /opt/ros/noetic/setup.bash
      source $JETSON_WS/devel/setup.bash
      export ROS_MASTER_URI=http://\$(hostname -I | awk \"{print \\\$1}\"):11311
      export ROS_IP=\$(hostname -I | awk \"{print \\\$1}\")
      $JETSON_WS/scripts/slam_bench_noetic.sh slam=${SLAM_TYPE} ns=${NAMESPACE}
    ' >/tmp/slam_session_bench.log 2>&1 &
    disown \$!
    echo \"slam_bench started\"
  "
  echo "  slam_bench started"
else
  echo "── [4/4] bench=false — skipping ────────────────────────────────"
fi

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Session running — slam=$SLAM_TYPE  bench=$WITH_BENCH"
echo "  Walk the robot.  Ctrl+C to stop + save bag."
echo ""
echo "  Monitor (open separate terminal):"
echo "    JETSON_PASS=$JETSON_PASS $0 monitor"
echo "    — or —"
echo "    ssh ${JETSON_USER}@${JETSON_HOST} 'bash -c \""
echo "      source /opt/ros/noetic/setup.bash &&"
echo "      export ROS_MASTER_URI=http://\$(hostname -I | awk \"{print \\\$1}\"):11311 &&"
echo "      rostopic hz /${NAMESPACE}/Odometry\"'"
echo "################################################"
echo ""

# ── Ctrl+C handler ────────────────────────────────────────────────────
_cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "── Stopping session ─────────────────────────────────────────────"

  echo "  1. SIGINT rosbag (waiting 10s for bag footer)..."
  _ssh "pkill -INT -f 'rosbag record' 2>/dev/null || true" 2>/dev/null || true
  sleep 10
  _ssh "pkill -9 -f 'rosbag record' 2>/dev/null || true" 2>/dev/null || true

  if [[ "$WITH_BENCH" == "true" ]]; then
    echo "  2. Stopping slam_bench..."
    _ssh "pkill -INT -f 'slam_bench_noetic' 2>/dev/null || true
          pkill -INT -f 'tegrastats' 2>/dev/null || true
          pkill -INT -f 'rostopic' 2>/dev/null || true
          sleep 2
          pkill -9 -f 'slam_bench_noetic' 2>/dev/null || true
          pkill -9 -f 'tegrastats' 2>/dev/null || true" 2>/dev/null || true
  fi

  echo "  3. Stopping $SLAM_TYPE..."
  _ssh "$JETSON_WS/scripts/onboard_${SLAM_TYPE}_noetic.sh stop 2>/dev/null || true" 2>/dev/null || true

  echo ""
  echo "  Bags saved on Jetson:"
  _ssh "ls -lh ${BAG_DIR}/*.bag ${BAG_DIR}/*.bag.active 2>/dev/null \
        | awk '{print \"    \" \$NF \"  \" \$5}' \
        || echo '    (check ${BAG_DIR}/)'" 2>/dev/null || true
  echo ""
  echo "  Rsync to laptop:"
  echo "    sshpass -p $JETSON_PASS rsync -avP ${JETSON_USER}@${JETSON_HOST}:${BAG_DIR}/ ~/bags/jetson/"
  echo ""
  exit 0
}
trap _cleanup INT TERM

# ── Keep alive + live Hz display ─────────────────────────────────────
START_T=$SECONDS
while true; do
  ELAPSED=$(( SECONDS - START_T ))
  # Grab latest Hz from rosbag's record log as a proxy (avoids needing ROS locally)
  ODOM_STATUS=$(_ssh "
    source /opt/ros/noetic/setup.bash 2>/dev/null
    source $JETSON_WS/devel/setup.bash 2>/dev/null
    export ROS_MASTER_URI=http://\$(hostname -I | awk '{print \$1}'):11311
    export ROS_IP=\$(hostname -I | awk '{print \$1}')
    timeout 3 rostopic hz /${NAMESPACE}/Odometry -w 10 2>/dev/null \
      | grep 'average rate' | tail -1 | grep -oP '[\d.]+(?= Hz)' || echo '?'
  " 2>/dev/null | tr -d '\n' || echo "?")
  printf "\r  t=%4ds  /${NAMESPACE}/Odometry: %-6s Hz  [Ctrl+C to stop+save]" \
    "$ELAPSED" "$ODOM_STATUS"
  sleep 5
done
