#!/usr/bin/env bash
# onboard_pointlio_noetic.sh — bring up Mid-360 + Point-LIO in ROS 1 Noetic on
# the Jetson.  Sister to onboard_slam.sh (Foxy) — same Mid-360 NIC bind logic,
# but no FastDDS profile or ulimit workaround (ROS 1 uses TCP/XML-RPC and
# doesn't trigger the Tegra/FastDDS speculative-mmap OOM).
#
# Why this exists:  gbplanner3 lives in ROS 1 Noetic and consumes Fast-LIO's
# /Odometry + /cloud_registered_body at 5-10 MB/s.  Running Fast-LIO native
# in Noetic eliminates ros1_bridge on the high-bandwidth SLAM stream; only
# the tiny /pci_command_path PoseArray still needs to cross to ROS 2.
#
# Topic layout (default namespace=robot):
#   /livox/lidar               (CustomMsg, livox driver — global)
#   /livox/imu                 (Imu      , livox driver — global)
#   /robot/Odometry            (Odometry from FAST-LIO, namespaced)
#   /robot/cloud_registered    (PointCloud2 — map frame)
#   /robot/cloud_registered_body (PointCloud2 — body frame, voxblox-ready)
#
# Static TFs published (matching onboard_slam.sh Foxy convention):
#   map → camera_init      (Mid-360 mount tilt: pitch +0.263591, roll -0.036809)
#   body → base_link       (inverse of above)
#   map → odom             (identity — for gbplanner / voxblox that expect it)
#
# Usage (run on Jetson, after deploy_noetic_to_jetson.sh + the two builds):
#   ./onboard_pointlio_noetic.sh                          # default ns=robot
#   ./onboard_pointlio_noetic.sh namespace=robot_b
#   ./onboard_pointlio_noetic.sh rviz=true                # local visualisation
#   ./onboard_pointlio_noetic.sh stop                     # tear down everything
#
# Ctrl+C exits cleanly via trap.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"   # /home/unitree/noetic_fastlio_ws
FOXY_WS_DEFAULT="/home/unitree/onboard_ws"

# ── Strip miniconda from env BEFORE sourcing ROS ─────────────────────
# This Jetson's .bashrc auto-activates `conda activate base`, putting
# /home/unitree/miniconda3/bin AHEAD of /usr/bin in PATH. ROS 1 C++ nodes
# (pointlio_mapping, livox_ros_driver2_node) then segfault before main()
# because the dynamic linker picks miniconda's libstdc++/libpython mismatch
# vs what they were built against. Symptoms: empty ~/.ros/log/*/laserMapping*.log,
# roslaunch prints "No processes to monitor" within ~1 second.
# Fix: deactivate conda (if loaded), scrub PATH, unset Python env vars.
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  echo "  Stripping miniconda from env (was contaminating ROS node link path)..."
  # Try the official deactivate first if the shell fn is loaded.
  if type conda 2>/dev/null | head -1 | grep -q function; then
    conda deactivate 2>/dev/null || true
  fi
  # Belt-and-suspenders: scrub any remaining miniconda entries.
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE "(miniconda|conda)" | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

# ── Defaults ─────────────────────────────────────────────────────────
NAMESPACE="robot"
LIVOX_HOST_IP="192.168.123.100"
LIVOX_NIC=""
ROS_MASTER_PORT="11311"
ENABLE_RVIZ="false"
FOXY_WS="$FOXY_WS_DEFAULT"   # for Livox-SDK2 install reuse (info only)
EXTERNAL_MASTER=""            # e.g. master=http://192.168.123.50:11311 → skip roscore

# ── Cleanup ──────────────────────────────────────────────────────────
_kill_noetic_stack() {
  for p in pointlio_mapping laserMapping livox_ros_driver2_node \
           static_transform_publisher rosout rosmaster; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  pkill -9 -f "roslaunch" 2>/dev/null || true
  pkill -9 -f "roscore" 2>/dev/null || true
  # rosout latches don't have their own process but the master cleans up.
  sleep 1
}

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping onboard Noetic Fast-LIO stack..."
      _kill_noetic_stack
      echo "Done."
      exit 0
      ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    livox_host=*)    LIVOX_HOST_IP="${arg#livox_host=}" ;;
    nic=*)           LIVOX_NIC="${arg#nic=}" ;;
    port=*)          ROS_MASTER_PORT="${arg#port=}" ;;
    rviz=*)          ENABLE_RVIZ="${arg#rviz=}" ;;
    foxy_ws=*)       FOXY_WS="${arg#foxy_ws=}" ;;
    ros_ip=*)        OVERRIDE_ROS_IP="${arg#ros_ip=}" ;;
    master=*)        EXTERNAL_MASTER="${arg#master=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$ENABLE_RVIZ" in true|false) ;; *) echo "ERROR: rviz must be true|false" >&2; exit 1 ;; esac

# ── Mid-360 NIC bind (same logic as onboard_slam.sh) ─────────────────
if [[ -z "$LIVOX_NIC" ]]; then
  LIVOX_NIC="$(ip -o link show | awk -F': ' '$2 ~ /^en|^eth/ {print $2; exit}')"
fi
if [[ -z "$LIVOX_NIC" ]]; then
  echo "ERROR: could not auto-detect Ethernet NIC. Pass nic=eth0 (or similar)." >&2
  exit 1
fi
echo "  Ethernet NIC: $LIVOX_NIC"

if ! ip -4 addr show "$LIVOX_NIC" | grep -q " ${LIVOX_HOST_IP}/"; then
  echo "  Adding ${LIVOX_HOST_IP}/24 as secondary on $LIVOX_NIC..."
  sudo -n ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || \
    sudo ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || {
    echo "  WARN: could not add ${LIVOX_HOST_IP}. Mid-360 bind() will fail." >&2
  }
fi
if ! ip -4 addr show "$LIVOX_NIC" | grep -q " ${LIVOX_HOST_IP}/"; then
  echo "  ERROR: ${LIVOX_HOST_IP} still missing on ${LIVOX_NIC}." >&2
  echo "         Run BEFORE this script:" >&2
  echo "         sudo ip addr add ${LIVOX_HOST_IP}/24 dev ${LIVOX_NIC}" >&2
  exit 1
fi

if ! ping -c 1 -W 2 192.168.123.20 &>/dev/null; then
  echo "WARN: Mid-360 at 192.168.123.20 didn't answer ping. Driver may fail." >&2
fi

# ── ROS 1 env ────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash

# livox_ros_driver2 ROS1 build writes its devel/setup.bash at the WORKSPACE
# root after `./build.sh ROS1` (catkin_make under the hood) — same path as
# the FAST-LIO catkin build.  One devel/setup.bash overlays both.
if [[ -f "$WS_ROOT/devel/setup.bash" ]]; then
  source "$WS_ROOT/devel/setup.bash"
else
  echo "ERROR: ${WS_ROOT}/devel/setup.bash not found." >&2
  echo "       Build first:  cd ${WS_ROOT}/src/livox_ros_driver2 && ./build.sh ROS1" >&2
  echo "                     cd ${WS_ROOT} && catkin_make -DCMAKE_BUILD_TYPE=Release" >&2
  exit 1
fi

# rospack sanity check — both packages must be findable.
if ! rospack find point_lio &>/dev/null; then
  echo "ERROR: rospack can't find 'point_lio'. catkin build incomplete?" >&2
  exit 1
fi
if ! rospack find livox_ros_driver2 &>/dev/null; then
  echo "ERROR: rospack can't find 'livox_ros_driver2'. ./build.sh ROS1 incomplete?" >&2
  exit 1
fi

if [[ -n "${OVERRIDE_ROS_IP:-}" ]]; then
  export ROS_IP="$OVERRIDE_ROS_IP"
else
  export ROS_IP="$(hostname -I | awk '{print $1}')"
fi

if [[ -n "$EXTERNAL_MASTER" ]]; then
  export ROS_MASTER_URI="$EXTERNAL_MASTER"
  echo "  External master: $ROS_MASTER_URI (roscore NOT started here)"
else
  export ROS_MASTER_URI="http://${ROS_IP}:${ROS_MASTER_PORT}"
fi

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Noetic Point-LIO (Jetson @ $(hostname))"
echo "    namespace   : $NAMESPACE   → /${NAMESPACE}/Odometry, /${NAMESPACE}/cloud_registered{,_body}"
echo "    livox topic : /livox/lidar  /livox/imu  (un-namespaced)"
echo "    ROS_MASTER  : $ROS_MASTER_URI"
echo "    ROS_IP      : $ROS_IP"
echo "    rviz        : $ENABLE_RVIZ"
echo "  Stop          : Ctrl+C  or  ./onboard_pointlio_noetic.sh stop
  Remote RViz   : on laptop run scripts/real/rviz_view_onboard_fastlio.sh
                  (uses ssh -X to render RViz on Jetson, display on laptop)"
echo "################################################"
echo ""

# ── Trap ─────────────────────────────────────────────────────────────
cleanup_on_signal() {
  trap - INT TERM EXIT
  echo ""
  echo "Caught interrupt — tearing down Noetic stack..."
  _kill_noetic_stack
  echo "Done."
  exit 0
}
trap cleanup_on_signal INT TERM

# Reset shm + any prior nodes.
_kill_noetic_stack 2>/dev/null || true
sleep 1

# ── 1. roscore (skipped when laptop is the master) ───────────────────
if [[ -n "$EXTERNAL_MASTER" ]]; then
  echo "[1/5] External master ($EXTERNAL_MASTER) — waiting for it to be reachable..."
  for i in $(seq 1 15); do
    if rostopic list &>/dev/null; then
      echo "      master reachable."
      break
    fi
    sleep 1
  done
  if ! rostopic list &>/dev/null; then
    echo "ERROR: cannot reach master at $ROS_MASTER_URI after 15s." >&2
    echo "       Make sure roscore is running on the laptop." >&2
    cleanup_on_signal
  fi
else
  echo "[1/5] Starting roscore on port ${ROS_MASTER_PORT}..."
  setsid -f roscore -p "$ROS_MASTER_PORT" </dev/null >/tmp/roscore.log 2>&1
  for i in 1 2 3 4 5 6 7 8 9 10; do
    if rostopic list &>/dev/null; then
      echo "      roscore ready."
      break
    fi
    sleep 1
  done
  if ! rostopic list &>/dev/null; then
    echo "ERROR: roscore didn't come up in 10s." >&2
    echo "       Log: /tmp/roscore.log" >&2
    cleanup_on_signal
  fi
fi

# ── 2. livox_ros_driver2 (ROS 1, MID-360) ────────────────────────────
echo "[2/5] Starting livox_ros_driver2 (ROS 1, Mid-360)..."
# The msg_MID360.launch wires xfer_format=0 by default (PointCloud2 + Imu).
# FAST-LIO wants xfer_format=1 (livox_ros_driver2/CustomMsg) — override.
# rviz_enable=false to avoid X dependency on a headless Jetson.
# IMPORTANT: do NOT use `setsid -f` on roslaunch — if a node crashes during
# init (LD path issue, missing config, etc.), setsid loses the stderr pipe
# and we get empty ~/.ros/log/*.log + a misleading "No processes to monitor"
# in the parent log with no actual cause. Background `&` keeps the pipe alive.
nohup roslaunch livox_ros_driver2 msg_MID360.launch \
  xfer_format:=1 \
  multi_topic:=0 \
  publish_freq:=10.0 \
  output_type:=0 \
  msg_frame_id:=body \
  rviz_enable:=false \
  rosbag_enable:=false \
  </dev/null >/tmp/livox_noetic.log 2>&1 &
LIVOX_LAUNCH_PID=$!
disown $LIVOX_LAUNCH_PID 2>/dev/null || true
echo "      PID=$LIVOX_LAUNCH_PID  log: /tmp/livox_noetic.log"

# Wait for /livox/lidar publisher.
for i in 1 2 3 4 5 6 7 8 9 10; do
  if rostopic info /livox/lidar 2>/dev/null | grep -q "Publishers:"; then
    echo "      /livox/lidar advertised."
    break
  fi
  sleep 1
done

# ── 3. Point-LIO ─────────────────────────────────────────────────────
# Wrap fast_lio under ns=$NAMESPACE so its outputs land at /${NAMESPACE}/...
# The mid360.yaml config's lid_topic/imu_topic ("/livox/lidar", "/livox/imu")
# are ABSOLUTE so they remain un-namespaced — subscription to the un-prefixed
# livox driver topics works as-is.
echo "[3/5] Starting Point-LIO (mapping_mid360, ns=${NAMESPACE})..."
ROS_NAMESPACE="$NAMESPACE" nohup \
  roslaunch point_lio mapping_mid360.launch \
    rviz:=false \
    </dev/null >/tmp/pointlio_noetic.log 2>&1 &
FASTLIO_LAUNCH_PID=$!
disown $FASTLIO_LAUNCH_PID 2>/dev/null || true
echo "      PID=$FASTLIO_LAUNCH_PID  log: /tmp/pointlio_noetic.log"

# Wait for /<ns>/Odometry.
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
  if rostopic info "/${NAMESPACE}/Odometry" 2>/dev/null | grep -q "Publishers:"; then
    echo "      /${NAMESPACE}/Odometry up."
    break
  fi
  sleep 1
done

# ── 4. Static TFs (match onboard_slam.sh Foxy convention) ────────────
# Mid-360 mount tilt: pitch +0.263591, roll -0.036809 (measured 2026-04-17).
echo "[4/5] Publishing static TFs..."
# Noetic static_transform_publisher: x y z yaw pitch roll parent child period_ms
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 0.263591 -0.036809 map camera_init \
  </dev/null >/tmp/tf_map_camera_init.log 2>&1
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 -0.263591 0.036809 body base_link \
  </dev/null >/tmp/tf_body_base_link.log 2>&1
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom \
  </dev/null >/tmp/tf_map_odom.log 2>&1

# ── 5. Optional RViz ─────────────────────────────────────────────────
if [[ "$ENABLE_RVIZ" == "true" ]]; then
  echo "[5/5] Starting RViz (loam_livox.rviz config from fast_lio)..."
  RVIZ_CFG="$(rospack find point_lio)/rviz_cfg/loam_livox.rviz"
  setsid -f rviz -d "$RVIZ_CFG" </dev/null >/tmp/rviz_noetic.log 2>&1
else
  echo "[5/5] rviz=false — skipping."
fi

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "  All nodes detached. Logs at /tmp/{roscore,livox_noetic,fastlio_noetic,tf_*}.log"
echo ""
echo "  Verify (this shell, or from another SSH session with same ROS_MASTER_URI):"
echo "    export ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "    rostopic hz /livox/lidar                # ~10 Hz from Mid-360"
echo "    rostopic hz /${NAMESPACE}/Odometry      # ~10 Hz from FAST-LIO"
echo "    rostopic hz /${NAMESPACE}/cloud_registered_body"
echo "    rosrun tf2_tools view_frames.py         # produces a TF tree PDF"
echo ""
echo "  Stop:   ${WS_ROOT}/scripts/onboard_pointlio_noetic.sh stop"
