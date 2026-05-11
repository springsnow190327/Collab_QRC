#!/usr/bin/env bash
# onboard_record.sh — record Livox raw + Fast-LIO outputs to a bag on the Jetson.
#
# Sister script to onboard_slam.sh: same env setup (Foxy + FastDDS UDP-only,
# ulimit -v cap, Mid-360 NIC bind) but the goal is a standalone DATA COLLECTION
# run — no laptop nav, no CFPA2, no cmd_vel. Walk the robot via the BT pad and
# capture everything to a bag on Jetson local disk (avoids cross-host UDP
# dropouts that the laptop-side record_livox_dataset.sh is exposed to).
#
# What it brings up:
#   1. Mid-360 NIC bind (192.168.123.100 on Jetson's Ethernet, required by
#      MID360_config.json — same as onboard_slam.sh)
#   2. FastDDS UDP-only + ulimit -v 1500000 (Foxy/Tegra OOM workaround)
#   3. livox_ros_driver2_node      → /livox/lidar (CustomMsg) + /livox/imu
#   4. (optional) fastlio_mapping  → /Odometry + /cloud_registered{,_body} + /path
#   5. (optional) static TFs       → map→camera_init, body→base_link, map→odom
#   6. ros2 bag record (sqlite3, 2 GB split) → $BAG_DIR
#
# Topics recorded (always):
#   /livox/lidar  /livox/imu                            ← raw sensor
# With fastlio=true (default):
#   /Odometry  /cloud_registered  /cloud_registered_body  /path  /tf  /tf_static
#
# Usage (run on Jetson):
#   ./onboard_record.sh                                # default: with fastlio, sqlite3
#   ./onboard_record.sh fastlio=false                  # raw lidar+imu only
#   ./onboard_record.sh tag=corridor_run1
#   ./onboard_record.sh bag_dir=/data/bags
#   ./onboard_record.sh namespace=robot_b              # /tf remap awareness
#   ./onboard_record.sh stop                           # tear down everything
#
# Replay (later, at the desk after rsync back):
#   ros2 bag play <bag_dir> --clock
#
# Ctrl+C exits cleanly via trap (bag finalized first, then publishers).

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"   # /home/unitree/onboard_ws

# ── Defaults ─────────────────────────────────────────────────────────
ROS_DISTRO="foxy"
NAMESPACE="robot"
LIVOX_HOST_IP="192.168.123.100"
LIVOX_NIC=""
DOMAIN_ID="0"
WITH_FASTLIO="true"
TAG=""
BAG_DIR_OVERRIDE=""
BAG_ROOT_DEFAULT="/home/unitree/bags"
BAG_STORAGE="${BAG_STORAGE:-sqlite3}"

# ── Cleanup ──────────────────────────────────────────────────────────
LIVOX_PID=""
FASTLIO_PID=""
BAG_PID=""
TF_STATICS_PIDS=()

_kill_record_stack() {
  # Bag first so MCAP/sqlite3 footer is written.
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo "  → SIGINT rosbag2 (waiting up to 8s for finalize)"
    kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8; do
      kill -0 "$BAG_PID" 2>/dev/null || break
      sleep 1
    done
  fi
  for pid in "$FASTLIO_PID" "$LIVOX_PID" "${TF_STATICS_PIDS[@]}"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  pkill -INT -f "ros2 bag record" 2>/dev/null || true
  pkill -9 -f livox_ros_driver2_node 2>/dev/null || true
  pkill -9 -f fastlio_mapping 2>/dev/null || true
  pkill -9 -f static_transform_publisher 2>/dev/null || true
  (ros2 daemon stop &>/dev/null &); sleep 1
  pkill -9 -f _ros2_daemon 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping onboard record..."
      _kill_record_stack
      echo "Done."
      exit 0
      ;;
    livox_host=*)    LIVOX_HOST_IP="${arg#livox_host=}" ;;
    nic=*)           LIVOX_NIC="${arg#nic=}" ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    domain=*)        DOMAIN_ID="${arg#domain=}" ;;
    distro=*)        ROS_DISTRO="${arg#distro=}" ;;
    fastlio=*)       WITH_FASTLIO="${arg#fastlio=}" ;;
    tag=*)           TAG="${arg#tag=}" ;;
    bag_dir=*)       BAG_DIR_OVERRIDE="${arg#bag_dir=}" ;;
    storage=*)       BAG_STORAGE="${arg#storage=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Validate ─────────────────────────────────────────────────────────
case "$WITH_FASTLIO" in true|false) ;; *) echo "ERROR: fastlio must be true|false" >&2; exit 1 ;; esac
case "$ROS_DISTRO"   in foxy|humble) ;; *) echo "ERROR: distro must be foxy|humble" >&2; exit 1 ;; esac

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
  echo "  Adding ${LIVOX_HOST_IP}/24 as secondary on $LIVOX_NIC (Mid-360 bind)..."
  sudo -n ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || \
    sudo ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || {
    echo "  WARN: could not add ${LIVOX_HOST_IP}. Mid-360 bind() will fail." >&2
    echo "        Run: sudo ip addr add ${LIVOX_HOST_IP}/24 dev $LIVOX_NIC" >&2
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

# ── ROS env (same Foxy/FastDDS workaround as onboard_slam.sh) ────────
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS_ROOT/install/setup.bash" ]] && source "$WS_ROOT/install/setup.bash"
[[ -f "$WS_ROOT/src/livox_ros_driver2/install/setup.bash" ]] && \
  source "$WS_ROOT/src/livox_ros_driver2/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"

if [[ "$ROS_DISTRO" == "foxy" ]]; then
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI
  # Foxy/aarch64 vmem cap — see onboard_slam.sh:142-153 for the full why.
  ulimit -v 1500000

  mkdir -p /tmp/dds_cfg
  cat > /tmp/dds_cfg/fastdds_no_shm.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/dds_cfg/fastdds_no_shm.xml
else
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
fi

# ── Bag output path ──────────────────────────────────────────────────
BAG_ROOT="${BAG_DIR_OVERRIDE:-$BAG_ROOT_DEFAULT}"
mkdir -p "$BAG_ROOT" || { echo "ERROR: cannot mkdir $BAG_ROOT" >&2; exit 1; }
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="onboard_${STAMP}"
[[ -n "$TAG" ]] && BAG_NAME="${BAG_NAME}_${TAG//[^A-Za-z0-9_-]/_}"
BAG_DIR="${BAG_ROOT}/${BAG_NAME}"

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Onboard RECORD (Jetson @ $(hostname))"
echo "    distro      : $ROS_DISTRO"
echo "    namespace   : $NAMESPACE"
echo "    fastlio     : $WITH_FASTLIO"
echo "    livox host  : $LIVOX_HOST_IP on $LIVOX_NIC"
echo "    bag_dir     : $BAG_DIR"
echo "    storage     : $BAG_STORAGE"
echo "    tag         : ${TAG:-<none>}"
echo "  Walk the robot via the BT pad."
echo "  Stop          : Ctrl+C  or  ./onboard_record.sh stop"
echo "################################################"
echo ""

# ── Trap ─────────────────────────────────────────────────────────────
cleanup_on_signal() {
  trap - INT TERM EXIT
  echo ""
  echo "Caught interrupt — finalizing bag + tearing down..."
  _kill_record_stack
  if [[ -d "$BAG_DIR" ]]; then
    echo "  Recorded:  $BAG_DIR"
    command -v du >/dev/null && echo "  Size    :  $(du -sh "$BAG_DIR" | cut -f1)"
    echo "  rsync back to laptop, then replay with:"
    echo "    ros2 bag play \"$BAG_DIR\" --clock"
  fi
  exit 0
}
trap cleanup_on_signal INT TERM

# Reset any leftover state from previous runs.
_kill_record_stack 2>/dev/null || true
sleep 1

# ── 1. livox_ros_driver2 ─────────────────────────────────────────────
LIVOX_CFG="$WS_ROOT/config/slam/MID360_config.json"
[[ -f "$LIVOX_CFG" ]] || { echo "ERROR: missing $LIVOX_CFG (run deploy_to_jetson.sh first)" >&2; exit 1; }

echo "[1/4] Starting livox_ros_driver2..."
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=body \
  -p lvx_file_path:='""' \
  -p user_config_path:="$LIVOX_CFG" \
  -p cmdline_input_bd_code:=livox0000000001 \
  </dev/null >/tmp/onboard_record_livox.log 2>&1 &
LIVOX_PID=$!
echo "       PID=$LIVOX_PID (log: /tmp/onboard_record_livox.log)"

sleep 4

# ── 2. fastlio_mapping + static TFs (optional) ───────────────────────
if [[ "$WITH_FASTLIO" == "true" ]]; then
  FASTLIO_CFG="$WS_ROOT/config/slam/fastlio_mid360.yaml"
  [[ -f "$FASTLIO_CFG" ]] || { echo "ERROR: missing $FASTLIO_CFG" >&2; cleanup_on_signal; }

  echo "[2/4] Starting fastlio_mapping (drift capture)..."
  ros2 run fast_lio fastlio_mapping --ros-args \
    -r __node:=fastlio_mapping \
    -r /Odometry:="/${NAMESPACE}/odom/nav" \
    --params-file "$FASTLIO_CFG" \
    -p use_sim_time:=false \
    </dev/null >/tmp/onboard_record_fastlio.log 2>&1 &
  FASTLIO_PID=$!
  echo "       PID=$FASTLIO_PID (log: /tmp/onboard_record_fastlio.log)"

  # Same 3 static TFs onboard_slam.sh publishes (Foxy positional syntax).
  ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0.263591 -0.036809 map camera_init \
    --ros-args -r __node:=map_to_camera_init \
    </dev/null >/tmp/onboard_record_tf_map_camera_init.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 -0.263591 0.036809 body base_link \
    --ros-args -r __node:=body_to_base_link_fastlio \
    </dev/null >/tmp/onboard_record_tf_body_base_link.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 map odom \
    --ros-args -r __node:=map_to_odom_identity \
    </dev/null >/tmp/onboard_record_tf_map_odom.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  sleep 2
else
  echo "[2/4] fastlio=false — recording raw lidar+imu only."
fi

# ── 3. Verify /livox/lidar is alive before recording ─────────────────
echo "[3/4] Verifying /livox/lidar..."
PUB_SEEN="false"
for i in 1 2 3 4 5 6 7 8 9 10; do
  if ros2 topic info /livox/lidar 2>/dev/null | grep -qE "Publisher count: [1-9]"; then
    echo "       publisher detected."
    PUB_SEEN="true"
    break
  fi
  sleep 1
done
if [[ "$PUB_SEEN" != "true" ]]; then
  echo "WARN: no publisher on /livox/lidar after 10s. Driver may be slow." >&2
  echo "      Last 15 lines of driver log:" >&2
  tail -15 /tmp/onboard_record_livox.log | sed 's/^/        /' >&2
fi

echo "       waiting up to 30s for first /livox/lidar message..."
if timeout 30 ros2 topic echo --once /livox/lidar > /dev/null 2>&1; then
  echo "       first message received — Mid-360 streaming."
else
  echo "WARN: no /livox/lidar message in 30s. Bag will still record; check size." >&2
  echo "      Common causes: Mid-360 power, IP (ping 192.168.123.20), FastDDS profile." >&2
fi

# ── 4. ros2 bag record ───────────────────────────────────────────────
TOPICS=(
  /livox/lidar
  /livox/imu
)
if [[ "$WITH_FASTLIO" == "true" ]]; then
  TOPICS+=(
    /${NAMESPACE}/odom/nav
    /cloud_registered
    /cloud_registered_body
    /path
    /tf
    /tf_static
  )
fi

# Manifest written next to bag dir, moved inside once rosbag2 creates the dir.
MANIFEST_PENDING="${BAG_ROOT}/.${BAG_NAME}.manifest"
{
  echo "kind=onboard_record"
  echo "fastlio_running_during_record=$WITH_FASTLIO"
  echo "namespace=$NAMESPACE"
  echo "tag=$TAG"
  echo "stamp=$STAMP"
  echo "hostname=$(hostname)"
  echo "user=$(whoami)"
  echo "ros_distro=$ROS_DISTRO"
  echo "date=$(date -Iseconds)"
  echo "topics=${TOPICS[*]}"
  echo "fastlio_yaml=${FASTLIO_CFG:-<n/a>}"
  echo "mid360_json=$LIVOX_CFG"
  echo "args=$*"
} > "$MANIFEST_PENDING"

RECORD_LOG_PENDING="${BAG_ROOT}/.${BAG_NAME}.record.log"
echo "[4/4] ros2 bag record (${#TOPICS[@]} topics, ${BAG_STORAGE}, 2 GB split)..."
ros2 bag record \
  --output "$BAG_DIR" \
  --storage "$BAG_STORAGE" \
  --max-bag-size 2147483648 \
  "${TOPICS[@]}" \
  > "$RECORD_LOG_PENDING" 2>&1 &
BAG_PID=$!
echo "       PID=$BAG_PID"

# Fail loud if rosbag2 dies within 2s (bad storage plugin etc).
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag2 died within 2s of startup. Recording aborted." >&2
  echo "       record.log:" >&2
  sed 's/^/        /' "$RECORD_LOG_PENDING" >&2
  rm -f "$MANIFEST_PENDING" "$RECORD_LOG_PENDING"
  cleanup_on_signal
fi

for _ in 1 2 3 4 5 6 7 8 9 10; do
  [[ -d "$BAG_DIR" ]] && break
  sleep 0.3
done
if [[ -d "$BAG_DIR" ]]; then
  mv -f "$MANIFEST_PENDING" "$BAG_DIR/manifest.txt"
  mv -f "$RECORD_LOG_PENDING" "$BAG_DIR/record.log" 2>/dev/null || true
else
  echo "WARN: rosbag2 didn't create $BAG_DIR within 3s — manifest left at $MANIFEST_PENDING" >&2
fi
echo ""

# ── Status loop ──────────────────────────────────────────────────────
echo "RECORDING.  Walk the robot through the scene."
echo "Press Ctrl+C to stop and finalize the bag."
echo ""
START_T=$SECONDS
while kill -0 "$BAG_PID" 2>/dev/null; do
  ELAPSED=$(( SECONDS - START_T ))
  if [[ -d "$BAG_DIR" ]]; then
    SIZE=$(du -sb "$BAG_DIR" 2>/dev/null | cut -f1)
    SIZE_HUMAN=$(numfmt --to=iec --suffix=B "${SIZE:-0}" 2>/dev/null || echo "?B")
  else
    SIZE_HUMAN="(not yet)"
  fi
  printf "\r  t=%4ds  size=%-8s  livox=%s  fastlio=%s  " \
    "$ELAPSED" "$SIZE_HUMAN" \
    "$(kill -0 "$LIVOX_PID" 2>/dev/null && echo OK || echo DEAD)" \
    "$([[ "$WITH_FASTLIO" == "true" ]] && (kill -0 "$FASTLIO_PID" 2>/dev/null && echo OK || echo DEAD) || echo OFF)"
  sleep 2
done

echo ""
echo "WARN: rosbag2 exited unexpectedly. Cleaning up..."
cleanup_on_signal
