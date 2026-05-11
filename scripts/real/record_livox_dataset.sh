#!/usr/bin/env bash
# record_livox_dataset.sh — record-only DRY-RUN for offline Fast-LIO debug.
#
# What it does:
#   1. Brings the laptop's USB-C Ethernet up + configures CycloneDDS for the
#      real Go2/Go2W (same path as real_autonomy.sh).
#   2. Starts the Livox MID-360 driver (xfer_format=1 → /livox/lidar in
#      CustomMsg + /livox/imu) — exactly what Fast-LIO consumes.
#   3. (default on) Starts fastlio_mapping locally so the bag captures the
#      drifting Odometry/cloud_registered output you want to diagnose.
#   4. Runs `ros2 bag record` on a Fast-LIO-replay-focused topic list:
#        /livox/lidar  /livox/imu                        ← raw inputs
#        /Odometry  /cloud_registered  /cloud_registered_body  /path  ← FLIO outputs
#        /tf  /tf_static                                  ← URDF + slam.launch statics
#      Output: $REPO_ROOT/bags/livox_dataset_<stamp>/  (MCAP, 2 GB split).
#   5. NO autonomy: no nav, no CFPA2, no cmd_vel publisher. Robot is fully
#      idle from the laptop's perspective. Walk it through the scene with
#      the Unitree BT pad.
#
# Usage:
#   ./scripts/real/record_livox_dataset.sh
#   ./scripts/real/record_livox_dataset.sh fastlio=false   # raw-only (re-run FLIO offline)
#   ./scripts/real/record_livox_dataset.sh tag=outdoor_park_run1
#   ./scripts/real/record_livox_dataset.sh bag_dir=/path/to/your/dir
#   ./scripts/real/record_livox_dataset.sh stop            # kill any leftover record
#
# Replay (later, at the desk):
#   ros2 bag play <bag_dir> --clock        # publishes /clock + recorded topics
#   ros2 launch fast_lio mapping_mid360.launch.py use_sim_time:=true \
#       <override params from src/go2w/go2w_real_bringup/config/slam/fastlio_mid360.yaml>
# Match real_autonomy.sh: -e only, no -u — sourcing /opt/ros/humble/setup.bash
# trips set -u on AMENT_TRACE_SETUP_FILES (Humble setup.bash isn't strict-clean).
set -eo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# Reuse real_autonomy.sh's stop logic — it already pkills livox + fastlio +
# rosbag2 cleanly with proper SIGINT-first ordering.
if [[ "${1:-}" == "stop" ]]; then
  exec "$SCRIPT_DIR/real_autonomy.sh" stop
fi

# ── Defaults + flags ──────────────────────────────────────────────────
WITH_FASTLIO="true"   # capture FLIO output for diagnostic comparison
TAG=""                # appended to bag dir name
BAG_DIR_OVERRIDE=""

for arg in "$@"; do
  case "$arg" in
    fastlio=*)  WITH_FASTLIO="${arg#fastlio=}" ;;
    tag=*)      TAG="${arg#tag=}" ;;
    bag_dir=*)  BAG_DIR_OVERRIDE="${arg#bag_dir=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$WITH_FASTLIO" in true|false) ;; *) echo "ERROR: fastlio must be true|false" >&2; exit 1 ;; esac

# ── Connection (Ethernet + CycloneDDS) ────────────────────────────────
source "$SCRIPT_DIR/connect_ethernet.sh"
ensure_link
setup_cyclonedds_ethernet

# ── Workspace ─────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# Bag output path
BAG_ROOT="${BAG_DIR_OVERRIDE:-$REPO_ROOT/bags}"
mkdir -p "$BAG_ROOT"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="livox_dataset_${STAMP}"
[[ -n "$TAG" ]] && BAG_NAME="${BAG_NAME}_${TAG//[^A-Za-z0-9_-]/_}"
BAG_DIR="${BAG_ROOT}/${BAG_NAME}"

# ── Banner ────────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Livox dataset RECORD-ONLY  (no autonomy)"
echo "    fastlio  : $WITH_FASTLIO  $([[ "$WITH_FASTLIO" == "true" ]] && echo "(records FLIO drift output too)" || echo "(raw lidar+imu only)")"
echo "    bag_dir  : $BAG_DIR"
echo "    tag      : ${TAG:-<none>}"
echo "  Walk the robot via the BT pad while recording."
echo "  Stop with Ctrl+C — bag is finalized cleanly first."
echo "################################################"
echo ""

# ── Process tracking + cleanup ────────────────────────────────────────
LIVOX_PID=""
FASTLIO_PID=""
BAG_PID=""
TF_STATICS_PIDS=()

cleanup_record_stack() {
  trap - INT TERM EXIT
  echo ""
  echo "Stopping recording..."
  # Stop bag FIRST so MCAP is finalized before the publishers go away.
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
  pkill -9 -f livox_ros_driver2_node 2>/dev/null || true
  pkill -9 -f fastlio_mapping 2>/dev/null || true
  pkill -INT -f "ros2 bag record" 2>/dev/null || true
  echo ""
  if [[ -d "$BAG_DIR" ]]; then
    echo "  Recorded:  $BAG_DIR"
    if command -v du >/dev/null; then
      echo "  Size    :  $(du -sh "$BAG_DIR" | cut -f1)"
    fi
    echo "  Replay  :  ros2 bag play \"$BAG_DIR\" --clock"
  fi
  exit 0
}
trap cleanup_record_stack INT TERM

# ── Livox driver ──────────────────────────────────────────────────────
# xfer_format=1 (Livox CustomMsg). If you also want to compare against
# PointCloud2-format LiDAR, change to xfer_format=0 — but Fast-LIO requires
# xfer_format=1 so leave the default for replay.
MID360_JSON="$REPO_ROOT/install/go2w_real_bringup/share/go2w_real_bringup/config/slam/MID360_config.json"
[[ -f "$MID360_JSON" ]] || MID360_JSON="$REPO_ROOT/src/go2w/go2w_real_bringup/config/slam/MID360_config.json"
if [[ ! -f "$MID360_JSON" ]]; then
  echo "ERROR: MID360_config.json not found at install/ or src/" >&2
  exit 2
fi

echo "[1/4] Starting livox_ros_driver2 (CustomMsg, /livox/lidar + /livox/imu)..."
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=body \
  -p user_config_path:="$MID360_JSON" \
  -p cmdline_input_bd_code:=livox0000000001 \
  > /tmp/record_livox_driver.log 2>&1 &
LIVOX_PID=$!
echo "       livox_ros_driver2 PID=$LIVOX_PID (log: /tmp/record_livox_driver.log)"

# Give the driver a moment to advertise topics before the bag subscribes —
# avoids the first second of bag being empty when topics show up late.
sleep 4

# ── Fast-LIO (optional — captures the drifting output for diagnosis) ──
FASTLIO_YAML="$REPO_ROOT/install/go2w_real_bringup/share/go2w_real_bringup/config/slam/fastlio_mid360.yaml"
[[ -f "$FASTLIO_YAML" ]] || FASTLIO_YAML="$REPO_ROOT/src/go2w/go2w_real_bringup/config/slam/fastlio_mid360.yaml"

if [[ "$WITH_FASTLIO" == "true" ]]; then
  if [[ ! -f "$FASTLIO_YAML" ]]; then
    echo "ERROR: fastlio_mid360.yaml not found." >&2
    cleanup_record_stack
  fi
  echo "[2/4] Starting fastlio_mapping (drift capture)..."
  ros2 run fast_lio fastlio_mapping \
    --ros-args \
    --params-file "$FASTLIO_YAML" \
    -p use_sim_time:=false \
    > /tmp/record_fastlio.log 2>&1 &
  FASTLIO_PID=$!
  echo "       fastlio_mapping PID=$FASTLIO_PID (log: /tmp/record_fastlio.log)"
  # The body→base_link mount-tilt static is part of slam.launch.py, not the
  # mapping node. Add it explicitly so /tf and /tf_static are complete in
  # the bag (replay won't have slam.launch.py running).
  ros2 run tf2_ros static_transform_publisher \
      --frame-id body --child-frame-id base_link \
      --x 0 --y 0 --z 0 --roll 0.036809 --pitch -0.263591 --yaw 0 \
      --ros-args -p use_sim_time:=false \
      > /tmp/record_tf_body_to_base.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  ros2 run tf2_ros static_transform_publisher \
      --frame-id map --child-frame-id camera_init \
      --x 0 --y 0 --z 0 --roll -0.036809 --pitch 0.263591 --yaw 0 \
      --ros-args -p use_sim_time:=false \
      > /tmp/record_tf_map_to_camera_init.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  ros2 run tf2_ros static_transform_publisher \
      --frame-id map --child-frame-id odom \
      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
      --ros-args -p use_sim_time:=false \
      > /tmp/record_tf_map_to_odom.log 2>&1 &
  TF_STATICS_PIDS+=($!)
  sleep 2
fi

# ── Verify livox is publishing before recording starts ────────────────
# Two-stage check, both non-fatal:
#   1) Publisher visible on the topic (DDS discovery completed) — within 10s
#      this confirms the driver advertised /livox/lidar successfully.
#   2) An actual message arrives within 30s. Mid-360 takes 3-5s post-init
#      to start streaming; CycloneDDS discovery via the peer list can add
#      another 2-3s. If no message in 30s, we WARN and continue anyway —
#      bag is already running, operator can see size=0 and decide.
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
  echo "WARN: no publisher on /livox/lidar after 10s. Driver may be slow to advertise." >&2
  echo "      Last 15 lines of driver log:" >&2
  tail -15 /tmp/record_livox_driver.log | sed 's/^/        /' >&2
fi

# Wait up to 30s for an actual message, but don't abort on timeout.
echo "       waiting up to 30s for first /livox/lidar message..."
if timeout 30 ros2 topic echo --once /livox/lidar > /dev/null 2>&1; then
  echo "       first message received — Mid-360 is streaming."
else
  echo "WARN: no /livox/lidar message in 30s." >&2
  echo "      Bag will still record; check bag size and 'ros2 topic hz' manually." >&2
  echo "      Common causes: Mid-360 power, IP (probe 'ping 192.168.123.20')," >&2
  echo "      CycloneDDS peer mismatch." >&2
fi

# ── ros2 bag record ───────────────────────────────────────────────────
TOPICS=(
  /livox/lidar
  /livox/imu
  /tf
  /tf_static
)
if [[ "$WITH_FASTLIO" == "true" ]]; then
  TOPICS+=(
    /Odometry
    /path
    /cloud_registered
    /cloud_registered_body
  )
fi

# Manifest with git + cmdline metadata so the bag is self-describing offline.
# ros2 bag record refuses an existing --output dir, so we stash the manifest
# in $BAG_ROOT and move it into $BAG_DIR after rosbag2 creates it.
MANIFEST_PENDING="${BAG_ROOT}/.${BAG_NAME}.manifest"
{
  echo "kind=livox_dataset_record_only"
  echo "fastlio_running_during_record=$WITH_FASTLIO"
  echo "tag=$TAG"
  echo "stamp=$STAMP"
  echo "git_sha=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_branch=$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty=$(git -C "$REPO_ROOT" diff --quiet 2>/dev/null && echo no || echo yes)"
  echo "hostname=$(hostname)"
  echo "user=$(whoami)"
  echo "date=$(date -Iseconds)"
  echo "topics=${TOPICS[*]}"
  echo "fastlio_yaml=$FASTLIO_YAML"
  echo "mid360_json=$MID360_JSON"
  echo "args=$*"
} > "$MANIFEST_PENDING"

# Storage: sqlite3 (universal, ships with ros-humble-rosbag2). MCAP would be
# nicer (better tooling) but `ros-humble-rosbag2-storage-mcap` apt pkg isn't
# installed on this laptop and we don't have network at the field site.
# Override with BAG_STORAGE=mcap if you've apt-installed the plugin.
BAG_STORAGE="${BAG_STORAGE:-sqlite3}"

RECORD_LOG_PENDING="${BAG_ROOT}/.${BAG_NAME}.record.log"
echo "[4/4] Starting ros2 bag record (${#TOPICS[@]} topics, ${BAG_STORAGE}, 2 GB split)..."
ros2 bag record \
  --output "$BAG_DIR" \
  --storage "$BAG_STORAGE" \
  --max-bag-size 2147483648 \
  "${TOPICS[@]}" \
  > "$RECORD_LOG_PENDING" 2>&1 &
BAG_PID=$!
echo "       ros2 bag PID=$BAG_PID"

# LOUD failure check: ros2 bag record returns non-zero immediately on bad
# args (e.g. unsupported storage plugin) but our `&` makes that silent.
# Confirm the process is still alive 2s in; if not, dump the log and bail.
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag2 died within 2s of startup. Recording aborted." >&2
  echo "       record.log:" >&2
  sed 's/^/        /' "$RECORD_LOG_PENDING" >&2
  rm -f "$MANIFEST_PENDING" "$RECORD_LOG_PENDING"
  cleanup_record_stack
fi

# Wait for rosbag2 to create the dir, then move manifest + log in.
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

# ── Status loop ───────────────────────────────────────────────────────
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

# Bag exited on its own (shouldn't happen normally).
echo ""
echo "WARN: rosbag2 exited unexpectedly. Cleaning up..."
cleanup_record_stack
