#!/usr/bin/env bash
# run_jetson_bag_test.sh — Jetson real-time verification via Noetic bag replay.
#
# Replays a real Go2 walk bag at wall-clock 1.0x; downstream autonomy stack
# (elevation_mapping + grid_map_to_occupancy + Nav2) processes it without
# /clock or sim_time. Pass criterion: ros2 bag play's "real-time factor"
# stays ≥0.95x throughout, no falling behind.
#
# Use:
#   bash /tmp/run_jetson_bag_test.sh
#
# Then in another terminal on the Jetson:
#   ros2 bag play /tmp/bag_test/onboard_noetic_*_ops2_ros2_raw --rate 1.0

# WALL-CLOCK MODE: no sim_time, no /clock subscription
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export PATH=$HOME/.local/bin:/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export ELEVATION_MAPPING_FORCE_CUPY=1

source /opt/ros/humble/setup.bash
source /home/johnpork233/jetson_ws/install/setup.bash

# ─────────────────────────────────────────────────────────────────────
# Preflight kill — same patterns as run_jetson_hil.sh
# ─────────────────────────────────────────────────────────────────────
_PREFLIGHT_PATTERNS=(
  "ros2 launch .*orin_nano"
  "ros2 launch"
  "fastlio_mapping" "fast_lio_tf_adapter" "fast_lio_tf_adapter.py"
  "elevation_mapping_node" "filter_chain_runner" "grid_map_to_occupancy_grid"
  "controller_server" "planner_server" "behavior_server" "bt_navigator"
  "lifecycle_manager_navigation"
  "static_transform_publisher"
  "ros2 bag play"
)

_ALIVE_RE='fastlio_mapping|fast_lio_tf_adapter|elevation_mapping_node|filter_chain_runner|grid_map_to_occupancy_grid|controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager_navigation|ros2 bag'

if pgrep -f "${_ALIVE_RE}" >/dev/null 2>&1; then
  echo "[preflight] killing stale procs..."
  for pat in "${_PREFLIGHT_PATTERNS[@]}"; do
    pkill -KILL -f "${pat}" 2>/dev/null || true
  done
  sleep 1
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null || true
  pid=$(pgrep -af "ros2cli.*daemon" 2>/dev/null | awk '{print $1}' | head -1)
  [[ -n "${pid}" ]] && kill -9 "${pid}" 2>/dev/null || true
  echo "[preflight] clean."
else
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null || true
fi

LAUNCH=/tmp/orin_nano_bag_test.launch.py
[[ -f "$LAUNCH" ]] || { echo "ERROR: launch file not at $LAUNCH" >&2; exit 1; }

echo "═══════════════════════════════════════════════════════════════════"
echo "  Orin Nano real-time bag test (WALL-CLOCK, no sim_time)"
echo "    elevation_mapping_cupy + grid_map_to_occupancy_grid + Nav2"
echo "    bag input: /robot/cloud_registered_body + /livox/imu + /tf"
echo "  ELEVATION_MAPPING_FORCE_CUPY=$ELEVATION_MAPPING_FORCE_CUPY"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""
echo "  After this stack is up, replay the bag in another terminal:"
echo "    ros2 bag play /tmp/bag_test/onboard_noetic_20260511_155920_ops2_ros2_raw \\"
echo "      --rate 1.0 --topics /robot/cloud_registered_body /livox/imu /tf /tf_static"
echo "═══════════════════════════════════════════════════════════════════"

exec ros2 launch "$LAUNCH"
