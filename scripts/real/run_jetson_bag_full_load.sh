#!/usr/bin/env bash
# run_jetson_bag_full_load.sh — full-load real-time test runner (Jetson side).
#
# Brings up Point-LIO SLAM + elevation_mapping + grid_map_to_occupancy + Nav2
# + CFPA2 (wall-clock, use_sim_time=false). The companion bag-play wrapper
# replays ONLY raw /livox/imu + /livox/lidar at 1.0x wall-clock rate; this
# Jetson stack must absorb that input and produce SLAM + perception + plans
# in real time.
#
# Usage on Jetson:
#   bash /home/johnpork233/jetson_ws/scripts/real/run_jetson_bag_full_load.sh
#
# In a second SSH window, launch bag:
#   ros2 bag play /tmp/bag_test/onboard_noetic_*_ops2_ros2_raw \
#     --topics /livox/imu /livox/lidar \
#     --qos-profile-overrides-path /tmp/bag_qos_overrides.yaml \
#     --clock \
#     --rate 1.0
#
# --clock is REQUIRED: bag messages carry capture-time stamps from 2026-05-11.
# Without /clock + use_sim_time=true, Point-LIO's dynamic TF emit-stamp would
# be in 2026-05-11 while TF lookups use wall-clock NOW → all transforms appear
# stale and lookups silently fail. RTF (real-time factor) is still measured
# against true wall-clock by bag-play itself, independent of /clock setup.
#
# Look at the bag-play output line "real-time factor X.XXx" — that is the
# single fundamental criterion. ≥ 0.95x = real-time. < 0.8x = too heavy.

set -e

WS="/home/johnpork233/jetson_ws"
LAUNCH_FILE="${WS}/scripts/real/orin_nano_bag_full_load.launch.py"

# ── Preflight kill ──────────────────────────────────────────────────────
echo "[preflight] killing stale ROS 2 / Point-LIO / Nav2 / CFPA2 processes..."

PATTERNS=(
  "ros2 launch"
  "ros2 bag play"
  "rosbag2_player"
  "rosbag"
  "pointlio_mapping"
  "fastlio_mapping"
  "fast_lio_tf_adapter"
  "cfpa2_to_nav2_bridge"
  "cfpa2_single_robot_node"
  "elevation_mapping_node"
  "grid_map_to_occupancy_grid"
  "filter_chain_runner"
  "controller_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "lifecycle_manager"
  "static_transform_publisher"
  "_ros2_daemon"
)

# Multi-pass kill loop — single pass sometimes misses processes whose
# argv match takes longer than pkill's signal delivery. Retry until clean.
for pass in 1 2 3; do
  for pat in "${PATTERNS[@]}"; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 1
done
# Recheck — anything still alive?
LEFT=""
for pat in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pat" 2>/dev/null || true)
  [[ -n "$pids" ]] && LEFT="${LEFT}\n  $pat → pids: $pids"
done
if [[ -n "$LEFT" ]]; then
  echo -e "[preflight] WARN: still alive:$LEFT"
fi

# Clean DDS shared-memory + daemon
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
sleep 0.5
ros2 daemon start 2>/dev/null || true

# ── Env ─────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ${WS}/install/setup.bash

# Force cupy backend (no torch on Orin Nano, sm_87)
export ELEVATION_MAPPING_FORCE_CUPY=1

# Make sure we're using FastDDS multicast (default), not Cyclone
unset RMW_IMPLEMENTATION || true
unset CYCLONEDDS_URI || true
export FASTRTPS_DEFAULT_PROFILES_FILE=""

# ── DDS isolation ──────────────────────────────────────────────────────
# Cross-host DDS contamination root cause: desktop runs HIL stack with
# its OWN imu→body static (the lidar mount tilt). Multicast leaks those
# /robot/tf_static messages to the Jetson, where they conflict with the
# Jetson's body→base_link static — `body` ends up with two parents (imu
# from desktop, base_link as child from Jetson) and the TF buffer splits
# into two unconnected trees: {map→odom} + {imu→body→base_link}. Nav2's
# controller_server then logs "Could not find a connection between odom
# and base_link" indefinitely.
#
# For bag-test (Jetson autonomous, no desktop), isolate by ROS_DOMAIN_ID.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_BAGTEST:-42}"
echo "[isolation] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (prevents cross-host TF leakage)"

# ── Launch ──────────────────────────────────────────────────────────────
echo ""
echo "================================================================"
echo "  Jetson full-load bag-test stack"
echo "    launch  : $LAUNCH_FILE"
echo "    cupy    : forced (no torch)"
echo "    sim_time: true (via bag --clock; RTF still measured wall-clock by ros2 bag)"
echo "================================================================"
echo ""
echo "Now in another window run:"
echo "  ros2 bag play /tmp/bag_test/onboard_noetic_*_ops2_ros2_raw \\"
echo "    --topics /livox/imu /livox/lidar \\"
echo "    --qos-profile-overrides-path /tmp/bag_qos_overrides.yaml \\"
echo "    --clock --rate 1.0"
echo ""
echo "Watch the bag-play 'real-time factor' line for the headline number."
echo ""

exec ros2 launch "$LAUNCH_FILE"
