#!/usr/bin/env bash
# Preflight zombie killer — sourced from every nav/sim launch script.
#
# Why this exists: relaunching without killing the previous stack leaves
# stale publishers on /robot/map (TRANSIENT_LOCAL → latches old grid),
# /robot/registered_scan_map, /robot/way_point_coord, etc. Multiple
# localPlanner/pathFollower processes then race on /cmd_vel_stamped and
# the robot freezes.
#
# Usage (inside a launch wrapper):
#   source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
#
# Set PREFLIGHT_KILL=0 to skip (e.g. if you want to attach debugger to a
# currently-running stack).

if [[ "${PREFLIGHT_KILL:-1}" == "0" ]]; then
  return 0 2>/dev/null || exit 0
fi

# Patterns cover: MuJoCo plugin, sim sensor bridges, CMU stack, SLAM, loop
# closure, exploration planners, CHAMP locomotion, EKF, TF publishers we
# spawn, and the ros2 launch processes themselves. NOTE: any new node
# spawned by launch files should be added here too — a surviving child
# leaves stale publishers on /robot/map, state_estimation_at_scan, etc.
_PREFLIGHT_PATTERNS=(
  "ros2 launch .*(nav_test|door_demo|vlm_demo|real_autonomy)"
  # Sim core
  "mujoco_ros2_control"
  "mujoco_sensor_bridge"
  "mujoco_odom_bridge"
  "mujoco_contact_node"
  "mujoco_lidar_node"
  # Frame + scan plumbing
  "sensor_scan_generation"
  "pointcloud_frame_bridge"
  "__node:=map_to_odom_tf"
  # Mapping / SLAM / loop closure
  "octomap_server_node"
  "fastlio_mapping"
  "cartographer_node"
  "sc_pgo_node"
  # CMU stack
  "tare_planner_node"
  "far_planner"
  "terrainAnalysis"
  "localPlanner"
  "pathFollower"
  "exploration_metrics_logger"
  # Legacy nav / exploration
  "simple_scan_mapper"
  "cfpa2_"
  "reactive_nav_node"
  # CHAMP locomotion + EKF
  "champ_base"
  "state_estimation_node"
  "__node:=base_to_footprint_ekf"
  "__node:=footprint_to_odom_ekf"
)

_preflight_kill_patterns() {
  local sig="$1"
  local pat
  for pat in "${_PREFLIGHT_PATTERNS[@]}"; do
    pkill "${sig}" -f "${pat}" 2>/dev/null || true
  done
}

if pgrep -f "mujoco_ros2_control|tare_planner_node|far_planner|localPlanner|pathFollower|sensor_scan_generation|champ_base|fastlio_mapping" >/dev/null 2>&1; then
  echo "[preflight] killing stale sim/nav processes..."
  _preflight_kill_patterns "-TERM"
  sleep 2
  _preflight_kill_patterns "-KILL"
  sleep 1
  ros2 daemon stop >/dev/null 2>&1 || true
  if pgrep -f "mujoco_ros2_control|tare_planner_node|far_planner|localPlanner|pathFollower|sensor_scan_generation|champ_base|fastlio_mapping" >/dev/null 2>&1; then
    echo "[preflight] WARNING: some processes survived SIGKILL — check manually:"
    pgrep -af "mujoco_ros2_control|tare_planner_node|far_planner|localPlanner|pathFollower"
  else
    echo "[preflight] clean."
  fi
fi

unset _PREFLIGHT_PATTERNS
unset -f _preflight_kill_patterns
