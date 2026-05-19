#!/usr/bin/env bash
# Record a Nav2 sim episode as the ROS 2 → ROS 1 equivalence-test baseline.
#
# Workflow:
#   1. In one terminal: ./scripts/launch/nav_test_3d_explore.sh gui:=false rviz:=true
#   2. In another:      ./scripts/bench/record_nav2_replay_bag.sh [duration_sec]
#   3. Drive the robot a bit (click Nav2 goal in RViz, or let CFPA2 explore)
#   4. Recording auto-stops after `duration_sec` (default 60s)
#
# What we capture is the planner + controller's full I/O surface:
#   - inputs  : costmap + odom + goal + plan
#   - outputs : cmd_vel, controller-side eval signals
# so that the ROS 1 port can replay the exact same input stream and the
# resulting cmd_vel + plan can be diff'd against the recorded sim trace.
#
# This bag is the gold reference for the equivalence harness. Don't re-record
# unless either the Nav2 yaml changed (planner/controller config) or the sim
# scenario changed (different scene or different spawn).

set -euo pipefail

DUR_SEC="${1:-60}"
NS="${ROBOT_NS:-robot}"
OUT_DIR="${OUT_DIR:-/tmp/nav2_replay_bag_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "${OUT_DIR}"
echo "Recording for ${DUR_SEC}s into ${OUT_DIR}"

# Inputs to planner+controller
INPUT_TOPICS=(
  /${NS}/global_costmap/costmap
  /${NS}/global_costmap/costmap_updates
  /${NS}/local_costmap/costmap
  /${NS}/local_costmap/costmap_updates
  /${NS}/odom/nav
  /${NS}/odom/ground_truth
  /${NS}/goal_pose
  /${NS}/traversability_grid
  /tf
  /tf_static
)

# Outputs that the ROS 1 port must reproduce
OUTPUT_TOPICS=(
  /${NS}/plan
  /${NS}/cmd_vel
  /${NS}/local_plan          # MPPI's selected trajectory
  /${NS}/optimal_trajectory  # if MPPI publishes it (visualize: true)
  /${NS}/transformed_global_plan
  /${NS}/behavior_tree_log
)

# Drop topics that don't exist; rosbag2 will warn but still record the rest.
ros2 bag record \
  --output "${OUT_DIR}/bag" \
  --storage mcap \
  "${INPUT_TOPICS[@]}" "${OUTPUT_TOPICS[@]}" &
REC_PID=$!

sleep "${DUR_SEC}"
kill -INT "${REC_PID}" 2>/dev/null || true
wait "${REC_PID}" 2>/dev/null || true

echo "Done. Bag at ${OUT_DIR}/bag"
ros2 bag info "${OUT_DIR}/bag" || true
