#!/usr/bin/env bash
# Publish a test goal directly to FAR's input topic for tuning runs.
#
# 2026-05-09: previously pushed to /goal_pose (rviz 2D Goal Pose) and let
# rviz_goal_relay rewrite into /robot/way_point_coord. Relay removed; this
# script now publishes the PointStamped directly so far_planner gets it
# regardless of which (or no) /goal_pose subscriber is alive.
#
# Usage:
#   ./scripts/ops/far_publish_corner_goal.sh                # default (4.5, 0.0)
#   ./scripts/ops/far_publish_corner_goal.sh 11.5 3.5       # explicit x y
#   ROBOT_NS=robot_a ./scripts/ops/far_publish_corner_goal.sh
set -euo pipefail

GOAL_X="${1:-4.5}"
GOAL_Y="${2:-0.0}"
ROBOT_NS="${ROBOT_NS:-robot}"

echo "[far_publish_corner_goal] publishing to /${ROBOT_NS}/way_point_coord: x=${GOAL_X} y=${GOAL_Y}"

# 5 messages @ 5 Hz to absorb DDS late-joiner discovery race.
exec ros2 topic pub -t 5 -r 5 "/${ROBOT_NS}/way_point_coord" geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}
}"
