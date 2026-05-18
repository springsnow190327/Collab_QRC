#!/usr/bin/env bash
# Launch nav test with the ops2 SLAM-reconstructed scene (mesh-based world,
# derived from real-robot walk via offline_slam + density-filter + Poisson).
# Uses Fast-LIO2 + MID-360 by default.
#
# Usage:
#   ./scripts/launch/nav_test_slam_ops2.sh                    # GUI + RViz
#   ./scripts/launch/nav_test_slam_ops2.sh gui:=false          # headless
#
# Scene file:  src/go2w/go2_gazebo_sim/mujoco/slam_ops2.xml
# Mesh asset:  src/go2w/go2_gazebo_sim/mujoco/assets/slam_ops2/scans.obj
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2.xml"

# Scene area is a rough estimate — used by exploration metrics for coverage %.
# Adjust if your reconstructed environment is larger/smaller.
exec "${WS_DIR}/scripts/launch/nav_test_fastlio.sh" \
  "mujoco_model_path:=${SCENE}" \
  "scene_area_m2:=400" \
  "spawn_x:=5.12" \
  "spawn_y:=-8.76" \
  "spawn_yaw:=0.0" \
  "$@"
