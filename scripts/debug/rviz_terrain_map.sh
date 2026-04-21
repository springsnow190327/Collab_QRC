#!/usr/bin/env bash
# Launch a second RViz instance focused on /robot/terrain_map (the cost
# surface localPlanner uses). Intensity = height-above-ground; with
# obstacleHeightThre=0.20 anything above that is an obstacle.
#
# Run alongside an already-started nav_test_go2_tare_real.sh session.
set -u -o pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

CONFIG="${WS_DIR}/install/go2_gazebo_sim/share/go2_gazebo_sim/rviz/terrain_map.rviz"
exec ros2 run rviz2 rviz2 -d "${CONFIG}" --ros-args -r __node:=rviz2_terrain
