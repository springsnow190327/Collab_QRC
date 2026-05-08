#!/usr/bin/env bash
set -euo pipefail

set +u
source "/opt/ros/${ROS1_DISTRO:-noetic}/setup.bash"
if [[ -f "/opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/setup.bash" ]]; then
  source "/opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/setup.bash"
fi
set -u

if command -v parameter_bridge >/dev/null 2>&1; then
  exec parameter_bridge --bridge-all-topics
fi

exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
