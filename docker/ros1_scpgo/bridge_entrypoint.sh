#!/usr/bin/env bash
set -euo pipefail

ROS1_DISTRO="${ROS1_DISTRO:-noetic}"
ROS2_DISTRO_IN_BRIDGE="${ROS2_DISTRO_IN_BRIDGE:-foxy}"

set +u
source "/opt/ros/${ROS1_DISTRO}/setup.bash"
source "/opt/ros/${ROS2_DISTRO_IN_BRIDGE}/setup.bash"
set -u

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_IP="${ROS_IP:-127.0.0.1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

until rostopic list >/dev/null 2>&1; do
  echo "[ros1_bridge] waiting for ROS 1 master at ${ROS_MASTER_URI}"
  sleep 1
done

rosparam load /bridge.yaml

exec ros2 run ros1_bridge parameter_bridge
