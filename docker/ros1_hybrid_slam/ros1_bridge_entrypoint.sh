#!/usr/bin/env bash
set -euo pipefail

set +u
source "/opt/ros/${ROS1_DISTRO:-noetic}/setup.bash"
source "/opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/setup.bash"
set -u

# Wait for ROS1 master before doing anything else (the master container can be
# slightly slower than the bridge container to start up).
echo "[bridge] waiting for ROS 1 master at ${ROS_MASTER_URI:-http://127.0.0.1:11311}"
for i in $(seq 1 60); do
  if rostopic list >/dev/null 2>&1; then
    echo "[bridge] ROS 1 master is up."
    break
  fi
  sleep 1
done

# Load bridge.yaml as ROS 1 parameters under /ros_bridge namespace, then run
# parameter_bridge. This is the only mode that gives the bridge full type
# information for topics where ROS 1 has only a SUBSCRIBER (no publisher) —
# dynamic_bridge --bridge-all-topics returns "ROS 1 type ''" for those and
# silently drops messages, which broke the sim_lidar → swarm_lio2 path.
BRIDGE_YAML="${BRIDGE_YAML:-/bridge.yaml}"
if [[ ! -f "${BRIDGE_YAML}" ]]; then
  echo "[bridge] WARN: ${BRIDGE_YAML} missing — falling back to dynamic_bridge."
  exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
fi

# parameter_bridge reads its topic list from rosparam under its own node
# namespace. Load the YAML there.
echo "[bridge] loading ${BRIDGE_YAML} into rosparam /ros_bridge/"
rosparam load "${BRIDGE_YAML}" /ros_bridge

echo "[bridge] starting parameter_bridge (static topic list from ${BRIDGE_YAML})"
exec /opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/lib/ros1_bridge/parameter_bridge \
    __name:=ros_bridge
