#!/usr/bin/env bash
# run_nx_hil_bridge.sh — start the ros1_bridge on the Orin NX for the HIL bench.
#
# Bridges the laptop's simulated sensors (ROS 2 Humble) ⇄ the NX's ROS 1 Noetic
# autonomy stack. CustomMsg requires the from-source ros1_bridge built by
# /tmp/nx_bridge_build.sh (bridge_ws). See docs/claude/orin_nx_hil_design.md.
#
# Sourcing order matters: ROS 1 (Noetic) + Noetic livox msgs first, then ROS 2
# (Foxy) + the Foxy livox msgs + the from-source ros1_bridge overlay.
#
# Usage (on the NX):
#   ./run_nx_hil_bridge.sh                 # parameter_bridge with the HIL topic set
#   ./run_nx_hil_bridge.sh dynamic         # dynamic_bridge (auto-bridge all matched)
#   ./run_nx_hil_bridge.sh stop

set -u
MODE="${1:-parameter}"
NX_WS="/home/unitree/autonomous_exploration_zhu"
BRIDGE_WS="/home/unitree/bridge_ws"
TOPICS_YAML="${NX_WS}/bridge_topics_hil.yaml"

if [[ "$MODE" == "stop" ]]; then
  pkill -9 -f "ros1_bridge" 2>/dev/null || true
  pkill -9 -f "parameter_bridge\|dynamic_bridge" 2>/dev/null || true
  echo "bridge stopped."
  exit 0
fi

# Strip conda (same guard as the onboard scripts).
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE '(miniconda|conda)' | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME
fi
unset PYTHONPATH

# ROS 1 Noetic + the Noetic livox CustomMsg (ROS 1 side of the pairing).
source /opt/ros/noetic/setup.bash
source "${NX_WS}/devel/setup.bash"
# ROS 2 Foxy + Foxy livox msgs + from-source ros1_bridge (ROS 2 side + factories).
source /opt/ros/foxy/setup.bash
source "${BRIDGE_WS}/install/setup.bash"

# Match the laptop's domain (default 0). Same wifi/eth link used for SSH.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# ROS 1 master must be reachable (the onboard stack starts roscore). Point at it.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.123.18:11311}"
export ROS_IP="${ROS_IP:-192.168.123.18}"

echo "=================================================================="
echo "  Orin NX HIL bridge ($MODE)"
echo "    ROS_MASTER_URI: $ROS_MASTER_URI"
echo "    ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "    sensors  2to1 : /livox/lidar (CustomMsg), /livox/imu"
echo "    cmd+viz  1to2 : /robot/{cmd_vel,traversability_grid,Odometry,...}, /tf"
echo "=================================================================="

# Confirm CustomMsg pairs (fail fast with a clear message if the source build
# didn't take).
if ! ros2 run ros1_bridge dynamic_bridge --print-pairs 2>/dev/null | grep -qi custommsg; then
  echo "ERROR: livox CustomMsg is NOT paired by ros1_bridge." >&2
  echo "       The from-source build (bridge_ws) is required — run /tmp/nx_bridge_build.sh." >&2
  exit 1
fi
echo "  ✓ livox CustomMsg pairing present"

if [[ "$MODE" == "dynamic" ]]; then
  exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
fi

# parameter_bridge: feed the explicit HIL topic set.
exec ros2 run ros1_bridge parameter_bridge __params:="${TOPICS_YAML}"
