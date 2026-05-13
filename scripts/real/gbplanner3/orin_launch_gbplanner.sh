#!/usr/bin/env bash
# Launch gbplanner3 + voxblox + (tiny) ros1_bridge on Orin.
#
# UPDATED 2026-05-12 for Point-LIO native ROS 1 stack:
#   - Point-LIO + Mid-360 driver are assumed to ALREADY be running natively
#     in ROS 1 Noetic (started by scripts/real/onboard_pointlio_noetic.sh).
#     We rely on it for:
#       /robot/cloud_registered_body  /robot/Odometry  /tf  /tf_static
#     Don't start a parallel roscore — re-use the one Point-LIO has.
#
#   - The Foxy<->Noetic bridge now only carries:
#       /pci_command_path                       (Noetic→Foxy, <1KB/s)
#       /gbplanner/exploration_status           (Noetic→Foxy, <1KB/s)
#       /planner_control_interface/* services   (Foxy→Noetic)
#     The heavy SLAM topics that used to bridge are gone — Point-LIO
#     produces them in Noetic natively.
#
# Architecture:
#   ROS 1 Noetic (Jetson, ALL native)
#     onboard_pointlio_noetic.sh ─► /robot/{cloud_registered_body,Odometry,tf}
#                                            │
#                                            ▼
#     voxblox + gbplanner_node + pci_general
#                                            │
#                                            │ /pci_command_path
#                                            ▼
#   ros1_bridge (this script's "bridge" tmux window) ─► ROS 2 Foxy
#                                                              │
#                                                              ▼
#                                                       Laptop ROS 2 Humble
#
# Run on Orin via SSH. Three-window tmux session.

set -euo pipefail

GBPLANNER_WS="${GBPLANNER_WS:-$HOME/gbplanner3_ws}"
BRIDGE_CONFIG="${BRIDGE_CONFIG:-$(dirname $0)/bridge_topics.yaml}"
ROBOT_NS="${ROBOT_NS:-robot}"

# ── Preflight: confirm Point-LIO is up ───────────────────────────────
echo "==> preflight: checking Point-LIO is publishing..."
source /opt/ros/noetic/setup.bash
if ! rostopic info "/${ROBOT_NS}/Odometry" 2>/dev/null | grep -q '^Publishers: $'; then
  # rostopic exits 1 if no publisher OR if no master.
  if ! rostopic list >/dev/null 2>&1; then
    echo "  ✗ no ROS master. Start onboard_pointlio_noetic.sh first." >&2
    exit 1
  fi
fi
if ! rostopic info "/${ROBOT_NS}/Odometry" 2>/dev/null | grep -q '^ \* /'; then
  echo "  ✗ /${ROBOT_NS}/Odometry has no publisher." >&2
  echo "    Start SLAM first:" >&2
  echo "      ~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh" >&2
  exit 1
fi
echo "  ✓ Point-LIO publishing /${ROBOT_NS}/Odometry"

# Confirm gbplanner3 workspace was built.
if [ ! -f "$GBPLANNER_WS/devel/setup.bash" ]; then
  echo "==> ERROR: $GBPLANNER_WS/devel/setup.bash missing." >&2
  echo "          Did orin_install.sh complete? Run:" >&2
  echo "            bash $(dirname $0)/orin_install.sh" >&2
  exit 1
fi

# tmux session managing the two extra windows (voxblox+gbplanner, bridge).
# roscore is NOT in here — Point-LIO already owns it.
SESSION="gbplanner3"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n bridge

# ── Window 1: ros1_bridge (Noetic ←→ Foxy, tiny PoseArray topic only) ──
tmux send-keys -t "$SESSION:bridge" \
    "source /opt/ros/noetic/setup.bash && \
     source /opt/ros/foxy/setup.bash && \
     ros2 run ros1_bridge parameter_bridge __params:=$BRIDGE_CONFIG" C-m

# ── Window 2: voxblox + gbplanner3 ──────────────────────────────────
tmux new-window -t "$SESSION" -n planner
tmux send-keys -t "$SESSION:planner" \
    "source /opt/ros/noetic/setup.bash && \
     source $GBPLANNER_WS/devel/setup.bash && \
     roslaunch gbplanner gbplanner_go2.launch \
       robot_name:=go2 \
       config_folder:=ugv/real/go2 \
       odometry_topic:=/$ROBOT_NS/Odometry \
       pointcloud_topic:=/$ROBOT_NS/cloud_registered_body" C-m

echo ""
echo "==> tmux session '$SESSION' running (windows: bridge, planner)."
echo "    Attach:        tmux attach -t $SESSION"
echo "    Switch window: Ctrl-b 1   (bridge)"
echo "                   Ctrl-b 2   (planner)"
echo ""
echo "==> Start exploration mission (new shell):"
echo "    source /opt/ros/noetic/setup.bash"
echo "    rosservice call /planner_control_interface/std_srvs/automatic_planning '{}'"
echo ""
echo "==> Stop everything:  tmux kill-session -t $SESSION"
echo "    (Point-LIO keeps running — stop it separately if needed:)"
echo "    ~/noetic_fastlio_ws/scripts/onboard_pointlio_noetic.sh stop"
