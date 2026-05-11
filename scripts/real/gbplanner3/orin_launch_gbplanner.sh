#!/usr/bin/env bash
# Launch gbplanner3 + voxblox + ros1_bridge on Orin, wired to Foxy Fast-LIO.
#
# Architecture:
#   ROS 2 Foxy (native) — Fast-LIO + Mid-360 publishing /robot/cloud_registered_body, /robot/Odometry
#                       │
#                       │  ros1_bridge (Foxy <-> Noetic, local to Orin)
#                       ▼
#   ROS 1 Noetic (native) — voxblox + gbplanner3
#                       │
#                       │  ros1_bridge writes /pci_command_path back to Foxy
#                       ▼
#   ROS 2 Foxy <DDS over WiFi> Laptop ROS 2 Humble — SanD-Planner consumes /pci_command_path
#
# Run on Orin via SSH. Three terminals or a tmux session.

set -euo pipefail

GBPLANNER_WS="${GBPLANNER_WS:-$HOME/gbplanner3_ws}"
BRIDGE_CONFIG="${BRIDGE_CONFIG:-$(dirname $0)/bridge_topics.yaml}"
ROBOT_NS="${ROBOT_NS:-robot}"

# tmux session managing the three nodes
SESSION="gbplanner3"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n roscore

# ---------------------------------------------------------------
# Window 1: roscore (Noetic side)
# ---------------------------------------------------------------
tmux send-keys -t "$SESSION:roscore" \
    "source /opt/ros/noetic/setup.bash && roscore" C-m

sleep 3   # let roscore come up

# ---------------------------------------------------------------
# Window 2: ros1_bridge (Foxy <-> Noetic)
# ---------------------------------------------------------------
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" \
    "source /opt/ros/noetic/setup.bash && \
     source /opt/ros/foxy/setup.bash && \
     ros2 run ros1_bridge parameter_bridge __params:=$BRIDGE_CONFIG" C-m

# ---------------------------------------------------------------
# Window 3: voxblox + gbplanner3
# ---------------------------------------------------------------
tmux new-window -t "$SESSION" -n planner
tmux send-keys -t "$SESSION:planner" \
    "source /opt/ros/noetic/setup.bash && \
     source $GBPLANNER_WS/devel/setup.bash && \
     roslaunch gbplanner gbplanner_go2.launch \
       robot_ns:=$ROBOT_NS \
       cloud_topic:=/$ROBOT_NS/cloud_registered_body \
       odom_topic:=/$ROBOT_NS/Odometry" C-m

echo ""
echo "==> tmux session '$SESSION' running. Attach with:"
echo "    tmux attach -t $SESSION"
echo ""
echo "==> When ready to start exploration mission, in a new shell:"
echo "    source /opt/ros/noetic/setup.bash"
echo "    rosservice call /planner_control_interface/std_srvs/automatic_planning '{}'"
echo ""
echo "==> To stop:"
echo "    tmux kill-session -t $SESSION"
