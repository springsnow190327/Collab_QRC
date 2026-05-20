#!/usr/bin/env bash
# End-to-end integration test for the Noetic move_base + our plugins.
# Runs entirely inside the nav_algo:build_env Docker image.
#
# Scenario (synthetic, no real sim):
#   - 100x100 cell OccupancyGrid at 0.10m/cell (10m x 10m), all free.
#   - TF tree: map → odom → base_link, all identity, static.
#   - Odom: zero velocity at origin.
#   - Goal: (3.0, 0.0) facing +x.
#   - Expect: move_base spins up, SmacLattice produces a path,
#     MPPI produces non-zero cmd_vel within 10 seconds.

set -e

export ROBOT_NS=robot
export ROS_MASTER_URI=http://localhost:11311

source /opt/ros/noetic/setup.bash
source /ws_rw/devel_isolated/setup.bash

echo "[1/6] starting roscore"
roscore &
ROSCORE_PID=$!
sleep 2

echo "[2/6] static TF (map → odom → base_link, identity)"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
sleep 1

echo "[3/6] fake OccupancyGrid + Odom publishers"
python3 - <<'PY' &
import rospy, struct
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Quaternion

rospy.init_node("fake_data_pub", anonymous=True)

# 100x100 cells @ 0.10 m, origin (-5, -5). Centered on map origin.
W, H, RES = 100, 100, 0.10
grid = OccupancyGrid()
grid.header.frame_id = "map"
grid.info.resolution = RES
grid.info.width  = W
grid.info.height = H
grid.info.origin.position.x = -5.0
grid.info.origin.position.y = -5.0
grid.info.origin.orientation.w = 1.0
grid.data = [0] * (W * H)   # all free

odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id  = "base_link"
odom.pose.pose.orientation = Quaternion(0, 0, 0, 1)

grid_pub = rospy.Publisher("/robot/traversability_grid", OccupancyGrid, latch=True, queue_size=1)
odom_pub = rospy.Publisher("/robot/odom/nav", Odometry, queue_size=10)

r = rospy.Rate(10)
i = 0
while not rospy.is_shutdown():
    now = rospy.Time.now()
    grid.header.stamp = now
    odom.header.stamp = now
    if i % 10 == 0:
        grid_pub.publish(grid)
    odom_pub.publish(odom)
    i += 1
    r.sleep()
PY
FAKE_PID=$!
sleep 2

echo "[4/6] launching move_base with our plugins"
roslaunch nav_algo_bringup move_base.launch &
MB_PID=$!

echo "[5/6] waiting 8s for move_base to come up"
sleep 8

echo "[6/6] sending goal at (3.0, 0.0) and watching cmd_vel for 8s"
rostopic pub -1 /robot/move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0, z: 0.0},
   orientation: {x: 0, y: 0, z: 0, w: 1.0}}}" &
sleep 1

echo
echo "=========== cmd_vel sample (8s) ============"
timeout 8 rostopic hz /robot/cmd_vel 2>&1 || true
echo
echo "=========== last cmd_vel value ============"
timeout 2 rostopic echo -n 1 /robot/cmd_vel 2>&1 || true

# cleanup
kill $MB_PID $FAKE_PID $ROSCORE_PID 2>/dev/null || true
echo
echo "DONE"
