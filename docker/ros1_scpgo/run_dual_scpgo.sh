#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

ROSCORE_PID=""

cleanup() {
  jobs -pr | xargs -r kill 2>/dev/null || true
  if [[ -n "${ROSCORE_PID}" ]]; then
    wait "$ROSCORE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if rostopic list >/dev/null 2>&1; then
  echo "[ros1_scpgo] using existing ROS 1 master at ${ROS_MASTER_URI}"
else
  roscore &
  ROSCORE_PID=$!
fi

until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done

rosparam set /use_sim_time true
rosparam load /opt/scpgo/scpgo_no_save.yaml

run_scpgo() {
  local ns="$1"
  rosrun fast_lio_sam fast_lio_sam_node \
    __name:="fast_lio_sam_${ns}" \
    /Odometry:="/${ns}/Odometry" \
    /cloud_registered:="/${ns}/cloud_registered_body" \
    /pose_stamped:="/${ns}/sc_pgo/pose_stamped" \
    /corrected_path:="/${ns}/sc_pgo/corrected_path" \
    /corrected_odom:="/${ns}/sc_pgo/corrected_odom_points" \
    /corrected_map:="/${ns}/sc_pgo/corrected_map" \
    /corrected_current_pcd:="/${ns}/sc_pgo/corrected_current_pcd" \
    /loop_detection:="/${ns}/sc_pgo/loop_detection" \
    /ori_odom:="/${ns}/sc_pgo/ori_odom_points" \
    /ori_path:="/${ns}/sc_pgo/ori_path" \
    /src:="/${ns}/sc_pgo/debug_src" \
    /dst:="/${ns}/sc_pgo/debug_dst" \
    /aligned:="/${ns}/sc_pgo/debug_aligned" \
    /save_dir:="/${ns}/sc_pgo/save_dir" &
}

run_scpgo robot_a
run_scpgo robot_b

wait -n
