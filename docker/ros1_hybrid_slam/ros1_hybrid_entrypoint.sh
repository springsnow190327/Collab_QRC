#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

MODE="${1:-idle}"
SRC_ROOT="/external"
WS="/catkin_ws"
GTSAM_LIB_DIR="/opt/ros/noetic/lib/x86_64-linux-gnu"

export LIBRARY_PATH="${GTSAM_LIB_DIR}${LIBRARY_PATH:+:${LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${GTSAM_LIB_DIR}:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CATKIN_JOBS="${CATKIN_JOBS:-1}"
export MAKEFLAGS="${MAKEFLAGS:--j${CATKIN_JOBS}}"

mkdir -p "${WS}/src" /logs

stage_backend() {
  local src="$1"
  local dst="$2"
  if [[ ! -d "${src}" ]]; then
    return 0
  fi
  if [[ -L "${dst}" ]]; then
    rm -f "${dst}"
  fi
  if [[ ! -e "${dst}" ]]; then
    mkdir -p "$(dirname "${dst}")"
    cp -a "${src}" "${dst}"
  fi
}

install_livox_sdk_if_needed() {
  if [[ -f /usr/local/lib/liblivox_sdk_static.a ]]; then
    return 0
  fi

  local sdk_src="${SRC_ROOT}/Livox-SDK"
  local sdk_work="/tmp/livox_sdk_src"
  if [[ ! -f "${sdk_src}/CMakeLists.txt" ]]; then
    sdk_src="${SRC_ROOT}/Swarm-LIO2/livox_ros_driver_mars/Livox-SDK"
  fi
  if [[ ! -d "${sdk_src}" ]]; then
    echo "ERROR: Livox-SDK source not found at ${sdk_src}" >&2
    return 6
  fi
  if [[ ! -f "${sdk_src}/CMakeLists.txt" ]]; then
    echo "ERROR: Livox-SDK source at ${sdk_src} has no CMakeLists.txt; fetch https://github.com/Livox-SDK/Livox-SDK into external/Livox-SDK." >&2
    return 7
  fi

  echo "Installing Livox-SDK from ${sdk_src} into /usr/local"
  rm -rf "${sdk_work}"
  mkdir -p "${sdk_work}"
  cp -a "${sdk_src}/." "${sdk_work}/"
  chmod -R u+w "${sdk_work}"
  rm -rf "${sdk_work}/build"
  mkdir -p "${sdk_work}/build"
  (
    cd "${sdk_work}/build"
    cmake ..
    make -j"$(nproc)"
    make install
  )
  ldconfig || true
}

write_swarm_lio2_collab_wrapper_launch() {
  local wrapper="/tmp/collab_swarm_lio2_wrapper.launch"
  local robot_a_lidar_topic="${ROBOT_A_SWARM_LIO2_LIDAR_TOPIC:-/robot_a/velodyne_points}"
  local robot_b_lidar_topic="${ROBOT_B_SWARM_LIO2_LIDAR_TOPIC:-/robot_b/velodyne_points}"
  local robot_a_imu_topic="${ROBOT_A_SWARM_LIO2_IMU_TOPIC:-/robot_a/imu/data}"
  local robot_b_imu_topic="${ROBOT_B_SWARM_LIO2_IMU_TOPIC:-/robot_b/imu/data}"
  cat > "${wrapper}" <<EOF
<launch>
  <node pkg="swarm_lio" type="swarm_lio" name="laserMapping_quad1" output="screen">
    <rosparam command="load" file="\$(find swarm_lio)/config/simulation.yaml" />
    <param name="drone_id" type="int" value="1" />
    <param name="common/drone_id" type="int" value="1" />
    <param name="common/lid_topic" type="string" value="/quad1_pcl_render_node/sensor_cloud" />
    <param name="common/imu_topic" type="string" value="/quad_1/imu" />
    <param name="sub_gt_pose_topic" type="string" value="/quad_1/lidar_slam/odom" />
    <param name="multiuav/actual_uav_num" type="int" value="2" />
    <param name="publish/scan_bodyframe_pub_en" type="bool" value="true" />
  </node>

  <node pkg="swarm_lio" type="swarm_lio" name="laserMapping_quad2" output="log">
    <rosparam command="load" file="\$(find swarm_lio)/config/simulation.yaml" />
    <param name="drone_id" type="int" value="2" />
    <param name="common/drone_id" type="int" value="2" />
    <param name="common/lid_topic" type="string" value="/quad2_pcl_render_node/sensor_cloud" />
    <param name="common/imu_topic" type="string" value="/quad_2/imu" />
    <param name="sub_gt_pose_topic" type="string" value="/quad_2/lidar_slam/odom" />
    <param name="multiuav/actual_uav_num" type="int" value="2" />
    <param name="publish/scan_bodyframe_pub_en" type="bool" value="true" />
  </node>

  <!-- ROS2 simulation sensor topics, bridged into ROS1, adapted to Swarm-LIO2 simulation names. -->
  <node pkg="topic_tools" type="relay" name="robot_a_lidar_to_swarm_lio2" args="${robot_a_lidar_topic} /quad1_pcl_render_node/sensor_cloud" output="log" />
  <node pkg="topic_tools" type="relay" name="robot_a_imu_to_swarm_lio2" args="${robot_a_imu_topic} /quad_1/imu" output="log" />
  <node pkg="topic_tools" type="relay" name="robot_b_lidar_to_swarm_lio2" args="${robot_b_lidar_topic} /quad2_pcl_render_node/sensor_cloud" output="log" />
  <node pkg="topic_tools" type="relay" name="robot_b_imu_to_swarm_lio2" args="${robot_b_imu_topic} /quad_2/imu" output="log" />

  <!-- Swarm-LIO2 native outputs normalized to the ROS2 adapter raw contract. -->
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_a_odom_raw" args="/quad1/lidar_slam/odom /robot_a/swarm_lio2_raw/Odometry" output="log" />
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_a_static_raw" args="/quad1/cloud_registered_body /robot_a/swarm_lio2_raw/cloud_static" output="log" />
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_a_map_raw" args="/quad1/cloud_registered /robot_a/swarm_lio2_raw/cloud_map" output="log" />
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_b_odom_raw" args="/quad2/lidar_slam/odom /robot_b/swarm_lio2_raw/Odometry" output="log" />
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_b_static_raw" args="/quad2/cloud_registered_body /robot_b/swarm_lio2_raw/cloud_static" output="log" />
  <node pkg="topic_tools" type="relay" name="swarm_lio2_robot_b_map_raw" args="/quad2/cloud_registered /robot_b/swarm_lio2_raw/cloud_map" output="log" />
</launch>
EOF
  printf '%s\n' "${wrapper}"
}

stage_backend "${SRC_ROOT}/Swarm-LIO2/swarm_msgs" "${WS}/src/swarm_msgs"
stage_backend "${SRC_ROOT}/Swarm-LIO2/udp_bridge" "${WS}/src/udp_bridge"
stage_backend "${SRC_ROOT}/Swarm-LIO2/livox_ros_driver_mars" "${WS}/src/livox_ros_driver_mars"
stage_backend "${SRC_ROOT}/Swarm-LIO2/swarm_lio" "${WS}/src/swarm_lio"
stage_backend "${SRC_ROOT}/dynamic_lio/sr_lio" "${WS}/src/sr_lio"
stage_backend "${SRC_ROOT}/ERASOR" "${WS}/src/erasor"
if [[ "${INCLUDE_DYNAMIC_LIO_SCPGO:-false}" == "true" ]]; then
  stage_backend "${SRC_ROOT}/dynamic_lio/SC-PGO" "${WS}/src/sc_pgo_dynamic_lio"
elif [[ -L "${WS}/src/sc_pgo_dynamic_lio" ]]; then
  rm -f "${WS}/src/sc_pgo_dynamic_lio"
fi

if [[ "${MODE}" == "build" ]]; then
  install_livox_sdk_if_needed
  cd "${WS}"
  catkin_make -j"${CATKIN_JOBS}"
  exit 0
fi

if [[ "${MODE}" != "run" ]]; then
  sleep infinity
fi

if [[ ! -f "${WS}/devel/setup.bash" ]]; then
  echo "ERROR: ${WS}/devel/setup.bash not found. Run the compose build service first." >&2
  exit 2
fi

source "${WS}/devel/setup.bash"

SLAM_BACKEND="${SLAM_BACKEND:-swarm_lio2_shadow}"
DYNAMIC_FILTER_BACKEND="${DYNAMIC_FILTER_BACKEND:-temporal_voxel_fallback}"
STATIC_MAP_CLEANUP_BACKEND="${STATIC_MAP_CLEANUP_BACKEND:-none}"

echo "ROS1 hybrid SLAM runtime:"
echo "  deployment_mode=${DEPLOYMENT_MODE:-sim_hybrid_ros1_slam_ros2_nav}"
echo "  slam_backend=${SLAM_BACKEND}"
echo "  dynamic_filter_backend=${DYNAMIC_FILTER_BACKEND}"
echo "  static_map_cleanup_backend=${STATIC_MAP_CLEANUP_BACKEND}"
echo "  swarm_lio2_feed_source=${SWARM_LIO2_FEED_SOURCE:-sim_bridge}"
echo "  robot_a_swarm_lio2_lidar_topic=${ROBOT_A_SWARM_LIO2_LIDAR_TOPIC:-/robot_a/velodyne_points}"
echo "  robot_a_swarm_lio2_imu_topic=${ROBOT_A_SWARM_LIO2_IMU_TOPIC:-/robot_a/imu/data}"
echo "  robot_b_swarm_lio2_lidar_topic=${ROBOT_B_SWARM_LIO2_LIDAR_TOPIC:-/robot_b/velodyne_points}"
echo "  robot_b_swarm_lio2_imu_topic=${ROBOT_B_SWARM_LIO2_IMU_TOPIC:-/robot_b/imu/data}"

case "${SLAM_BACKEND}" in
  swarm_lio2_shadow|swarm_lio2_primary)
    if roslaunch --files swarm_lio simulation.launch >/dev/null 2>&1; then
      wrapper_launch="$(write_swarm_lio2_collab_wrapper_launch)"
      if [[ "${SWARM_LIO2_FEED_SOURCE:-sim_bridge}" == "synthetic_contract_test" ]]; then
        /swarm_lio2_synthetic_contract_feeder.py &
      fi
      exec roslaunch "${wrapper_launch}"
    fi
    echo "ERROR: swarm_lio simulation.launch not available after catkin build." >&2
    exit 3
    ;;
  fast_lio_scpgo)
    echo "ERROR: fast_lio_scpgo is ROS2 production baseline, not this ROS1 Swarm-LIO2 container." >&2
    exit 4
    ;;
  *)
    echo "ERROR: unsupported SLAM_BACKEND=${SLAM_BACKEND}" >&2
    exit 5
    ;;
esac
