# CMakeLists_ros1.txt — ROS 1 (Noetic) catkin build for the CFPA2 C++ node.
#
# Swapped in on the Jetson Orin NX (real Go2 onboard) in place of the
# ament_cmake CMakeLists.txt:
#
#     mv CMakeLists.txt CMakeLists_ros2.txt
#     mv CMakeLists_ros1.txt CMakeLists.txt
#     mv package.xml package_ros2.xml
#     mv package_ros1.xml package.xml
#     catkin build cfpa2_collaborative_autonomy
#
# The CFPA2_ROS1 compile definition is what selects the ros1/ adapter
# layer + ros::NodeHandle ctor path inside the (otherwise shared) source.

cmake_minimum_required(VERSION 3.10)
project(cfpa2_collaborative_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cfpa2_peer_coordination_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cfpa2_ops cfpa2_node_lib
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs visualization_msgs cfpa2_peer_coordination_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

# ─────────────────────────────────────────────────────────────────────
# cfpa2_ops — modular kernel library (Phase A). Pure C++, no ROS.
# ─────────────────────────────────────────────────────────────────────
add_library(cfpa2_ops STATIC
  src/ops/frontier_extract.cpp
  src/ops/distance_transform.cpp
  src/ops/info_gain.cpp
  src/ops/dead_frontier_filter.cpp
  src/ops/cluster.cpp)
set_target_properties(cfpa2_ops PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────────────
# cfpa2_node_lib — CFPA2Coordinator + CFPA2SingleRobotNode bodies.
# CFPA2_ROS1 selects the ros1/ adapters + ros::NodeHandle ctor path.
# ─────────────────────────────────────────────────────────────────────
add_library(cfpa2_node_lib STATIC
  src/cfpa2_coordinator.cpp
  src/cfpa2_single_robot.cpp)
target_compile_definitions(cfpa2_node_lib PUBLIC CFPA2_ROS1)
target_link_libraries(cfpa2_node_lib cfpa2_ops ${catkin_LIBRARIES})
set_target_properties(cfpa2_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ─────────────────────────────────────────────────────────────────────
# Executables — kept with the `_cpp` suffix for naming parity with the
# ROS 2 build (the launch arg `cfpa2_executable_suffix:=_cpp` selects them).
# ─────────────────────────────────────────────────────────────────────
add_executable(cfpa2_coordinator_node_cpp
  src/cfpa2_coordinator_node_main.cpp)
target_compile_definitions(cfpa2_coordinator_node_cpp PRIVATE CFPA2_ROS1)
target_link_libraries(cfpa2_coordinator_node_cpp cfpa2_node_lib ${catkin_LIBRARIES})

add_executable(cfpa2_single_robot_node_cpp
  src/cfpa2_single_robot_node_main.cpp)
target_compile_definitions(cfpa2_single_robot_node_cpp PRIVATE CFPA2_ROS1)
target_link_libraries(cfpa2_single_robot_node_cpp cfpa2_node_lib ${catkin_LIBRARIES})

# ─────────────────────────────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────────────────────────────
install(TARGETS cfpa2_ops cfpa2_node_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS
  cfpa2_coordinator_node_cpp
  cfpa2_single_robot_node_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL)
