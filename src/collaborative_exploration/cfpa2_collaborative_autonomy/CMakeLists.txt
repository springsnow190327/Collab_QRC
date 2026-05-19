cmake_minimum_required(VERSION 3.10)
project(cfpa2_collaborative_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cfpa2_peer_coordination_msgs REQUIRED)

# ─────────────────────────────────────────────────────────────────────
# cfpa2_ops — modular kernel library (Phase A).
# ─────────────────────────────────────────────────────────────────────
add_library(cfpa2_ops STATIC
  src/ops/frontier_extract.cpp
  src/ops/distance_transform.cpp
  src/ops/info_gain.cpp
  src/ops/dead_frontier_filter.cpp
  src/ops/cluster.cpp)
target_include_directories(cfpa2_ops PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(cfpa2_ops PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS cfpa2_ops
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# ─────────────────────────────────────────────────────────────────────
# cfpa2_grid_ops.so — extern "C" ABI shim for the Python coordinator's
# ctypes loader. Same symbol names + signatures as the pre-Phase-A
# monolithic file; implementations just delegate to cfpa2_ops.
# Retained while the Python coordinator is still the production path.
# ─────────────────────────────────────────────────────────────────────
add_library(cfpa2_grid_ops SHARED
  src/cfpa2_grid_ops_c_api.cpp)
set_target_properties(cfpa2_grid_ops PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  PREFIX "")
target_link_libraries(cfpa2_grid_ops cfpa2_ops)
install(TARGETS cfpa2_grid_ops
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin)

# ─────────────────────────────────────────────────────────────────────
# cfpa2_node_lib — rclcpp::Node implementations (Phase B).
#
# `CFPA2Coordinator` (dual-robot joint allocator) + `CFPA2SingleRobotNode`
# (subclass with peer-coordination filter). Both ctor+helpers live in
# this static library so the executables link cleanly.
# ─────────────────────────────────────────────────────────────────────
add_library(cfpa2_node_lib STATIC
  src/cfpa2_coordinator.cpp
  src/cfpa2_single_robot.cpp)
target_link_libraries(cfpa2_node_lib cfpa2_ops)
ament_target_dependencies(cfpa2_node_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cfpa2_peer_coordination_msgs)
set_target_properties(cfpa2_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
install(TARGETS cfpa2_node_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# ─────────────────────────────────────────────────────────────────────
# Pure-C++ executables. Named with `_cpp` suffix during the transition
# so the Python entry-points (`cfpa2_coordinator_node`,
# `cfpa2_single_robot_node`) remain runnable for fallback comparison.
# Once C++ verified on desktop + Jetson, rename + drop Python install.
# ─────────────────────────────────────────────────────────────────────
add_executable(cfpa2_coordinator_node_cpp
  src/cfpa2_coordinator_node_main.cpp)
target_link_libraries(cfpa2_coordinator_node_cpp cfpa2_node_lib)
ament_target_dependencies(cfpa2_coordinator_node_cpp rclcpp)

add_executable(cfpa2_single_robot_node_cpp
  src/cfpa2_single_robot_node_main.cpp)
target_link_libraries(cfpa2_single_robot_node_cpp cfpa2_node_lib)
ament_target_dependencies(cfpa2_single_robot_node_cpp rclcpp)

install(TARGETS
  cfpa2_coordinator_node_cpp
  cfpa2_single_robot_node_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ─────────────────────────────────────────────────────────────────────
# Python sources (transitional fallback).
# ─────────────────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# Stage cfpa2_grid_ops.so next to the installed Python module.
install(TARGETS cfpa2_grid_ops
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

install(
  FILES cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py
  RENAME cfpa2_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  FILES cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py
  RENAME cfpa2_single_robot_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cfpa2_ops cfpa2_node_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  cfpa2_peer_coordination_msgs)

ament_package()
