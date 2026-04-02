cmake_minimum_required(VERSION 3.8)
project(go2_nav_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(simple_scan_mapper_cpp src/simple_scan_mapper_cpp.cpp)
ament_target_dependencies(simple_scan_mapper_cpp
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
)

add_executable(scan_rear_filter src/scan_rear_filter.cpp)
ament_target_dependencies(scan_rear_filter
  rclcpp
  sensor_msgs
)

install(DIRECTORY
  config
  docs
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  simple_scan_mapper_cpp
  scan_rear_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/simple_scan_mapper.py
  scripts/simple_frontier_explorer.py
  scripts/multi_robot_goal_assigner.py
  scripts/geometric_frontier.py
  scripts/gbplanner2_local.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(go2_nav_algorithms)

ament_package()
