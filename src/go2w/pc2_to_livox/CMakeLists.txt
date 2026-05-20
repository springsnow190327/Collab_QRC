cmake_minimum_required(VERSION 3.8)
project(pc2_to_livox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# Provides livox_ros_driver2/msg/CustomMsg + CustomPoint (msgs-only package).
find_package(livox_ros_driver2 REQUIRED)

add_executable(pc2_to_livox_node src/pc2_to_livox_node.cpp)
ament_target_dependencies(pc2_to_livox_node
  rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS pc2_to_livox_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
