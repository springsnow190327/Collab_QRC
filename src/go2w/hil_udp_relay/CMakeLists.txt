cmake_minimum_required(VERSION 3.8)
project(hil_udp_relay)

# HIL UDP relay (ROS 2 side). Lightweight typed UDP bridge replacing the broken
# ros1_bridge on the Orin NX HIL bench. See include/hil_udp_relay/udp_protocol.hpp.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(include)

# DOWN-link transmitter: subscribes /livox/lidar + /livox/imu, sends over UDP.
add_executable(hil_relay_tx_node src/hil_relay_tx_node.cpp)
ament_target_dependencies(hil_relay_tx_node
  rclcpp sensor_msgs geometry_msgs nav_msgs livox_ros_driver2)

# UP-link receiver: binds UDP, republishes /robot/cmd_vel (+ optional viz).
add_executable(hil_relay_rx_node src/hil_relay_rx_node.cpp)
ament_target_dependencies(hil_relay_rx_node
  rclcpp sensor_msgs geometry_msgs nav_msgs livox_ros_driver2)

install(TARGETS
  hil_relay_tx_node
  hil_relay_rx_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_package()
