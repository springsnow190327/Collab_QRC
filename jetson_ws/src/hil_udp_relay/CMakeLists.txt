cmake_minimum_required(VERSION 3.0.2)
project(hil_udp_relay)

# HIL UDP relay (ROS 1 Noetic side). Lightweight typed UDP bridge replacing the
# broken ros1_bridge on the Orin NX. See include/hil_udp_relay/udp_protocol.hpp.

add_compile_options(-std=c++14 -Wall -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  livox_ros_driver2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs livox_ros_driver2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# pthread for the recv threads in UdpReceiver poll loops.
find_package(Threads REQUIRED)

# DOWN-link receiver: binds UDP, republishes /livox/lidar + /livox/imu.
add_executable(hil_relay_rx_node src/hil_relay_rx_node.cpp)
target_link_libraries(hil_relay_rx_node ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(hil_relay_rx_node ${catkin_EXPORTED_TARGETS})

# UP-link transmitter: subscribes /robot/cmd_vel (+ optional viz), sends over UDP.
add_executable(hil_relay_tx_node src/hil_relay_tx_node.cpp)
target_link_libraries(hil_relay_tx_node ${catkin_LIBRARIES} Threads::Threads)
add_dependencies(hil_relay_tx_node ${catkin_EXPORTED_TARGETS})

install(TARGETS hil_relay_rx_node hil_relay_tx_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
