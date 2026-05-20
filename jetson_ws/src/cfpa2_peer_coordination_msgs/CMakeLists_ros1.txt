# CMakeLists_ros1.txt — ROS 1 (Noetic) catkin build for the CFPA2 peer
# coordination message package. Swapped in on the Jetson Orin NX:
#
#     mv CMakeLists.txt CMakeLists_ros2.txt && mv CMakeLists_ros1.txt CMakeLists.txt
#     mv package.xml package_ros2.xml && mv package_ros1.xml package.xml
#
# Same 4 .msg files as the ROS 2 build; only the generator changes
# (message_generation/message_runtime instead of rosidl).

cmake_minimum_required(VERSION 3.10)
project(cfpa2_peer_coordination_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs)

add_message_files(
  FILES
  ClaimedFrontier.msg
  PeerState.msg
  NegotiationRequest.msg
  NegotiationResponse.msg)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs)
