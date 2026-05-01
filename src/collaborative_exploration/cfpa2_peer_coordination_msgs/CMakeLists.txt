cmake_minimum_required(VERSION 3.8)
project(cfpa2_peer_coordination_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClaimedFrontier.msg"
  "msg/PeerState.msg"
  "msg/NegotiationRequest.msg"
  "msg/NegotiationResponse.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

# Notes
# - rosidl_generate_interfaces — this is the call that triggers code generation for your message types
# - The DEPENDENCIES line must match what your messages actually use. If you add a message later that uses, say, nav_msgs, you must update both this list AND package.xml. Forgetting this is the most common build failure with interface packages.

