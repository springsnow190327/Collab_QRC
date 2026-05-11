cmake_minimum_required(VERSION 3.8)
project(go2_gazebo_sim)

# CMake 4.x removed compatibility with cmake_policy(VERSION < 3.5), but system
# jsoncppConfig.cmake (pulled via gazebo_ros -> gazebo -> ignition-fuel_tools)
# still emits cmake_policy(VERSION 3.0). Force a minimum policy floor so the
# build works under any cmake the toolchain happens to invoke. Unknown to
# CMake < 3.24, where it is silently ignored.
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  urdf
  rviz
  config
  mujoco
  DESTINATION share/${PROJECT_NAME}
)

# Install remaining top-level scripts (non-wrapper implementations)
install(PROGRAMS
  scripts/goal_assigner_passthrough.py
  scripts/waypoint_mux.py
  scripts/shared_map_fuser.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/assets
  scripts/slam
  scripts/observability
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
