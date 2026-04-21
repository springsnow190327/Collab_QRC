cmake_minimum_required(VERSION 3.5)
project(local_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/nlohmann)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Unitree SDK is optional — only needed for real-robot pathFollower
find_package(unitree_api QUIET)
find_package(go2_sport_api QUIET)

add_executable(localPlanner src/localPlanner.cpp)
ament_target_dependencies(localPlanner rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)

# pathFollower always builds; Unitree real-robot API is optional.
add_executable(pathFollower src/pathFollower.cpp)
if(unitree_api_FOUND AND go2_sport_api_FOUND)
  target_compile_definitions(pathFollower PRIVATE HAS_UNITREE_API)
  ament_target_dependencies(pathFollower rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions go2_sport_api unitree_api)
else()
  message(STATUS "Unitree SDK not found — pathFollower built without real-robot API")
  ament_target_dependencies(pathFollower rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)
endif()
install(TARGETS pathFollower DESTINATION lib/${PROJECT_NAME})

install(TARGETS localPlanner DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  paths
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
