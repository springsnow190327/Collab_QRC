cmake_minimum_required(VERSION 3.8)
project(go2w_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# ── C++ RRT* reactive nav node ──
add_executable(reactive_nav_node src/reactive_nav_node.cpp)
ament_target_dependencies(reactive_nav_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

# ── C++ MPPI nav node ──
add_executable(mppi_nav_node src/mppi_nav_node.cpp)
ament_target_dependencies(mppi_nav_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

install(TARGETS reactive_nav_node mppi_nav_node DESTINATION lib/${PROJECT_NAME})

# ── Python default nav (A* grid planner) ──
install(PROGRAMS
  scripts/default_nav.py
  scripts/collision_monitor.py
  scripts/rviz_goal_relay.py
  scripts/far_status_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/default_nav_core
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
