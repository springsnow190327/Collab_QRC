cmake_minimum_required(VERSION 3.10)
project(trav_cost_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# C++ executable: filter_chain_runner
add_executable(filter_chain_runner src/filter_chain_runner.cpp)
ament_target_dependencies(filter_chain_runner
  rclcpp
  filters
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_filters
)

# C++ executable: grid_map_to_occupancy_grid_cpp
# Port of trav_cost_filters/grid_map_to_occupancy_grid.py (Python @ 0.59 Hz
# on Orin Nano was a real bottleneck; the C++ port targets ~5 Hz).
# Same param contract + topic names — drop-in replacement.
add_executable(grid_map_to_occupancy_grid_cpp src/grid_map_to_occupancy_grid_node.cpp)
ament_target_dependencies(grid_map_to_occupancy_grid_cpp
  rclcpp
  grid_map_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS filter_chain_runner grid_map_to_occupancy_grid_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Python: grid_map_to_occupancy_grid adapter (Phase 5)
# ament_python_install_package installs the trav_cost_filters Python package so
# the adapter can import from it. The executable is installed without the .py
# extension so `ros2 run trav_cost_filters grid_map_to_occupancy_grid` works.
ament_python_install_package(${PROJECT_NAME})

install(
  FILES trav_cost_filters/grid_map_to_occupancy_grid.py
  RENAME grid_map_to_occupancy_grid
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  FILES trav_cost_filters/ramp_ascent_goal_node.py
  RENAME ramp_ascent_goal_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(
  FILES trav_cost_filters/ramp_cmd_vel_assist_node.py
  RENAME ramp_cmd_vel_assist_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

# Config + launch
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
