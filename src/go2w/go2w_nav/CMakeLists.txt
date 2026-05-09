cmake_minimum_required(VERSION 3.8)
project(go2w_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)

# 2026-05-09: removed astar_nav_node, hybrid_astar_nav_node,
# nav2_hybrid_astar_nav_node, and Python default_nav.py — superseded by
# Nav2 SmacPlannerHybrid/Lattice + MPPI (nav2_mppi backend).
# Surviving Python utilities are install-only (no rclcpp deps needed).

install(PROGRAMS
  scripts/collision_monitor.py
  scripts/cmd_vel_safety_shield.py
  scripts/far_status_adapter.py
  scripts/path_safety_filter.py
  scripts/rl_locomotion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
