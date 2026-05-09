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
find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 REQUIRED)

# ── C++ A* nav node (curvature-aware pure pursuit + Option B footprint check) ──
add_executable(astar_nav_node src/astar_nav_node.cpp)
ament_target_dependencies(astar_nav_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

install(TARGETS astar_nav_node DESTINATION lib/${PROJECT_NAME})

# ── C++ Hybrid A* nav node (OMPL Reeds-Shepp + Ceres smoother) ──
# Search core: hand-written state-lattice Hybrid A*.
# Reeds-Shepp distance + analytic shot: ompl::base::ReedsSheppStateSpace.
# Path smoother: Ceres autodiff non-linear least squares with corridor-
# aware obstacle cost (bilinear interpolation of chamfer EDT).
# Both libraries are AV/robotics-grade and already on this system
# (ros-humble-ompl, libceres-dev). The build degrades gracefully if
# either is missing — astar_nav_node still ships.
find_package(ompl QUIET)
find_package(Ceres QUIET)
if(ompl_FOUND AND Ceres_FOUND)
  add_executable(hybrid_astar_nav_node src/hybrid_astar_nav_node.cpp)
  ament_target_dependencies(hybrid_astar_nav_node
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
    tf2 tf2_geometry_msgs tf2_ros visualization_msgs
  )
  target_include_directories(hybrid_astar_nav_node PRIVATE
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(hybrid_astar_nav_node
    ${OMPL_LIBRARIES}
    Ceres::ceres
  )
  install(TARGETS hybrid_astar_nav_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "go2w_nav: building hybrid_astar_nav_node "
                 "(OMPL ${OMPL_VERSION}, Ceres ${Ceres_VERSION})")
else()
  message(WARNING "go2w_nav: OMPL or Ceres not found; "
                  "hybrid_astar_nav_node will NOT be built. "
                  "Install: ros-humble-ompl libceres-dev")
endif()

# ── B-route SPIKE: Hybrid A* + nav2_smac_planner Smoother ──
# Drop our hand-written Ceres smoother in favour of nav2_smac_planner's
# library smoother. The node inherits rclcpp_lifecycle::LifecycleNode
# (Nav2 library code expects a LifecycleNode parent).
find_package(rclcpp_lifecycle QUIET)
find_package(lifecycle_msgs QUIET)
find_package(nav2_costmap_2d QUIET)
find_package(nav2_smac_planner QUIET)
if(ompl_FOUND
   AND rclcpp_lifecycle_FOUND
   AND lifecycle_msgs_FOUND
   AND nav2_costmap_2d_FOUND
   AND nav2_smac_planner_FOUND)
  add_executable(nav2_hybrid_astar_nav_node src/nav2_hybrid_astar_nav_node.cpp)
  ament_target_dependencies(nav2_hybrid_astar_nav_node
    rclcpp rclcpp_lifecycle lifecycle_msgs
    geometry_msgs nav_msgs sensor_msgs std_msgs
    tf2 tf2_geometry_msgs tf2_ros visualization_msgs
    nav2_costmap_2d nav2_smac_planner
  )
  # B-route final shape: SEARCH = our hand-written Hybrid A* over OMPL
  # ReedsShepp state space; SMOOTH = nav2_smac_planner::Smoother (lib).
  # We DO need to link OMPL ourselves because our search calls
  # `ompl::base::ReedsSheppStateSpace::distance` / `interpolate` directly.
  # nav2_smac_planner.so also links OMPL internally; both load the same
  # /lib/x86_64-linux-gnu/libompl.so.18, so this is a single-instance link.
  target_include_directories(nav2_hybrid_astar_nav_node PRIVATE
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(nav2_hybrid_astar_nav_node
    ${OMPL_LIBRARIES}
  )
  # ── ASAN diagnostic build (toggle with -DGO2W_NAV_ASAN=ON) ──
  # Compiles & links the node with -fsanitize=address so a real malloc
  # corruption gives us a backtrace instead of a bare SIGABRT.
  if(GO2W_NAV_ASAN)
    target_compile_options(nav2_hybrid_astar_nav_node PRIVATE
      -fsanitize=address -fno-omit-frame-pointer -g -O1)
    target_link_options(nav2_hybrid_astar_nav_node PRIVATE
      -fsanitize=address)
    message(STATUS "go2w_nav: nav2_hybrid_astar_nav_node built with ASAN")
  endif()
  install(TARGETS nav2_hybrid_astar_nav_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "go2w_nav: building nav2_hybrid_astar_nav_node "
                 "(B-route spike: nav2_smac_planner smoother)")
else()
  message(WARNING "go2w_nav: nav2_smac_planner / lifecycle deps not found; "
                  "nav2_hybrid_astar_nav_node will NOT be built. "
                  "Install: ros-humble-nav2-smac-planner ros-humble-rclcpp-lifecycle")
endif()

# ── Python default nav (A* grid planner) ──
install(PROGRAMS
  scripts/default_nav.py
  scripts/collision_monitor.py
  scripts/far_status_adapter.py
  scripts/path_safety_filter.py
  scripts/cmd_vel_safety_shield.py
  scripts/rl_locomotion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/default_nav_core
  DESTINATION lib/${PROJECT_NAME}/scripts
)

ament_package()
