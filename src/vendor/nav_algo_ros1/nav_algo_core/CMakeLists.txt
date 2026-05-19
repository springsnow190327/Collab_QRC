cmake_minimum_required(VERSION 3.10)
project(nav_algo_core LANGUAGES CXX)

# -----------------------------------------------------------------------------
# nav_algo_core
#
# Pure C++ static library: ported algorithm cores from Nav2 Humble's
# SmacPlannerLattice + MPPIController, made ROS 1 Noetic build-clean via
# include/nav_algo_core/compat.hpp.
#
# This library does NOT register any nav_core plugins or pluginlib XML.
# It only provides linkable .a archives. The ROS 1 plugin wrappers live
# in sibling packages (nav_algo_smac_ros1 / nav_algo_mppi_ros1).
# -----------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -O2 -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  costmap_2d
  angles
)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

# xtensor is header-only; pull from system path or vendored headers.
# Apt: libxtensor-dev (universe). Noetic typically has 0.21.x.
find_path(XTENSOR_INCLUDE_DIR xtensor/xtensor.hpp
  PATHS /usr/include /usr/local/include)
if(NOT XTENSOR_INCLUDE_DIR)
  message(FATAL_ERROR
    "xtensor headers not found. Install `apt-get install libxtensor-dev` "
    "or vendor xtensor into /usr/local.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nav_algo_smac nav_algo_mppi
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs visualization_msgs
                 tf2 tf2_geometry_msgs costmap_2d angles
  DEPENDS EIGEN3 OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${XTENSOR_INCLUDE_DIR}
)

# Smac core static library.
# costmap_downsampler is excluded — it's only invoked when
# downsample_costmap=true (your yaml has it off), and its ROS 2 publisher
# constructor signature differs from ROS 1's Costmap2DPublisher. Re-include
# once a ROS 1-native rewrite of the publisher path is added.
file(GLOB SMAC_SRC CONFIGURE_DEPENDS src/smac/*.cpp)
# costmap_downsampler is now in via stub of the ROS 1 Costmap2DPublisher
# create-call (yaml has downsample_costmap=false so the publisher path is
# never exercised; including the cpp provides the dtor symbol that
# SmacPlannerLattice's unique_ptr member needs at link time).
add_library(nav_algo_smac STATIC ${SMAC_SRC})
target_link_libraries(nav_algo_smac
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_options(nav_algo_smac PRIVATE -fPIC)

# MPPI core static library.
# path_handler.cpp and trajectory_visualizer.cpp are excluded — they need
# targeted rewrites (pair return types + LifecyclePublisher creation) that
# are out of scope for the first-compile pass.
file(GLOB MPPI_SRC CONFIGURE_DEPENDS
  src/mppi/*.cpp
  src/mppi/critics/*.cpp)
# path_handler now in via geometry_utils::{first_after_integrated_distance,
# min_by} ports in compat.hpp + ros::Duration replacement for tf2::durationFromSec.
list(FILTER MPPI_SRC EXCLUDE REGEX ".*trajectory_visualizer\\.cpp$")
add_library(nav_algo_mppi STATIC ${MPPI_SRC})
target_link_libraries(nav_algo_mppi
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_options(nav_algo_mppi PRIVATE -fPIC)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS nav_algo_smac nav_algo_mppi
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
