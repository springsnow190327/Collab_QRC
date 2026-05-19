cmake_minimum_required(VERSION 3.10)
project(nav_algo_mppi_ros1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2 -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  angles
  nav_algo_core
)

# CUDA acceleration is opt-in. If nav_algo_mppi_cuda is in the workspace
# (CUDA toolchain available), link it + define NAV_ALGO_MPPI_HAS_CUDA so
# the plugin can instantiate a CudaBackend when use_cuda=true in yaml.
find_package(nav_algo_mppi_cuda QUIET)
if(nav_algo_mppi_cuda_FOUND)
  message(STATUS "nav_algo_mppi_ros1: CUDA backend available, will be enabled at runtime if use_cuda=true.")
  add_definitions(-DNAV_ALGO_MPPI_HAS_CUDA)
endif()
find_package(Eigen3 REQUIRED)

# xtensor headers (installed under /usr/local/include by build_env).
find_path(XTENSOR_INCLUDE_DIR xtensor/xtensor.hpp
  PATHS /usr/include /usr/local/include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib nav_core costmap_2d tf2 tf2_ros
                 tf2_geometry_msgs geometry_msgs nav_msgs angles
                 nav_algo_core
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${XTENSOR_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/mppi_controller_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
if(nav_algo_mppi_cuda_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${nav_algo_mppi_cuda_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}
    ${nav_algo_mppi_cuda_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
