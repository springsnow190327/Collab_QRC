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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
