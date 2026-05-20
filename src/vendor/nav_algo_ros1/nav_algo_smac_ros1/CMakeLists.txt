cmake_minimum_required(VERSION 3.10)
project(nav_algo_smac_ros1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2 -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
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
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospack pluginlib nav_core costmap_2d tf2 tf2_ros
                 tf2_geometry_msgs geometry_msgs nav_msgs angles
                 nav_algo_core
  DEPENDS EIGEN3 OMPL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/smac_lattice_planner_ros.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Sample lattice primitives shipped with the package so the default
# lattice_filepath param resolves out of the box.
install(DIRECTORY sample_primitives/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sample_primitives
  OPTIONAL)
