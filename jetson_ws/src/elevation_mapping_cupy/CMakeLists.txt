# Minimal CMakeLists.txt for elevation_mapping_cupy — Python-only build.
#
# Replaces the upstream CMakeLists.txt which requires pybind11_catkin (not in
# standard Noetic apt).  We only need the Python package installed into the
# catkin devel space so that `from elevation_mapping_cupy import ...` works.
# The C++ pybind11 node is not needed for our Python-entry-point wrapper.

cmake_minimum_required(VERSION 3.0.2)
project(elevation_mapping_cupy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  grid_map_msgs
  elevation_map_msgs
)

catkin_package()

# Install the Python package into the catkin devel space.
catkin_python_setup()

# Install config and launch directories so rospack find works.
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
