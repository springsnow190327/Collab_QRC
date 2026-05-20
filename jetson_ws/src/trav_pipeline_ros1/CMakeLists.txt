cmake_minimum_required(VERSION 3.0.2)
project(trav_pipeline_ros1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  grid_map_msgs
  actionlib_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/elevation_mapping_node.py
  scripts/trav_filter_occ_grid.py
  scripts/cfpa2_to_movebase_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
