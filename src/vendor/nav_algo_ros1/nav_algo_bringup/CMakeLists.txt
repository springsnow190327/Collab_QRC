cmake_minimum_required(VERSION 3.10)
project(nav_algo_bringup)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
