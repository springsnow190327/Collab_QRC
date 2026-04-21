cmake_minimum_required(VERSION 3.8)
project(go2w_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/qos_bridge.py
  scripts/pointcloud_adapter.py
  scripts/twist_bridge.py
  scripts/slam_odom_relay.py
  scripts/temporal_scan_filter.py
  scripts/carto_odom_bridge.py
  scripts/frontier_3d_markers.py
  scripts/probability_grid_binarizer.py
  scripts/pointcloud_frame_bridge.py
  scripts/map_merger.py
  scripts/elevation_to_occupancy.py
  scripts/transform_everything.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
