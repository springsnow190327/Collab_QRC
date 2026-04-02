-- Cartographer 3D config for Go2W Gazebo simulation
-- Based on go2w_3d_mapping.lua but adapted for simulated sensor topics/frames
--
-- Sim sensor pipeline:
--   Gazebo IMU plugin  → /{ns}/imu/data          (frame: imu)
--   Gazebo LiDAR plugin → /{ns}/registered_scan  (frame: livox_mid360)
--   URDF TF chain: base_link → base → imu → livox_mid360
--
-- IMPORTANT:
--   tracking_frame should remain the physical IMU sensor frame so Cartographer
--   can consume IMU data correctly, but published_frame must be the robot body
--   frame. Publishing on imu gives that sensor frame two parents:
--     1. base → imu from robot_state_publisher
--     2. odom → imu from Cartographer
--   which breaks the TF tree and corrupts downstream pose consumers.
--
-- Cartographer therefore tracks 'imu' but publishes TF on 'base_link':
--   map → odom → base_link → base → imu → livox_mid360
-- carto_odom_bridge then converts TF(map→base_link) to /{ns}/odom/nav.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",                 -- Gazebo IMU plugin publishes in imu frame
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 1.0,     -- sim TF can lag behind scan timestamps
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  publish_tracked_pose = true,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.use_trajectory_builder_3d = true

-- Local SLAM
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_3D.min_range = 0.2     -- match sim lidar min
TRAJECTORY_BUILDER_3D.max_range = 20.0    -- match sim lidar max

-- Disable online correlative matcher (expensive for 3D)
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- Constant-velocity extrapolator (avoid gravity drift from accelerometer integration)
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.001

-- Ceres scan matcher
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5e-1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5e0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50

-- Motion filter
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(5)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 5.0

-- Global SLAM (loop closure)
POSE_GRAPH.optimize_every_n_nodes = 60
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 3.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-2
POSE_GRAPH.optimization_problem.rotation_weight = 1e-2
POSE_GRAPH.global_constraint_search_after_n_seconds = 3

POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.optimization_problem.log_solver_summary = false

return options
