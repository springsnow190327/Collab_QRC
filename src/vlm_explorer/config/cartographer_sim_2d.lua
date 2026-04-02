-- Cartographer 2D config for Go2W Gazebo simulation using PointCloud2 input.
--
-- Motivation:
--   The 3D Cartographer occupancy grid is useful for SLAM visualization, but in
--   this stack it often produces "occupied + unknown" with very little explicit
--   free space after 2D projection. For CFPA2/default_nav we want a planner-
--   friendly 2D map with proper free-space carving, so this config runs
--   Cartographer's 2D trajectory builder directly on the 3D lidar PointCloud2.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 1.0,
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

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- Match max_range so rays up to 8 m carve free space instead of being dropped.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0

-- Keep only the wall-height band; exclude ground returns from the
-- downward-pitched lidar (mounted ~10-15 cm above base, 13° pitch down).
-- min_z filters ground returns that leak through as occupied-cell litter.
-- Tracking frame is "imu" (~0.25m above floor); lowered to 0.05 so short
-- obstacles (e.g. 0.5m green markers at z=0.25) are captured — they only
-- had ~5cm in-band before and barely registered.
TRAJECTORY_BUILDER_2D.min_z = 0.05
TRAJECTORY_BUILDER_2D.max_z = 0.80

-- Strongly asymmetric: hits are much stickier than misses so that small
-- obstacles (cones, boxes) are NOT erased by miss rays once the robot
-- turns away.  ~4-5 misses needed to cancel 1 hit.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(3.)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0

-- Larger submaps (more scans per submap) → more self-consistent before
-- being frozen, reducing inter-submap misalignment artifacts.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 70

POSE_GRAPH.optimize_every_n_nodes = 70
-- Raise min_score so only high-confidence loop closures are accepted;
-- bad constraints cause submap shifts that duplicate walls at edges.
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
POSE_GRAPH.optimization_problem.log_solver_summary = false

return options
