include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "robot_imu_link",
  published_frame = "robot_base_link",
  odom_frame = "robot_odom",
  provide_odom_frame = true,
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,  -- 3D SLAM doesn't need projection
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,  -- We're not using 2D scans
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- Using point cloud from RSLidar
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 5.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 15.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 4.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1e3
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 120

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
POSE_GRAPH.optimization_problem.rotation_weight = 3e5
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3

return options 