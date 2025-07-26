-- trajectory_builder_2d.lua

local TRAJECTORY_BUILDER_2D = {
  min_range = 0.1,
  max_range = 8.0,
  num_accumulated_range_data = 10,
  adaptive_voxel_filter = {
    max_length = 0.075,
    min_num_points_in_voxel = 2,
    max_range = 50.,
  },
  fast_correlative_scan_matcher = {
    linear_search_window = 11.,
    angular_search_window = 0.2,
    translation_delta_cost_factor = 1e-1,
    rotation_delta_cost_factor = 1e-1,
    num_scans = 0,
    subdivisions = {5, 5, 5, 5},
    correspondence_cost_weight = 1.,
    high_resolution_threshold = 0.,
    low_resolution_threshold = 0.,
    likelihood_field_weight = 0.,
    adaptive_weight = 0.,
  },
  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 1.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
    adaptive_weight = 0.,
  },
  motion_filter = {
    max_time_seconds = 0.2,
    max_distance_meters = 0.05,
    max_angle_radians = 0.003,
  },
  real_time_correlative_scan_matcher = {
    linear_search_window = 1.6,
    angular_search_window = 0.02618,
    translation_delta_cost_factor = 1e-1,
    rotation_delta_cost_factor = 1e-1,
    correspondence_cost_weight = 1.,
  },
  use_imu_data = false,
  num_accumulated_range_data = 1,
  max_range = 8.,
}

return TRAJECTORY_BUILDER_2D
