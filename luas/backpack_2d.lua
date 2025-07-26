-- trajectory_builder_2d.lua
TRAJECTORY_BUILDER_2D = {
  use_imu_data = false, -- Set to true if using IMU
  min_range = 0.3, -- Minimum valid laser range (meters)
  max_range = 8.0, -- Maximum valid laser range (meters)
  missing_data_ray_length = 5.0, -- Length for "no data" rays

  -- Submap Configuration
  submaps = {
    num_range_data = 90, -- Scans per submap
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05, -- Grid resolution (meters)
    },
  },

  -- Scan Matching
  ceres_scan_matcher = {
    occupied_space_weight = 20.0,
    translation_weight = 10.0,
    rotation_weight = 1.0,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 20,
    },
  },

  -- Motion Filter (reduces computational load)
  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.2,
    max_angle_radians = 0.017, -- ~1 degree
  },

  -- For Multiple Lasers (Front + Rear)
  num_accumulated_range_data = 1, -- Process each scan independently
}
