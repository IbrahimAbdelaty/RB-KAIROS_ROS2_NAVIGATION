options = {
  map_builder = {
    use_trajectory_builder_2d = true,
    num_background_threads = 4,
  },
  trajectory_builder = {
    use_imu = false,
    trajectory_builder_2d = {  -- Moved min/max_range here
      min_range = 0.1,         -- Required parameter
      max_range = 8.0,         -- Required parameter
      submaps = {
        num_range_data = 90,
        grid_options_2d = {
          grid_type = "PROBABILITY_GRID",
          resolution = 0.05,
        },
      },
      scan_matching = {
        ceres_scan_matcher = {
          occupied_space_weight = 20.0,
          translation_weight = 10.0,
          rotation_weight = 1.0,
        },
      },
    },
  },
  map_frame = "map",
  tracking_frame = "robot_base_link",
  published_frame = "robot_base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  num_laser_scans = 2,  -- Keep this as 2 for two separate lasers
  use_odometry = false,
}

return options
