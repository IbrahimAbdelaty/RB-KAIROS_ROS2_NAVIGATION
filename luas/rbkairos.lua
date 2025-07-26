
include "pose_graph.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "robot_base_link",
  published_frame = "robot_base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  num_laser_scans = 2,
  use_odometry = false
}

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0

return options
