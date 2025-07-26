-- rbkairos.lua



-- Now require the modules
local pose_graph_options = require "pose_graph"
local map_builder = pose_graph_options.map_builder
local trajectory_builder = pose_graph_options.trajectory_builder

options = {
  map_builder = map_builder,
  trajectory_builder = trajectory_builder,
  map_frame = "map",
  tracking_frame = "robot_base_link",
  published_frame = "robot_base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  num_laser_scans = 2,
  use_odometry = false
}

trajectory_builder.trajectory_builder_2d.min_range = 0.1
trajectory_builder.trajectory_builder_2d.max_range = 8.0

return options
