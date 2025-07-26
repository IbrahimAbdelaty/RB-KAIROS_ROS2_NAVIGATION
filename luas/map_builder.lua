-- map_builder.lua


MAP_BUILDER = {
  use_trajectory_builder_2d = true, -- Use 2D SLAM
  use_trajectory_builder_3d = false,
  num_background_threads = 4, -- Adjust based on your CPU cores
  pose_graph = POSE_GRAPH,
}
