-- trajectory_builder.lua
include "trajectory_builder_2d.lua"

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  -- Disable 3D if not needed
  trajectory_builder_3d = nil,
  pure_localization = false,
}
