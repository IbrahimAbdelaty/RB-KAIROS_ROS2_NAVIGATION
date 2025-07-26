-- trajectory_builder.lua

include "trajectory_builder_2d.lua"  -- Include 2D builder first

local TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,  -- TRAJECTORY_BUILDER_2D is available
  -- Disable 3D if not needed
  trajectory_builder_3d = nil,
  pure_localization = false,
}

return TRAJECTORY_BUILDER
