-- Pure localization with the physically calibrated DOGZILLA IMU.

local options = include "dogzilla_localization.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

return options
