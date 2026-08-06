-- Calibrated LiDAR + IMU profile for Yahboom DOGZILLA S2.
--
-- The host command refuses to select this file unless calibration/imu.json
-- contains a successful six-pose axis calibration. Cartographer's local
-- trajectory builder then combines angular/gravity data with scan matching.

local options = include "dogzilla_2d.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

return options
