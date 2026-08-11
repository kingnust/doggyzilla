# URDF and monocular RTAB-Map framework

## Current status

This is source-only scaffolding. It is disabled by default and is not included
by the normal mapping or navigation launch files. The current Docker image was
not rebuilt, the required runtime packages are not pinned, and no deployment
command invokes the framework. A later image rebuild will copy the source
files because the ROS package is copied as a whole, but they will remain inert
until their dependencies and launch integration are deliberately added.

The framework deliberately separates two jobs:

```text
calibrated mono image + CameraInfo -------------------+
MS200 /scan ------------------------------------------+--> RTAB-Map
base_link -> camera_optical_frame TF -----------------+       |
Cartographer odom -> base_link TF                     |       +--> database
  -> namespaced odometry adapter                      |       +--> maps
  -> /rtabmap_shadow/odom_input ----------------------+       +--> no TF
```

Cartographer remains the owner of the operational `map -> odom` transform.
RTAB-Map uses the monocular camera for visual features and place recognition,
then uses scan-matched odometry and the MS200 scan to retain metric scale.
During mapping, the isolated adapter converts Cartographer's existing
`odom -> base_link` TF into `/rtabmap_shadow/odom_input`; it does not publish or
modify TF. A single monocular camera cannot provide dependable absolute scale
by itself.

This is an appearance-enhanced 2D LiDAR map, not dense 3D reconstruction.
Reliable dense geometry would require depth, stereo, or another 3D sensor;
monocular RGB is mainly valuable here for recognizing previously seen places.

## Files

- `urdf/dogzilla_s2.urdf.xacro` defines the body and the existing
  `base_link`, `laser_frame`, and `imu_link` frames. It adds `camera_link` and
  a REP-103 `camera_optical_frame`.
- `urdf/dogzilla_leg.xacro` defines 12 revolute joints with the exact names
  published by `safe_base.py`.
- `launch/robot_description.launch.py` is a standalone robot-state-publisher
  launch with `enabled:=false` as its default.
- `config/rtabmap_mono_shadow.yaml` configures RGB-only visual input together
  with `/scan` and scan-matched odometry. It sets `publish_tf: false`.
- `launch/rtabmap_mono_shadow.launch.py` starts no process unless its explicit
  safety gate is enabled. Its output stays under `/rtabmap_shadow` and its
  database defaults to `/logs/rtabmap_mono_shadow.db`. Its read-only odometry
  adapter can be disabled when an existing odometry message is selected.

## Camera contract

The current Yahboom camera application reads `/dev/video0` or `/dev/video1`
with OpenCV. It does not publish ROS messages, so it cannot feed this framework
directly. A later deployment needs one ROS camera owner that publishes:

| Topic | Type | Requirement |
| --- | --- | --- |
| `/camera/image_rect` | `sensor_msgs/msg/Image` | Rectified monocular image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Real intrinsic calibration, matching timestamps |

The image `frame_id` should be `camera_optical_frame`. The camera calibration
must be produced at the exact deployed resolution and must not contain the
all-zero placeholder matrices commonly emitted by an uncalibrated driver.

The framework does not open a video device. This avoids competing with the
Yahboom app while the driver and device selection are still undecided.

## Measurements still required

The current sensor transforms that already work for 2D mapping are preserved:

- `base_link -> laser_frame`: `(0.000, 0.000, 0.180)` metres
- `base_link -> imu_link`: `(0.085, 0.000, 0.070)` metres

Everything else in the mechanical model is provisional. Before operational
use, measure and verify:

1. The camera translation and roll/pitch/yaw relative to `base_link`.
2. Chassis dimensions, leg link lengths, and joint origins.
3. Which physical corner Yahboom calls legs 1 through 4.
4. Every motor's axis direction and zero offset against `/joint_states`.
5. Joint limits without commanding the robot from the URDF.

The model is currently suitable only as a TF and visualization framework. It
must not be treated as a control, collision, or kinematics model.

## Deployment gates

Deployment should happen in a later, stationary maintenance session:

1. Finish and save the current Cartographer map.
2. Measure the camera extrinsic and complete monocular intrinsic calibration.
3. Select one camera driver and one rectification path.
4. Pin Xacro, robot-state-publisher, camera, image-processing, and RTAB-Map
   packages in `deploy/ros-packages.lock`, then rebuild the image.
5. Move LiDAR and IMU static-transform ownership into one place. The current
   LiDAR launch and `hardware.launch.py` already publish those transforms, so
   robot-state-publisher must not publish duplicates during migration.
6. Verify the URDF and camera topics from a recorded bag before live shadow
   mode.
7. Run RTAB-Map under its isolated namespace with TF publication disabled,
   compare its loop closures with Cartographer, and only then decide whether it
   should become part of deployment.

Existing PGM/YAML/PBStream maps remain usable because their frame names and
working sensor transforms have not changed. They contain no historical camera
frames, so evaluating visual loop closure will require a new camera-enabled
recording or mapping run after deployment.

## Validation status

Offline validation currently passes:

- 20 static safety and structure tests from both the package directory and
  repository root
- 2,000 repeated test executions without state or path-dependent failures
- real Xacro expansion with versions 2.1.1 and 2.0.8
- 100 randomized camera-pose Xacro expansions
- semantic URDF parsing with one `base_link` root, 21 unique links, and 20
  unique joints forming one connected acyclic tree
- native YAML parsing and checks for monocular input, LiDAR input, isolated
  odometry, planar optimization, disabled TF output, persistent database, and
  absence from operational/deployment entry points

These checks establish source and configuration reliability, not mapping
quality on the physical robot. Still unverified are ROS 2 Humble launch-time
parameter acceptance, camera calibration quality, camera/scan/odometry timing,
CPU and memory load on the Pi, visual feature quality under motion and changing
light, real loop-closure precision, and long-duration database behavior. Those
require a stationary maintenance window followed by recorded-data testing and
an explicitly approved shadow-mode deployment.
