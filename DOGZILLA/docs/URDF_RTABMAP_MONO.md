# URDF and monocular RTAB-Map framework

## Current status

This is a deployed but calibration-gated shadow workflow. Exact Humble
versions of the camera, image processing, Xacro, robot-state-publisher, and
RTAB-Map dependencies are installed in `dogzilla-mapping:humble`. Normal
mapping and navigation launch files remain unchanged. Only `dogzilla shadow`
can start the separate visual service, and it refuses before hardware startup
when either required camera calibration file is absent or invalid.

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
- `config/mono_camera.yaml` is the tested 640x480, 30 Hz MJPEG camera profile.
- `deploy/Dockerfile` checksum-pins the upstream `usb_cam` frame-draining fix;
  the stock Humble 0.8.0 timer otherwise returned nearly one-second-old frames
  from this camera's 120 Hz V4L2 queue.
- `launch/mono_camera.launch.py` owns `/dev/video0` and optionally rectifies.
- `launch/camera_calibration.launch.py` combines the tested camera profile with
  the official ROS intrinsic-calibration GUI and shuts the camera down when the
  GUI exits.
- `launch/visual_shadow.launch.py` combines the gated camera, camera TF and
  RTAB launch while omitting duplicate LiDAR and IMU transforms.
- `dogzilla_slam/camera_model.py` rejects missing, placeholder, wrong-size, or
  unmeasured calibration before deployment starts.
- `dogzilla_slam/camera_validate.py` validates live frames, timing and
  calibrated `CameraInfo`.
- `dogzilla_slam/shadow_validate.py` proves that image, LiDAR, and scan-matched
  odometry timestamps overlap and that RTAB is publishing processed-node
  statistics. Topic existence alone is not treated as success.

## Camera contract

The Yahboom application and ROS camera cannot own `/dev/video0` together. The
operator command stops the Yahboom application through its normal Ctrl-C path,
refuses unknown competing owners, and then makes `usb_cam` the sole ROS owner:

| Topic | Type | Requirement |
| --- | --- | --- |
| `/camera/image_rect` | `sensor_msgs/msg/Image` | Rectified monocular image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Real intrinsic calibration, matching timestamps |

The image `frame_id` should be `camera_optical_frame`. The camera calibration
must be produced at the exact deployed resolution and must not contain the
all-zero placeholder matrices commonly emitted by an uncalibrated driver.

`dogzilla camera-check 12` verifies raw data without calibration. It must not
be used as evidence that rectification or RTAB-Map is ready. The `shadow`
command additionally requires non-zero intrinsic matrices and a measured mount
transform before it opens the camera.

`dogzilla camera-calibrate BOARD_SIZE SQUARE_METRES` writes the GUI COMMIT to a
temporary path, validates it, then atomically installs `camera.yaml`; a cancel,
crash, malformed result, or wrong resolution cannot replace a prior file.
`dogzilla camera-extrinsics --measured X Y Z ROLL PITCH YAW` generates the mount
YAML from metres and degrees, avoiding manual YAML formatting errors.

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

## Remaining activation gates

1. Measure the real checkerboard and run `dogzilla camera-calibrate` to generate
   `calibration/camera.yaml` for `dogzilla_mono` at exactly 640x480.
2. Measure the camera translation and RPY relative to `base_link`, then run
   `dogzilla camera-extrinsics --measured ...` with the physical values.
3. Run `dogzilla shadow --headless`, then `dogzilla shadow-check 10` while the
   robot is stationary.
4. In a second test, run `dogzilla shadow-route-check 120 1.0` in one terminal
   and `dogzilla teleop slow` in another. Drive a closed route, revisit a
   textured starting view, and stop near the start. The read-only observer must
   report at least one global RTAB loop closure before relying on the visual
   database.
5. Run `dogzilla stop`, then `dogzilla shadow-db-check`. The saved database must
   pass SQLite integrity, RTAB 0.23.7 schema, node-data, statistics, and link
   consistency checks.

Existing PGM/YAML/PBStream maps remain usable because their frame names and
working sensor transforms have not changed. They contain no historical camera
frames, so evaluating visual loop closure will require a new camera-enabled
recording or mapping run after deployment.

## Validation status

Current validation passes:

- 115 source and deployment tests plus ROS Python/launch style validation
- installed-image package discovery for the pinned camera overlay and DOGZILLA
  package
- Xacro expansion to 21 links/20 joints normally and 19/18 in shadow mode
- live `base_link -> camera_optical_frame` TF with LiDAR/IMU duplicates omitted
- physical raw camera data at about 28 Hz, 640x480, correct frame ID,
  monotonic timestamps, about 0.016-second mean age, and low delay jitter
- RTAB 0.23.7 launch-time parameter acceptance, intended subscriptions,
  isolated odometry, no devices, no privilege, no `/cmd_vel`, `publish_tf`
  false, no observed TF messages, SQLite database creation, and clean SIGINT
  save/exit
- missing-calibration refusal before any mapping or hardware service starts
- guarded intrinsic/extrinsic writers that cannot replace an existing file
  with a cancelled, malformed, implausible, or unacknowledged measurement
- read-only route acceptance logic that distinguishes global loop closures from
  proximity detections and rejects insufficient travel, poor return, and large
  odometry jumps
- atomic JSON health and route reports under the current session log, including
  failed measurements and rejection reasons
- clean Docker SIGINT persistence and read-only integrity checking across an
  RTAB database restart

Still unverified are calibrated rectification quality, combined physical
camera/scan/odometry synchronization, CPU/temperature during the full stack,
visual features while moving, real loop-closure precision, and long-duration
database behavior. Those require the two physical camera calibration files;
the deployment does not invent them or bypass the gate.
