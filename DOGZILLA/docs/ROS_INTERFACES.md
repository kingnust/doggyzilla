# DOGZILLA ROS 2 interface reference

This is the developer-facing contract between hardware, mapping, localization,
navigation, vision and the web gateway. Default ROS domain ID is 12.

## Frames

```text
map
`-- odom
    `-- base_link
        +-- laser_frame
        +-- imu_link
        `-- camera_link
            `-- camera_optical_frame
```

| Frame | Owner and meaning |
| --- | --- |
| `map` | Saved-map coordinate system. Cartographer localization supplies the correction to `odom`. Web waypoints and keepouts are expressed here. |
| `odom` | Continuous local scan-matched frame. It is not wheel odometry. |
| `base_link` | Robot body reference used by Cartographer and Nav2. |
| `laser_frame` | MS200 scan origin, currently 0.18 m above `base_link`. |
| `imu_link` | Calibrated IMU frame, currently `(0.085, 0, 0.070)` m from `base_link`. |
| `camera_link` | Physical camera mount; operational only after measured extrinsics. |
| `camera_optical_frame` | REP-103 optical convention: z forward, x right, y down. |

Operational Cartographer owns `map -> odom -> base_link`. RTAB shadow mode
publishes no competing TF. The experimental URDF must not become a second owner
of existing LiDAR or IMU transforms.

## Core topics

### Hardware and state estimation

| Topic | Type | Publisher | Main consumers |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | MS200 driver | Cartographer, Nav2 costmaps, web pose validation, RTAB shadow |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | `dogzilla_safe_base` | Web task gate and operator telemetry |
| `/joint_states` | `sensor_msgs/msg/JointState` | `dogzilla_safe_base` | Web telemetry, URDF visualization, passive rest capture |
| `/imu/data_uncalibrated` | `sensor_msgs/msg/Imu` | `dogzilla_safe_base` when enabled | `dogzilla_imu_corrector` |
| `/imu/data_corrected` | `sensor_msgs/msg/Imu` | `dogzilla_imu_corrector` | IMU Cartographer profiles |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Cartographer occupancy grid in mapping; Nav2 map server in localization | RViz, Nav2, web gateway |
| `/odom` | `nav_msgs/msg/Odometry` | `dogzilla_tf_odometry` | Nav2 controller, velocity smoother, diagnostics, tuning recorder, web |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz or web gateway | `dogzilla_localization_manager` |
| `/localization/cancel` | `std_msgs/msg/Bool` | Web gateway | `dogzilla_localization_manager` |

Raw IMU orientation is explicitly unavailable. The controller's fused
orientation convention is not trusted as a ROS world-frame orientation.

### Velocity and safety

| Topic | Type | Publisher or source | Consumer |
| --- | --- | --- | --- |
| `/cmd_vel_nav_raw` | `geometry_msgs/msg/Twist` | Nav2 controller/behavior server | Nav2 velocity smoother |
| `/cmd_vel_nav_smoothed` | `geometry_msgs/msg/Twist` | Nav2 velocity smoother | `dogzilla_steering_guard` |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | `dogzilla_steering_guard` | Twist Mux navigation channel |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | Priority teleop or web zero-stop path | Twist Mux keyboard channel |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Twist Mux, direct teleop in mapping/drive, or web zero-stop path | `dogzilla_safe_base` |
| `/safety/estop` | `std_msgs/msg/Bool` | Web gateway | `dogzilla_safe_base` |

Twist Mux priorities are navigation 10 and keyboard 100, each with a 0.5
second input timeout. `safe_base` applies final speed clamps and its own 0.6
second watchdog after arbitration.

### Navigation support

| Topic | Type | Purpose |
| --- | --- | --- |
| `/keepout_filter_mask` | `nav_msgs/msg/OccupancyGrid` | Map-specific rasterized keepout polygons |
| `/keepout_filter_info` | `nav2_msgs/msg/CostmapFilterInfo` | Nav2 keepout-filter metadata |
| `/navigation/diagnostics` | `std_msgs/msg/String` | Warning-only health/stall observations as JSON |
| `/navigation/tuning/status` | `std_msgs/msg/String` | Read-only tuning-recorder state as JSON |
| `/navigation/tuning/marker` | `std_msgs/msg/String` | Operator marker added to the current tuning trial |

Diagnostics and tuning status are observational. Neither node publishes
velocity, E-stop, Nav2 goals, cancellations or parameter updates.

### Vision

| Topic | Type | Publisher | Purpose |
| --- | --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `usb_cam` | Raw mono-camera stream |
| `/camera/image_rect` | `sensor_msgs/msg/Image` | `image_proc` in calibrated shadow mode | Rectified RTAB input |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera driver | Intrinsics and synchronized metadata |
| `/vision/status` | `std_msgs/msg/String` | `dogzilla_vision` | Mode, readiness, model coverage and disabled-action declaration |
| `/vision/detections` | `std_msgs/msg/String` | `dogzilla_vision` | JSON detections and non-executed proposals |
| `/vision/danger_confirmed` | `std_msgs/msg/String` | `dogzilla_vision` | Multi-frame confirmed person/hazard event |
| `/vision/annotated/compressed` | `sensor_msgs/msg/CompressedImage` | `dogzilla_vision` | Browser preview and alert image source |
| `/vision/mode_command` | `std_msgs/msg/String` | Web gateway | Validated detector configuration request |
| `/vision/action_status` | `std_msgs/msg/String` | Armed `dogzilla_safe_base` only | Execution state for explicitly armed vision control |

Normal vision and patrol perception always report `action_output: disabled`.
Only the separate, interactively armed `vision_control` service can execute a
fixed proposal through the serial manager.

### RTAB shadow namespace

| Topic | Purpose |
| --- | --- |
| `/rtabmap_shadow/odom_input` | Cartographer `odom -> base_link` converted to an isolated Odometry message |
| `/rtabmap_shadow/info` | RTAB processing, graph and loop-closure statistics |
| `/rtabmap_shadow/map` | Experimental RTAB map in its independent frame |

RTAB shadow uses `/scan`, rectified mono images and scan-matched odometry. It
does not publish operational TF or control movement.

## Actions

| Action | Type | Client | Server |
| --- | --- | --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Web gateway | Nav2 BT navigator |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Web gateway | Nav2 planner server |

The web gateway sends one `NavigateToPose` goal per task waypoint. A route
preview calls `ComputePathToPose` and cannot move the robot.

## Services

| Service | Purpose |
| --- | --- |
| `/start_trajectory` | Start a live Cartographer localization trajectory |
| `/finish_trajectory` | Finish the current Cartographer trajectory |
| `/get_trajectory_states` | Discover frozen and active trajectories |
| `/write_state` | Persist a Cartographer PBStream during map saving |
| `/dogzilla_safe_base/set_parameters` | Change bounded speed, turn and drive-mode posture parameters |
| `/dogzilla_safe_base/set_parameters_atomically` | Apply autonomous speed/turn changes as one unit |
| `/controller_server/set_parameters_atomically` | Web autonomy profile update for Nav2 controller limits |
| `/velocity_smoother/set_parameters_atomically` | Web autonomy profile update for smoothed velocity limits |

Posture parameter changes are disabled in mapping/localization modes so the
physical LiDAR scan plane remains consistent with `base_link -> laser_frame`.

## Nodes by launch

### `hardware.launch.py`

- MS200 scan driver and its working LiDAR frame
- `/dogzilla_safe_base`
- optional `base_link -> imu_link` static transform

### `mapping.launch.py`

- optional `/dogzilla_imu_corrector`
- `/cartographer_node`
- `/cartographer_occupancy_grid_node`
- optional RViz

### `full_mapping.launch.py`

Includes `hardware.launch.py` and `mapping.launch.py` in one process group.

### `localization.launch.py`

- optional `/dogzilla_imu_corrector`
- frozen-state `/cartographer_node`
- `/dogzilla_localization_manager`
- `/dogzilla_tf_odometry`
- `/map_server`
- `/lifecycle_manager_localization`
- optional RViz

### `nav2.launch.py`

- `/controller_server`
- `/smoother_server`
- `/planner_server`
- `/behavior_server`
- `/bt_navigator`
- `/waypoint_follower`
- `/velocity_smoother`
- `/dogzilla_steering_guard`
- `/lifecycle_manager_navigation`

### `full_navigation.launch.py`

Includes hardware, Twist Mux, localization and optionally Nav2. When Nav2 is
enabled, it also starts `/dogzilla_navigation_diagnostics` and
`/dogzilla_navigation_tuning_recorder`.

### `vision.launch.py`

- `usb_cam` through `mono_camera.launch.py`
- `/dogzilla_vision`

### `vision_control.launch.py`

Includes vision plus a startup-gated `/dogzilla_safe_base` that accepts no
external velocity topic. It also enables the visualization-only robot
description without LiDAR or IMU transforms.

### `visual_shadow.launch.py`

Includes the calibrated camera, raw vision processor, camera-only robot
description and namespaced RTAB shadow launch. It starts no controller or LiDAR
driver and must run beside the mapping container.

## Compose modes and devices

| Service/container | Main launch | Devices |
| --- | --- | --- |
| `mapping` / `dogzilla_mapping` | `full_mapping.launch.py` | `/dev/ttyAMA0`, `/dev/ttyAMA1` |
| `drive` / `dogzilla_drive` | `safe_base` | `/dev/ttyAMA0` |
| `navigation` / `dogzilla_navigation` | `full_navigation.launch.py` | `/dev/ttyAMA0`, `/dev/ttyAMA1` |
| `shadow` / `dogzilla_visual_shadow` | `visual_shadow.launch.py` | `/dev/video0` |
| `vision` / `dogzilla_vision` | `vision.launch.py` | `/dev/video0` |
| `vision_control` / `dogzilla_vision_control` | `vision_control.launch.py` | `/dev/ttyAMA0`, `/dev/video0` |
| `perception` / `dogzilla_perception` | `vision.launch.py` | `/dev/video0` |
| `web` / `dogzilla_web` | `web_gateway` | None |

Serial-owning services are mutually exclusive. Mission Mode combines
`navigation`, `perception` and `web`. Shadow mode combines `mapping` and
`shadow`.

## Web API summary

The HTTP server listens on port 8080 by default. `/healthz` and static login
assets are public; `/api/v1/*` requires `X-Dogzilla-Password` or the compatible
Bearer form. Do not expose this port directly to the public internet.

Read endpoints provide state, map, tasks, named locations, patrol areas,
keepout zones, hazards, alerts, alert photographs, the current annotated frame
and server-sent events.

Write endpoints provide:

- delivery, generic route and patrol task creation;
- route/patrol preview;
- waypoint-mission (`delivery`) pause/continue and task cancellation;
- location, patrol-area and keepout persistence;
- initial-pose start and localization stop;
- map-switch preparation and completion;
- detector-mode changes;
- autonomous speed/turn changes;
- software E-stop and reset;
- tuning trial markers.

The authoritative implementation is `dogzilla_slam/web_http.py`. When adding
an endpoint, keep validation and state transitions in the service/core layer,
not in the browser JavaScript.

## QoS and freshness expectations

- `/map` and keepout masks use reliable, transient-local QoS so late
  subscribers receive the latest map.
- sensor data uses sensor-data QoS.
- vision status and diagnostic status use reliable, transient-local QoS.
- web task dispatch requires pose no older than 3 seconds.
- web battery start-gating requires a valid reading no older than 12 seconds.
- patrol requires vision status no older than 5 seconds.

Topic existence is not enough for readiness. Health and mission gates also
check fresh content, transforms, action availability and validated state.
