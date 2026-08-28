# DOGZILLA S2 software framework

This document describes the software currently implemented in this repository.
It separates the operational robot stack from experimental components so that
future work does not accidentally bypass a hardware or safety boundary.

## System boundary

```text
Raspberry Pi host
|
+-- operator commands: deploy/dogzilla-map and deploy/dogzilla-mission
+-- persistent data: maps, calibration, profiles, logs, web/data
`-- Docker Compose (ROS 2 Humble, ROS_DOMAIN_ID 12)
    |
    +-- navigation OR mapping OR drive       owns robot serial devices
    +-- perception                           owns the mission camera
    `-- web                                  owns no hardware
```

Normal Mission Mode is three cooperating containers:

| Container | Responsibility | Hardware access |
| --- | --- | --- |
| `dogzilla_navigation` | LiDAR, controller bridge, localization, Nav2, command filtering | `/dev/ttyAMA0`, `/dev/ttyAMA1` |
| `dogzilla_perception` | Camera capture and detection-only vision | `/dev/video0` |
| `dogzilla_web` | Browser UI, validation, task execution and persistence | None |

All three use host networking and the same ROS domain. Only the navigation
container can move the robot. The web and perception containers cannot open the
leg controller serial port.

## Hardware ownership

| Device | Owner | Data or action |
| --- | --- | --- |
| `/dev/ttyAMA0` | `dogzilla_safe_base` only | Movement, stop, battery, 12 joint angles and optional raw IMU |
| `/dev/ttyAMA1` | MS200 driver | 2-D `LaserScan` on `/scan` |
| `/dev/video0` | One camera service at a time | 640 x 480 mono-camera frames |

The single-owner rule for `/dev/ttyAMA0` is mandatory. Yahboom programs, a
second `safe_base`, or a separate IMU reader must not run beside an operational
mode. Competing processes can split controller replies and make movement and
telemetry unreliable.

`safe_base` is the hardware safety boundary. It:

- stops the controller during startup and shutdown;
- clamps linear and angular velocity;
- uses independent movement and turning levels from 1 through 9;
- stops after a 0.6 second command timeout;
- accepts the latched `/safety/estop` signal;
- enforces the controller low-battery movement lockout;
- publishes `/battery_state`, `/joint_states`, and optional raw IMU data.

## Coordinate frames

```text
map
`-- odom                         Cartographer localization correction
    `-- base_link                scan-matched robot motion
        +-- laser_frame          x=0.000, y=0.000, z=0.180 m
        +-- imu_link             x=0.085, y=0.000, z=0.070 m
        `-- camera_link
            `-- camera_optical_frame
```

`map` is the saved-map coordinate system used by waypoints and keepout zones.
`odom` is the continuous local motion frame. `base_link` is the robot body
reference used by Cartographer and Nav2. The camera transform is valid only
after its extrinsics are physically measured.

The URDF under `ros2/dogzilla_slam/urdf/` is a visualization framework. Leg
dimensions, axes, corner assignments, offsets, and limits remain provisional;
it must not be used for control or collision checking yet.

## ROS components

### Hardware and state estimation

| Component | Role |
| --- | --- |
| `oradar_lidar` | Publishes MS200 scans on `/scan` and uses the patched motor-off shutdown path. |
| `dogzilla_safe_base` | Converts `/cmd_vel` into guarded controller commands and publishes controller telemetry. |
| `dogzilla_imu_corrector` | Applies axis, gravity, bias, scale and covariance calibration to raw IMU data. |
| Cartographer | Builds maps or localizes against a frozen PBStream. |
| `dogzilla_localization_manager` | Starts, replaces, pauses and cancels Cartographer localization trajectories from `/initialpose`. |
| `dogzilla_tf_odometry` | Converts `odom -> base_link` scan-matched TF into `/odom` for Nav2. |

DOGZILLA has no wheel encoders. Its operational odometry is derived from
Cartographer scan matching. IMU fusion is optional and should be enabled only
with a validated `calibration/imu.json`.

### Navigation and motion commands

```text
Nav2 controller
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel_nav_smoothed
  -> dogzilla_steering_guard
  -> /cmd_vel_nav
  -> twist_mux ---------------------> /cmd_vel -> dogzilla_safe_base
keyboard / web safety stop
  -> /cmd_vel_teleop ---------------^
```

The keyboard channel has higher priority than Nav2. The steering guard rejects
rapid left/right reversals without changing the requested forward motion.
Nav2 uses a regulated pure-pursuit controller, the measured 260 x 145 mm body
footprint plus padding, live LiDAR obstacle layers, static-map obstacles and
per-map keepout masks. Automatic spin and backup recovery are disabled.

Navigation diagnostics and the tuning recorder are observation-only. They can
write bounded reports, but cannot command, cancel or slow the robot.

### Web mission control

`dogzilla_web_gateway` connects the browser to ROS. It owns:

- password-authenticated HTTP and event-stream endpoints;
- live map, pose, battery, joints, vision and task state;
- initial-pose search and LiDAR-to-map verification;
- map-specific named locations, patrol areas and keepout zones;
- route preview through Nav2 without movement;
- one-at-a-time delivery, waypoint-route and patrol execution;
- autonomy speed/turn settings and the software emergency stop;
- SQLite task history and the last 25 vision alerts/photos.

The gateway validates every goal against its own occupancy grid. Browser-side
checks are for feedback only and are not trusted as the safety decision.

Task states are:

```text
queued -> running -------------------------------------> completed
             |  |
             |  +-> pausing -> paused ------------------+
             |  +------------> waiting -----------------+-> running
             +-> cancelling ---------------------------> cancelled
             `------------------------------------------> failed
```

Only one task can be active. A task in `waiting` for manual continuation still
owns the executor, so a queued patrol will not move until that task is continued
or cancelled. Pause/continue and manual checkpoints belong to the dashboard's
one-to-ten-stop waypoint mission, stored internally as task kind `delivery`.

### Vision and patrol

The perception container publishes:

| Topic | Contents |
| --- | --- |
| `/vision/status` | Active mode, detector readiness and non-actuating status |
| `/vision/detections` | Structured detections |
| `/vision/danger_confirmed` | Multi-frame confirmed person or hazard event |
| `/vision/annotated/compressed` | Browser image with overlays |
| `/vision/mode_command` | Web-to-perception configuration request |

Normal Mission Mode keeps vision action output disabled. A confirmed hazard can
create a notification and photo, but it does not stop an autonomous task.
Duplicate alerts are suppressed within the configured cooldown and stored
photos are capped at 25.

Autonomous patrol is stricter than ordinary delivery. It requires:

- a saved patrol polygon on the active map;
- at least two generated safe coverage waypoints;
- verified localization, Nav2, fresh pose, map and acceptable battery;
- vision mode `patrol`, complete detector coverage and disabled action output;
- no other active or waiting task.

At present, queuing a patrol does not automatically change the vision mode.
The operator must apply **Patrol** in the Vision panel before the patrol task
can leave `queued`.

## Persistence

| Path | Persistent content |
| --- | --- |
| `maps/` | PBStream, PGM and YAML map bundles |
| `calibration/` | IMU calibration, camera intrinsics and extrinsics |
| `profiles/` | Captured controller profiles; firmware-rest replay remains disabled |
| `logs/sessions/` | Timestamped ROS and diagnostic sessions |
| `logs/latest` | Symlink to the newest session |
| `web/data/tasks.sqlite3` | Tasks, locations, patrol areas, keepouts, hazards and alerts |
| `web/data/alerts/` | Bounded alert photographs |

Containers are disposable; these host-mounted paths are the durable state.
Docker images are build products and are not stored in Git.

## Operational status

| Area | Status |
| --- | --- |
| LiDAR-only mapping and map saving | Operational |
| Initial-pose localization and wide nearby search | Operational, requires operator confirmation |
| Nav2 waypoint, delivery and route missions | Operational, supervised development use |
| Patrol route generation and detection gate | Implemented; vision mode must currently be selected manually |
| IMU correction | Implemented and calibrated; optional because it has not improved every navigation trial |
| URDF visualization | Experimental; measurements incomplete |
| Monocular RTAB-Map shadow | Experimental, calibration-gated and non-controlling |
| Firmware-identical rest/release | Not implemented; replay intentionally disabled |

## Source map

| Path | Purpose |
| --- | --- |
| `deploy/dogzilla-map` | Main host operator command |
| `deploy/dogzilla-mission` | Mission startup, health checks, switching and rollback |
| `deploy/compose.yaml` | Container boundaries, devices and persistent mounts |
| `deploy/Dockerfile` | Reproducible ARM64 ROS image |
| `ros2/dogzilla_slam/launch/` | Combined ROS launch descriptions |
| `ros2/dogzilla_slam/config/` | Cartographer, Nav2, Twist Mux, camera and RTAB parameters |
| `ros2/dogzilla_slam/dogzilla_slam/` | Robot, localization, navigation, vision and web nodes |
| `ros2/dogzilla_slam/behavior_trees/` | Fail-closed Nav2 behavior trees |
| `ros2/dogzilla_slam/urdf/` | Provisional robot-description framework |
| `ros2/dogzilla_slam/test/` | Pure-Python and deployment contract tests |
