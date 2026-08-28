# DOGZILLA S2 pipelines

These pipelines show how sensor data, operator requests and robot commands move
through the implemented system. The arrows describe runtime ownership, not just
which files import one another.

For presentations, use the simplified four-layer 16:9 diagram as either scalable
[SVG](DOGZILLA_PIPELINE_SLIDE.svg) or a ready-to-insert
[1920 x 1080 PNG](DOGZILLA_PIPELINE_SLIDE.png).

## 1. Mapping pipeline

```text
MS200 LiDAR (/dev/ttyAMA1)
  -> oradar driver
  -> /scan
  -> Cartographer online scan matching
  -> map -> odom -> base_link TF
  -> Cartographer occupancy-grid node
  -> /map (default 0.05 m cells)
  -> RViz and map saver
  -> MAP.pbstream + MAP.pgm + MAP.yaml
```

Movement during mapping follows a separate guarded path:

```text
keyboard teleop -> /cmd_vel -> dogzilla_safe_base -> /dev/ttyAMA0
```

The PBStream preserves Cartographer's pose graph and scan data for future pure
localization. PGM/YAML preserve the 2-D occupancy grid used by Nav2 and the web
map. Both forms belong to one map bundle and should be kept together.

Optional IMU path:

```text
/dev/ttyAMA0
  -> dogzilla_safe_base
  -> /imu/data_uncalibrated
  -> dogzilla_imu_corrector + calibration/imu.json
  -> /imu/data_corrected
  -> Cartographer IMU configuration
```

## 2. Localization pipeline

```text
saved MAP.pbstream --------------------------> frozen Cartographer trajectory
saved MAP.yaml + MAP.pgm --------------------> /map
operator estimated pose + current /scan
  -> web coarse-to-fine nearby search
     (configured translation and angle window)
  -> candidate score and ambiguity checks
  -> /initialpose
  -> localization_manager starts live trajectory relative to frozen map
  -> Cartographer scan matching
  -> map -> odom -> base_link
  -> web stability + scan/map agreement checks
  -> localization state READY
```

The initial pose is an estimated search centre, not a claim that the exact pose
is already known. Mission dispatch remains blocked until the resulting pose is
stable and the live scan agrees with the saved occupancy map. The operator can
stop matching and submit a new initial pose without restarting all containers.

## 3. Navigation pipeline

```text
validated map-frame goal
  -> Nav2 planner + global costmap
     - static map
     - live /scan obstacles
     - map-specific keepout mask
  -> global path
  -> regulated pure-pursuit controller + local costmap
  -> /cmd_vel_nav_raw
  -> velocity smoother
  -> /cmd_vel_nav_smoothed
  -> steering guard
  -> /cmd_vel_nav
  -> Twist Mux
  -> /cmd_vel
  -> dogzilla_safe_base clamp + watchdog
  -> controller serial
  -> leg movement
```

Motion feedback closes the loop through Cartographer TF and `/odom`. There is
no wheel-encoder feedback. If scan matching is unstable, Nav2 receives unstable
pose and heading estimates even when its planned path is straight.

## 4. Web mission pipeline

```text
browser map click
  -> browser preview validation
  -> authenticated web API
  -> authoritative occupancy + keepout validation
  -> Nav2 ComputePathToPose preview (no movement)
  -> SQLite task in QUEUED state
  -> runtime gate
     - E-stop clear
     - selected map matches
     - localization verified
     - Nav2 action available
     - map and pose fresh
     - fresh, valid battery telemetry at or above the task threshold
  -> one NavigateToPose action per waypoint
  -> result + progress persisted
  -> browser event stream update
```

The dashboard's one-to-ten-stop waypoint mission (internally task kind
`delivery`) may pause or wait at a waypoint:

```text
waypoint reached
  +-> automatic: wait configured time -> next waypoint
  `-> manual: task WAITING -> operator Continue -> next waypoint
```

A waiting task remains active. New tasks stay queued until it is continued or
cancelled.

Invalid or stale battery telemetry blocks the start of a new task. Once a task
is moving, a single invalid or stale reading does not stop it; only a confirmed
valid reading below the configured threshold triggers the active-task safety
path.

## 5. Patrol pipeline

```text
draw polygon on active map
  -> validate bounds, area and self-intersection
  -> save patrol area for that map
  -> generate serpentine coverage points
  -> reject occupied, unknown, boundary and keepout points
  -> preview route
  -> queue patrol task
  -> patrol-specific readiness gate
  -> send coverage waypoints to Nav2 one at a time
  -> repeat configured number of cycles
  -> complete task
```

The patrol-specific gate adds the following vision pipeline:

```text
/dev/video0
  -> usb_cam
  -> /camera/image_raw
  -> dogzilla_vision in PATROL mode
     +-> person and face-presence detection
     +-> dangerous-object models
     +-> multi-frame confidence / IoU confirmation
  -> /vision/status must report ready + action_output disabled
  -> /vision/danger_confirmed
  -> web notification + annotated photo + robot map pose
  -> duplicate cooldown and last-25 retention
```

Vision is observational during patrol. It reports hazards but does not directly
move or stop the robot. The current UI requires the operator to apply Patrol
vision mode before a queued patrol can start. Selecting Patrol vision alone
does not create a movement task; a saved area must also be queued.

## 6. Keepout pipeline

```text
polygon drawn on map
  -> server validation
  -> SQLite record keyed by map name
  -> rasterized OccupancyGrid mask
  -> /keepout_filter_mask + /keepout_filter_info
  -> Nav2 global and local costmap filters
  -> planning and control avoid the zone
```

Keepout zones do not leak between maps. Switching maps loads and publishes only
the zones associated with the new map.

## 7. Camera and RTAB-Map shadow pipeline

```text
/dev/video0
  -> usb_cam
  -> rectification using measured camera intrinsics
  -> /camera/image_rect + /camera/camera_info

Cartographer odom -> base TF
  -> isolated /rtabmap_shadow/odom_input

rectified mono + CameraInfo + /scan + isolated odom
  -> namespaced RTAB-Map
  -> /rtabmap_shadow/* + persistent shadow database
```

Shadow mode publishes no movement command and no competing TF. It is an
experimental visual-memory pipeline, not the authority for current Nav2
localization. It remains gated on real camera intrinsics and measured camera
extrinsics.

## 8. Build and deployment pipeline

```text
repository source
  -> unit and contract tests
  -> deploy/Dockerfile
     - pinned Yahboom base image
     - locked ROS packages
     - checksum-pinned usb_cam fix
     - patched MS200 motor shutdown
     - colcon build of usb_cam, oradar_lidar and dogzilla_slam
  -> local dogzilla-mapping:humble image
  -> Docker Compose services
  -> health checks
  -> timestamped ROS log session
```

Normal release sequence:

```text
doctor -> test -> build -> start requested mode -> verify health -> supervised trial
```

`deploy/dogzilla-map build` replaces the local image only. It does not push to
GitHub or a Docker registry. Source backup and runtime deployment are separate:

- Git stores code, configuration and documentation.
- Docker stores the locally built runtime image.
- Host mounts store maps, calibration, databases, alerts and logs.

## Failure boundaries

| Failure | Expected result |
| --- | --- |
| LiDAR or map unavailable | Mapping/navigation health check fails; mission does not dispatch. |
| Initial pose not verified | Autonomous task remains queued. |
| Another task is waiting | New patrol/delivery remains queued. |
| Patrol vision is raw or incomplete | Patrol remains queued; ordinary delivery may still run. |
| Nav2 aborts a waypoint | Task becomes failed with the action status in its error. |
| Software E-stop | Active goal is cancelled, zero velocity is published and the latch must be reset. |
| Web restart during an active task | Interrupted task is marked failed rather than silently resumed. |
| Container shutdown | `safe_base` stops motion and the patched LiDAR driver stops the motor. |
