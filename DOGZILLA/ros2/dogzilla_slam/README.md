# DOGZILLA S2 mapping, localization, and navigation

This package replaces the separate Yahboom mapping VM with Cartographer running
inside the Raspberry Pi 5 ROS 2 Humble Docker container.

It includes a focused hardware launch that starts the MS200 LiDAR, a
single-owner controller/telemetry manager, and required static transforms. The
manager clamps commands and calls the controller's stop command directly at
startup, after a 0.6-second command timeout, and during shutdown. The same
process reads battery, all 12 motor angles, and optionally the raw controller
IMU, so no second Yahboom process competes for `/dev/ttyAMA0`.

## Recommended deployment

For normal use, run the reproducible host-side workflow from
`/home/pi/DOGZILLA`:

```bash
./deploy/dogzilla-map doctor
./deploy/dogzilla-map build
./deploy/dogzilla-map start --rviz
```

This starts the hardware bridge and mapping together, mounts `maps/` and
`logs/` from the Pi into the container, and removes the need to source ROS in
multiple manually managed shells. See `deploy/README.md` at the repository root
for all commands.

## Architecture

- `dogzilla_slam hardware.launch.py`: LiDAR, the single-owner movement/IMU
  bridge, and required static TF without a serial-port conflict.
- `dogzilla_slam mapping.launch.py`: Cartographer and the occupancy grid, with
  an optional calibrated IMU correction stage.
- `dogzilla_slam localization.launch.py`: frozen-PBStream Cartographer pure
  localization, RViz initial-pose handling, fixed-map serving, and `/odom` from
  the scan-matched `odom -> base_link` transform.
- `dogzilla_slam nav2.launch.py`: conservative Nav2 planner, holonomic DWB
  controller, live LiDAR costmaps, behaviors, and velocity smoothing.
- `dogzilla_slam full_navigation.launch.py`: hardware, localization, Twist Mux,
  and optional Nav2 in one process group.
- `dogzilla_slam save_map`: finishes the trajectory and writes PBStream,
  PGM, and YAML files. PBStream is written by Cartographer; PGM/YAML are saved
  from `/map` with Nav2 because Yahboom's ARM64 Cartographer converter crashes
  inside Cairo.
- `dogzilla_slam teleop`: safety-oriented keyboard control with live
  slow/normal/high profile selection using the `1`/`2`/`3` keys, plus bounded
  body height and whole-body look controls in controller-only drive mode.
- `dogzilla_slam servo_power`: one-shot, single-owner access to supported servo
  operations. Host-side `rest` is blocked without sending any movement or
  torque command because public action 1 is not treated as equivalent to the
  controller's private low-battery/power-button safety trajectory. Rest remains
  unavailable until that real 12-joint trajectory is captured and validated.
  `stand` uses public action 2 and cannot override Yahboom low-battery rest at
  25% or below or when battery telemetry fails.
- `dogzilla_slam firmware_rest_capture`: a command-free state machine fed by
  the single serial manager's existing battery and 12-joint readbacks. It arms
  at 30%, triggers on the configured 25% crossing, retains two seconds of
  pre-roll, records at 5 Hz, and requires a two-second stationary tail. Every
  saved result explicitly has `replay_enabled: false`.
- `dogzilla_slam firmware_rest_monitor`: dedicated read-only capture for an
  otherwise stopped, stationary robot. It invokes only `read_battery()` and
  `read_motor()` and writes raw results atomically to `/profiles/captures`.

For normal operation use `deploy/dogzilla-map`; do not start these launch files
independently on the hardware. The operator commands enforce mutually exclusive
serial ownership, logs, display setup, and safe shutdown:

```bash
./deploy/dogzilla-map start
./deploy/dogzilla-map localize test1
./deploy/dogzilla-map navigate test1
./deploy/dogzilla-map drive
```

Only one mode may run at a time. `localize` loads `test1.pbstream` as frozen
Cartographer state. `navigate` adds Nav2 and treats unknown cells in the
unfinished map as blocked. Whole-body height/pitch/yaw control is deliberately
disabled whenever LiDAR localization is active because changing body attitude
changes the scan plane.

`dogzilla_2d.lua` is LiDAR-only. `dogzilla_2d_imu.lua` keeps the same online
correlative scan matcher and adds calibrated IMU input to Cartographer's local
trajectory builder. DOGZILLA has no wheel odometry. Move slowly and remain on a
flat floor.

Yahboom's controller returns gyro degrees/second and acceleration with a
non-ROS gravity convention. The single-owner bridge converts gyro units at the
hardware boundary. `imu_corrector` then applies the six-pose axis transform,
gravity convention/scale, stationary gyro bias, and measured covariance from
`/calibration/imu.json`, publishing `/imu/data_corrected`. Orientation is marked
unavailable because Yahboom's fused RPY world convention is undocumented and
Cartographer does not need it here.

## Undeployed URDF and monocular RTAB-Map framework

The repository contains a disabled-by-default URDF/Xacro model and an isolated
RTAB-Map shadow-mode launch for a calibrated monocular camera. They are not
included in `full_mapping.launch.py`, `full_navigation.launch.py`, or the host
deployment entry points, so the normal `dogzilla` workflow does not load them.
The current image was not rebuilt and the new runtime dependencies are not yet
pinned.

The current mechanical dimensions, camera pose, leg corner assignment, joint
axes, and joint zero offsets are provisional. Do not use the model for control
or collision checking. RTAB-Map is configured to consume a rectified mono
image and `CameraInfo`, plus Cartographer's scan-matched odometry and MS200
`/scan`; it publishes no TF and keeps output under `/rtabmap_shadow`.

See `docs/URDF_RTABMAP_MONO.md` at the repository root for the camera contract,
transform-ownership warning, measurements, and gates required before a later
deployment.

## Legacy manual-container workflow

The steps below remain useful for debugging the existing `pedantic_elgamal`
container. They are not required by the recommended deployment.

### Start hardware

On the Raspberry Pi host:

```bash
cd /home/pi
sh DOGZILLA/app_dogzilla/kill_dogzilla.sh
docker start -ai pedantic_elgamal
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=12
ros2 launch dogzilla_slam hardware.launch.py
```

Use only the `dogzilla_slam hardware.launch.py` command. Do not run Yahboom's
`Navigation_bringup.launch.py` at the same time.

Leave that terminal running.

### Start mapping

Open a second Raspberry Pi terminal:

```bash
docker exec -it pedantic_elgamal bash
```

Inside the second container shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=12
ros2 launch dogzilla_slam mapping.launch.py
```

Confirm that Cartographer is publishing:

```bash
ros2 topic hz /map
```

### RViz on the Raspberry Pi desktop

The existing container was created without a `DISPLAY` value, so mapping is
headless by default. From the Pi host desktop, allow the local container and
open another shell with the display explicitly set:

```bash
xhost +local:docker
docker exec -e DISPLAY=:0 -it pedantic_elgamal bash
```

Then source ROS and start only RViz while mapping is already running:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=12
rviz2 -d /root/yahboomcar_ws/install/dogzilla_slam/share/dogzilla_slam/rviz/dogzilla_mapping.rviz
```

### Move and build the map

Only begin moving after `/map` exists. Use a third container shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=12
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Reduce the speed using the keys printed by the teleop program before moving.
Walk slowly, avoid sudden turns, keep the body level, and use `k` to stop.

### Save the map

While Cartographer is still running:

```bash
ros2 run dogzilla_slam save_map /root/yahboomcar_ws/maps/mymap
```

This creates:

```text
/root/yahboomcar_ws/maps/mymap.pbstream
/root/yahboomcar_ws/maps/mymap.pgm
/root/yahboomcar_ws/maps/mymap.yaml
```

Copy the results to the Pi host after mapping:

```bash
mkdir -p /home/pi/DOGZILLA/maps
docker cp pedantic_elgamal:/root/yahboomcar_ws/maps/mymap.pbstream /home/pi/DOGZILLA/maps/
docker cp pedantic_elgamal:/root/yahboomcar_ws/maps/mymap.pgm /home/pi/DOGZILLA/maps/
docker cp pedantic_elgamal:/root/yahboomcar_ws/maps/mymap.yaml /home/pi/DOGZILLA/maps/
```
