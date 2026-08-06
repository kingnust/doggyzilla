# Reproducible DOGZILLA mapping deployment

This deployment replaces the random, manually modified Docker container with a
versioned Dockerfile, Compose configuration, one combined ROS launch, persistent
host maps/logs, and one host-side command.

No deployment command pushes to GitHub.

## Runtime architecture

```text
Pi host
├── deploy/dogzilla-map          operator command
├── maps/                        persistent PBStream/PGM/YAML files
├── logs/latest                  latest timestamped ROS log session
├── calibration/imu.json         robot-specific IMU calibration
└── Docker Compose
    └── dogzilla_mapping
        ├── MS200 LiDAR           /dev/ttyAMA1 -> /scan
        ├── safe_base             one owner of /dev/ttyAMA0
        │   ├── movement          /cmd_vel -> controller
        │   └── raw IMU           controller -> /imu/data_uncalibrated
        ├── IMU corrector         optional axes/bias/covariance correction
        ├── Cartographer          scan matching (+ optional IMU) -> /map
        ├── occupancy grid        /map at 0.05 m/cell
        ├── RViz                  optional Pi-monitor window
        └── patched shutdown      deactivates the MS200 motor
```

## First build

Run from `/home/pi/DOGZILLA`:

```bash
./deploy/dogzilla-map doctor
```

`doctor` checks Docker, Compose, the pinned Yahboom base image, both serial
devices, free storage, and whether this terminal has a desktop display.

```bash
./deploy/dogzilla-map build
```

`build` creates the local `dogzilla-mapping:humble` image from the Dockerfile.
It does not push the image or repository anywhere.

The build is intentionally refused below 2.5 GiB free disk space. If the
doctor reports less than that, free or expand storage before building; do not
use the low-disk override unless the risk of filling the root filesystem is
acceptable. The image has been built and smoke-tested on this Pi.

## Daily use

From a terminal on the Pi monitor, start mapping with one command:

```bash
./deploy/dogzilla-map start
```

`start` detects `DISPLAY`: it opens RViz on the Pi desktop and automatically
uses headless mode over SSH. `--rviz` or `--headless` can override that choice.
The default remains the tested LiDAR-only profile.

From SSH, start without a graphical window:

```bash
./deploy/dogzilla-map start --headless
```

`--headless` starts the same mapping stack without a graphical window, which
is the correct mode for SSH.

Check the container and ROS topics:

```bash
./deploy/dogzilla-map status
```

`status` shows the Compose container state and the ROS topics visible inside
it. Compose reports `healthy` after both `/scan` and `/map` exist.

Open keyboard control:

```bash
./deploy/dogzilla-map teleop
```

`teleop` opens the ROS keyboard controller inside the running mapping
container. The safety node still clamps motion and applies its watchdog.

Save a map directly to the host `maps/` directory:

```bash
./deploy/dogzilla-map save room1
```

`save` finishes the active Cartographer trajectory and writes
`room1.pbstream`, `room1.pgm`, and `room1.yaml` directly into host `maps/`.

Saving finishes the active Cartographer trajectory. Stop or restart mapping
before beginning a new map.

Follow logs:

```bash
./deploy/dogzilla-map logs
```

`logs` follows the last 200 container log lines; press `Ctrl+C` to stop
following logs without stopping mapping. ROS file logs are grouped under a UTC
session name in `logs/sessions/`; `logs/latest` always points to the newest one.

Stop safely:

```bash
./deploy/dogzilla-map stop
```

`stop` gives ROS up to 20 seconds to shut down safely, sends the fallback
LiDAR motor-off packet, and leaves saved maps on the Pi.

`stop` stops ROS, sends a second explicit MS200 motor-off command, and removes
the temporary X11 permission. To stop only a spinning LiDAR when mapping is not
running:

```bash
./deploy/dogzilla-map lidar-off
```

`lidar-off` sends only the MS200 deactivate packet on `/dev/ttyAMA1`; it does
not start ROS and cannot command the legs.

The safe base owns `/dev/ttyAMA0` exclusively and sends the controller stop
command at startup, after a 0.6-second command timeout, and during shutdown.
The host command temporarily stops `yahboom_oled.service`, which normally owns
the same controller port, and records that state in `logs/`. A normal `stop`
restores the OLED service after releasing the controller.
The deployment patches Yahboom's MS200 driver to call `Activate()` after
connecting and `Deactive()` before closing `/dev/ttyAMA1`. The unmodified
driver neither reactivates a deliberately stopped scanner nor stops its motor
when it closes the serial port.

The vendor binary also has an existing ARM64 shutdown buffer-overflow bug. It
can print `buffer overflow detected` after Ctrl+C even with the motor patch.
The host command therefore sends the independent motor-off packet after the
container exits; this fallback was tested directly on `/dev/ttyAMA1`.

Install the shutdown guard once so an orderly Raspberry Pi shutdown invokes
the same stop path before Docker exits:

```bash
./deploy/dogzilla-map shutdown-install
```

`shutdown-install` copies and enables the included systemd unit. It asks for
`sudo` because `/etc/systemd/system` is system-owned. Sudden power removal still
cannot run software shutdown code, so use the normal OS shutdown command.

## IMU calibration and fusion

Stop mapping, place the robot where you can safely support its body, then run:

```bash
./deploy/dogzilla-map imu-calibrate
```

The command does not start the LiDAR. It sends a movement STOP (servo torque is
not unloaded), temporarily pauses the OLED process that shares `/dev/ttyAMA0`,
and guides six motionless body poses.
Keep the LiDAR and legs from bearing the robot's weight while tilted. It fits
the raw axes to ROS x-forward/y-left/z-up, corrects Yahboom's gravity direction,
converts gyro degrees/s to radians/s, measures stationary gyro bias and both
sample covariance matrices, and records sampling/timing statistics in
`calibration/imu.json`.

After calibration, start fused mapping:

```bash
./deploy/dogzilla-map start --imu
```

Add `--rviz` or `--headless` if automatic display selection is not desired.
The command refuses to enable fusion if the calibration is absent or did not
pass the six-pose axis fit. In this profile Cartographer's local trajectory
builder combines corrected angular velocity and gravity with its existing
LiDAR scan-based motion estimate; no fake wheel odometry is introduced.

Leave the robot motionless and validate the live stream:

```bash
./deploy/dogzilla-map imu-check 10
```

`imu-check` samples for 10 seconds and fails on a low rate, non-monotonic or
stale timestamps, gaps over 0.25 seconds, bad gravity magnitude, missing
covariance, or an unexpected frame ID.

The calibration belongs to this physical DOGZILLA. Back it up, but do not copy
another robot's file. Recalibrate after controller/IMU replacement or a change
to the body mounting orientation.

## Optional desktop shortcut

Install it once from the Pi desktop account:

```bash
./deploy/dogzilla-map desktop-install
```

The shortcut runs the same `start` command in a terminal, including automatic
display selection and predictable logging.

## Manual ROS shell

```bash
./deploy/dogzilla-map shell
```

`shell` opens an extra shell in the running container with ROS 2 Humble, the
Yahboom workspace, and domain ID 12 already sourced.

The shell opens with ROS 2 Humble, the Yahboom workspace, and domain ID 12
already configured.

## Rebuilding

The image is based on `yahboomtechnology/ros-humble:3.8`. The ROS apt-source
bootstrap is SHA-256 verified, and the additional ROS packages are pinned in
`ros-packages.lock`. Rebuild after changing the ROS package or deployment files:

```bash
./deploy/dogzilla-map build
```

Re-running `build` updates the local image from the current checked-out files.

The upstream Yahboom image does not publish a repository digest. The build
therefore verifies that its local `3.8` tag has the known ARM64 image ID
`sha256:7e79c61b64b8...e566047`, then gives it an internal pinned alias. If
Yahboom changes the tag, the build stops and requires a deliberate review
instead of silently using different contents. Availability of that historical
tag on a completely fresh machine remains an upstream limitation.

## Current scope and next stage

This prototype performs manual 2D mapping with a stable LiDAR-only profile and
an opt-in calibrated LiDAR+IMU profile. It saves both a Cartographer PBStream
and a Nav2-compatible PGM/YAML map. It does not yet run autonomous localization
or path planning.

The next development stage should:

1. Record `/scan`, `/tf`, and movement trials with rosbag for repeatable tests.
2. Compare LiDAR-only and fused rosbag trials before making IMU fusion default.
3. Add a localization launch, using either Cartographer pure localization with
   the PBStream or Nav2 AMCL with the PGM/YAML map.
4. Add Nav2 only after a stable `map -> odom -> base_link` transform exists;
   DOGZILLA has no wheel odometry, so this needs scan/IMU-based odometry or a
   separately validated equivalent.
5. Keep autonomous velocity output behind `safe_base` and add a physical
   emergency-stop procedure before unattended testing.
