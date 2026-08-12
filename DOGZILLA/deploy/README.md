# Reproducible DOGZILLA mapping deployment

This deployment replaces the random, manually modified Docker container with a
versioned Dockerfile, Compose configuration, one combined ROS launch, persistent
host maps/logs, and one host-side command.

No deployment command pushes to GitHub.

## Runtime architecture

```text
Pi host
├── deploy/dogzilla-map          operator command
├── deploy/dogzilla-mission      navigation + web coordinator
├── maps/                        persistent PBStream/PGM/YAML files
├── logs/latest                  latest timestamped ROS log session
├── calibration/imu.json         robot-specific IMU calibration
├── calibration/camera.yaml      required 640x480 mono intrinsics
├── calibration/camera_extrinsics.yaml  measured camera mount
└── Docker Compose
    ├── dogzilla_mapping          full mapping mode
    ├── dogzilla_drive            controller-only mode
    ├── dogzilla_navigation       localization or localization + Nav2
        ├── MS200 LiDAR           /dev/ttyAMA1 -> /scan
        ├── serial manager        sole owner of /dev/ttyAMA0
        │   ├── movement          /cmd_vel -> controller + watchdog
        │   ├── battery           controller -> /battery_state
        │   ├── 12 servo angles   controller -> /joint_states
        │   ├── raw IMU           controller -> /imu/data_uncalibrated
        │   └── posture           bounded height/pitch/yaw in drive mode
        ├── Cartographer          mapping or frozen-PBStream localization
        ├── TF odometry           scan-matched TF -> /odom
        ├── Nav2                  optional planner/controller/behaviors
        ├── Twist Mux             teleop priority over Nav2 commands
        └── patched shutdown      deactivates the MS200 motor
    ├── dogzilla_visual_shadow    camera + URDF + isolated RTAB-Map
    │   ├── /dev/video0           sole camera owner
    │   ├── robot-state publisher camera TF; no duplicate LiDAR/IMU TF
    │   ├── TF odometry           reads Cartographer odom -> base_link
    │   └── RTAB-Map              no motion command, serial device, or TF output
    └── dogzilla_web              browser mission UI; no serial devices
```

The three serial-owning Compose services are mutually exclusive. Every mode
uses the same serial-manager implementation, so movement, battery reads,
motor-angle reads, raw IMU reads, and posture writes never compete for
`/dev/ttyAMA0`. The device-free web service may run beside navigation.
The visual shadow service may run only beside mapping. It receives `/scan` and
scan-matched odometry over ROS but has no access to either serial port.

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

### Monocular camera and visual shadow mapping

Test the raw USB camera without starting the legs, LiDAR, or mapping:

```bash
dogzilla camera-check 12
```

`camera-check` is the operator alias plus the `camera-check` subcommand. `12`
is the number of seconds sampled. The command gracefully closes Yahboom's
camera application if it is running, opens only `/dev/video0` in a temporary
container, checks 640x480 images, `camera_optical_frame`, rate, monotonic
timestamps, bounded latency and jitter, then removes the container. It does
not require or pretend to provide intrinsic calibration.

The deployed 30 Hz MJPEG profile produces about 27-28 Hz on this Pi. The image
uses a checksum-pinned upstream `usb_cam` frame-draining fix because Humble's
0.8.0 binary accumulated old V4L2 frames on this 120 Hz camera. A clean
camera-only test reduced mean capture-to-receipt age from about 0.91 seconds to
0.016 seconds while retaining the original capture timestamp. Warnings about
unsupported white-balance, exposure, and focus controls are expected with this
camera.

From a terminal on the Pi monitor, generate intrinsics without moving files by
hand:

```bash
dogzilla camera-calibrate 8x6 0.025
```

`8x6` counts inner checkerboard corners. `0.025` is the measured square edge in
metres; replace both examples with the real board. The command automatically
finds the local Pi display, opens only `/dev/video0` in an isolated container,
and runs the official ROS GUI. Move the checkerboard while DOGZILLA stays
still. Fill the coverage bars, click `CALIBRATE`, inspect the rectified image,
then click `COMMIT`. The wrapper validates the committed 640x480 file and
atomically installs it as `calibration/camera.yaml`. Closing without `COMMIT`
leaves the existing file unchanged.

The calibration container has no network or serial devices and a 30-minute
maximum lifetime. If its terminal is forcibly closed, the next camera,
shadow, or stop command reconciles the labelled session: a valid COMMIT is
recovered, while an empty or invalid partial result is removed.

Save physically measured mount coordinates without hand-editing YAML:

```bash
dogzilla camera-extrinsics --measured X Y Z ROLL PITCH YAW
```

XYZ is in metres and RPY is in degrees. The command converts RPY to radians,
validates the transform, backs up an existing file, and atomically writes
`calibration/camera_extrinsics.yaml`. `--measured` must not be used with guessed
values.

Visual shadow mapping is deliberately calibration-gated. It requires:

- `calibration/camera.yaml`, produced for `dogzilla_mono` at exactly 640x480.
- `calibration/camera_extrinsics.yaml`, copied from the example and filled
  with measured `base_link -> camera_link` XYZ metres and RPY radians, with
  `measured: true` only after measurement.

After both files validate, start one combined headless session:

```bash
dogzilla shadow --headless
```

`shadow` starts normal Cartographer mapping first, then the calibrated camera,
camera TF, rectification, and namespaced RTAB-Map service. `--headless` disables
RViz, which is appropriate over SSH. Add `--imu` only when the existing
`imu.json` calibration should be fused into Cartographer:

```bash
dogzilla shadow --headless --imu
```

Before moving the robot, validate all four synchronized inputs:

```bash
dogzilla shadow-check 10
```

`shadow-check` first samples ten seconds of rectified calibrated images and
compares the live CameraInfo matrices with the installed YAML. It then samples
the complete pipeline for ten seconds: image, scan, scan-matched odometry, and
RTAB `Info`. It rejects stale or sparse streams, bad LiDAR metadata, unusable
ranges, missing odometry covariance, weak timestamp overlap, or a running RTAB
node whose processing timestamp does not advance. Docker reports the shadow
service healthy only after receiving an RTAB message with a processed reference
node and non-empty working memory. The validator reports loop closures, but
does not require one while the robot is stationary.

Both validators write atomic JSON evidence into the current session directory:
`shadow-camera-report.json` and `shadow-health-report.json`. A failed check also
writes a report, so its measurements and exact rejection reasons are retained.

After the stationary check passes, test an actual closed route with two Pi
terminals. In terminal 1, start the passive observer:

```bash
dogzilla shadow-route-check 120 1.0
```

`120` is the route observation time in seconds after a five-second camera
gate. `1.0` is the minimum scan-odometry path length in metres. Wait until it
prints `Begin the route now.` In terminal 2, open slow keyboard control:

```bash
dogzilla teleop slow
```

Drive at least one metre, return within 0.75 m of the starting position, and
point the camera toward a previously seen, textured view. Stop the robot before
terminal 1 finishes. The route passes only if all synchronized streams remain
healthy, odometry has no half-metre jump, and RTAB reports a global
`loop_closure_id`. A proximity detection alone does not pass. The observer
never publishes `/cmd_vel` and cannot move the robot. It stores
`route-camera-report.json` and `shadow-route-report.json` beside the ROS logs;
`logs/latest` points to that session from the host.

Stop both services and the LiDAR safely with:

```bash
dogzilla stop
```

After RTAB has stopped and finished saving, validate the latest database:

```bash
dogzilla shadow-db-check
```

This opens the database read-only, runs SQLite `quick_check`, verifies the
pinned RTAB database version, required tables, saved nodes, Node/Data
relationships, Statistics rows, and Link references. It writes
`shadow-database-report.json` into `logs/latest`. To inspect an older session,
pass its directory name from `logs/sessions`, for example `dogzilla
shadow-db-check 20260812T120000Z`.

Without both camera calibration files, `shadow` exits before starting mapping
or opening any hardware. Normal `start`, `drive`, `localize`, and `navigate`
commands do not load the camera, URDF publisher, or RTAB-Map.

Open keyboard control:

```bash
./deploy/dogzilla-map teleop
```

`teleop` opens the DOGZILLA keyboard menu with the Yahboom mobile app's
normal/default speed: controller step 10 instead of the previous minimum step
4. Use `w/s` to move forward/back, `a/d` to strafe, `q/e` to turn, and
`Space` or `k` to stop. Press `1`, `2`, or `3` while it is open to switch to
slow, normal, or high. The safety node still clamps motion and applies its
0.6-second watchdog. When teleop closes, the operator command restores slow.

The optional argument chooses the initial menu profile:

```bash
./deploy/dogzilla-map teleop slow
./deploy/dogzilla-map teleop normal
./deploy/dogzilla-map teleop high
```

`slow`, `normal`, and `high` use controller steps 4, 10, and 20 respectively,
along with the matching Yahboom pace command. Use `normal` for general driving.
Use `slow` while prioritizing map quality. `high` is the controller maximum and
can reduce scan-matching quality or make the robot unstable; test it only in a
clear area with an immediate stop available.

In controller-only `drive` mode, the same menu also provides:

- `r` / `f`: raise / lower body height in 5 mm steps, clamped to 75–110 mm.
- `i` / `,`: look up / down using bounded whole-body pitch.
- `j` / `l`: look left / right using bounded whole-body yaw.
- `c`: center pitch/yaw and restore 105 mm body height.

DOGZILLA has no separate neck; “head” control uses whole-body attitude, as in
Yahboom's look controls. These keys are deliberately disabled during mapping,
localization, and Nav2 because moving the body also changes the LiDAR plane and
invalidates its fixed transform.

### Drive without mapping

For keyboard control without spinning the LiDAR or running mapping:

```bash
./deploy/dogzilla-map drive
./deploy/dogzilla-map status
./deploy/dogzilla-map teleop
./deploy/dogzilla-map stop
```

`drive` starts only `dogzilla_drive`, which exposes `/dev/ttyAMA0` to the safe
controller bridge. It does not expose `/dev/ttyAMA1` and does not start the
LiDAR, Cartographer, occupancy grid, or RViz. Mapping and drive modes are
mutually exclusive because both require the controller serial port. `teleop`,
`status`, `logs`, `shell`, and `stop` automatically use whichever mode is
active.

### Firmware rest and animated stand

The real controller low-battery trajectory must be captured before host-side
rest can be implemented. Capture is passive: it uses only Yahboom's supported
battery and 12-joint read requests. It does not call movement, action,
motor-angle, motor-speed, load, or unload commands.

When mapping, controller-only drive, localization, or navigation is running,
the existing single serial manager automatically watches for the configured
25% low-battery transition. Joint feedback remains at the normal 1 Hz rate
until the battery reaches the 30% capture window, then temporarily changes to
5 Hz. The recorder retains a two-second pre-roll and requires measurable joint
movement followed by two stationary seconds.

For a stationary robot that is already near—but still above—the low-battery
threshold, stop all other modes and start the read-only monitor:

```bash
./deploy/dogzilla-map rest-capture
```

Type `CAPTURE` exactly. The command stops competing vendor/OLED serial readers,
claims `/dev/ttyAMA0` exclusively, and sends read requests only. Place DOGZILLA
on a clear, level floor before starting. Do not press the physical power button:
allow the firmware's natural low-battery routine to occur so the Raspberry Pi
stays powered long enough to save the file. Do not intentionally hold a fully
charged robot standing for hours merely to drain it; normal operation can arm
the automatic recorder instead.

Raw results are written atomically under `profiles/captures/`. A successful
capture has `status: captured_unvalidated` and `replay_enabled: false`.
Telemetry loss, no observed motion, or failure to reach a stable tail produces
an `incomplete` diagnostic, never a replay profile. If the monitor starts after
the battery is already low and the dog is already resting, it deliberately
skips capture because the descent has already been missed. Raw captures are
ignored by Git until they have been reviewed and explicitly promoted.

After stopping mapping or drive mode, place DOGZILLA on a clear, level floor,
keep hands away from every joint, then run:

```bash
./deploy/dogzilla-map rest
```

`rest` is intentionally disabled and sends no movement or torque command. The
public preset action group 1 has not been verified as identical to the lower
controller's private low-battery/power-button safety trajectory. A real
12-joint trajectory must be captured from that firmware behavior and validated
before host-side replay is enabled. Guessing a folded pose or treating action 1
as the same routine is not accepted as a safe implementation.

To run the matching animated stand-up sequence:

```bash
./deploy/dogzilla-map stand
```

`stand` requires typing `STAND`. It first reads Yahboom's battery percentage,
stops locomotion, loads torque while holding every joint at its current folded
position, and then runs firmware action group 2, Yahboom's animated stand-up
sequence. It allows four seconds for the documented three-second action. At 25% or below—or
if the battery read fails—`stand` exits before loading torque or starting the
animation, allowing the controller's built-in low-battery rest to win. Support
the body and keep hands clear because every leg moves during the animation.
Both commands refuse to run while `mapping`, `drive`, or `navigation` is
active, temporarily pause the OLED serial reader, claim `/dev/ttyAMA0`
exclusively, and restore the OLED service afterward.

### Localization on `test1`

The default localization map is the comprehensive but unfinished `test1` map.
Start scan-matched pure localization without autonomous planning:

```bash
./deploy/dogzilla-map localize test1
```

`localize` loads `test1.pbstream` as frozen Cartographer state and serves
`test1.yaml` as the fixed occupancy map. Cartographer estimates local motion
directly from the 10 Hz LiDAR because DOGZILLA has no wheel odometry. The
deployment converts Cartographer's `odom -> base_link` transform into `/odom`
for downstream ROS tools. Unknown cells in the unfinished map remain blocked.

From SSH, append `--headless`. From a Pi monitor terminal, automatic display
detection opens RViz; use its **2D Pose Estimate** tool to place DOGZILLA on the
map. The localization manager safely restarts only the live trajectory against
the frozen PBStream. Without an initial pose, Cartographer attempts global
scan matching, which can take longer in repetitive rooms.

After calibrated IMU validation, optional fused localization is:

```bash
./deploy/dogzilla-map localize test1 --imu
```

### Nav2 path planning on `test1`

Start the same localization plus conservative holonomic Nav2:

```bash
./deploy/dogzilla-map navigate test1
```

Nav2 uses a Smac 2D global planner, DWB holonomic local controller, 0.32 m
conservative robot radius, live LiDAR obstacle layers, velocity smoothing, and
maximum commands of 0.10 m/s and 0.30 rad/s. Twist Mux gives keyboard teleop
priority over autonomous commands, and every final command still passes through
the serial manager's clamp, low-battery inhibit, and 0.6-second watchdog.

Wait for `status` to report healthy, set the initial pose in RViz, confirm that
the scan aligns with walls, and only then use **Nav2 Goal**. Keep the robot in a
clear test area with immediate access to `Space`, `k`, or `dogzilla-map stop`.
The unfinished part of `test1` is intentionally not traversable.

### Mission mode on a saved map

After saving the room map and stopping mapping, start localization, Nav2, and
the browser mission gateway with one host command:

```bash
./deploy/dogzilla-map mission room1 --headless
```

`mission` selects the guarded mission coordinator. `room1` requires the
non-empty files `maps/room1.pbstream`, `maps/room1.yaml`, and
`maps/room1.pgm`. `--headless` starts navigation without RViz; this is the
normal browser or SSH mode. `mission start room1 --headless` is an equivalent,
more explicit spelling. If neither `--headless` nor `--rviz` is supplied,
Mission Mode defaults to headless operation.

The coordinator refuses to replace an active mapping, drive, navigation, or
web container. On a free system it starts navigation first, starts the web
gateway second, waits for both container health checks, and verifies the
required ROS topics, Nav2 actions and nodes, and `map -> base_link` transform.
It gives both services one ROS log session and records only its own managed
state in `logs/mission-current`. It never queues a goal automatically. If any
startup check fails or startup is interrupted, it stops the web gateway first
and then follows the normal navigation shutdown path.

Inspect a managed session without moving the robot:

```bash
./deploy/dogzilla-map mission status
```

`status` prints the coordinator state, selected map, log session, navigation
container status, ROS topics, web container status, and dashboard address.

Print the existing private browser token:

```bash
./deploy/dogzilla-map mission token
```

`token` prints the existing token used by the dashboard login. It does not
start a service or queue a mission. Keep this token private.

Follow both navigation and web logs:

```bash
./deploy/dogzilla-map mission logs
```

`logs` follows the last 200 lines from both containers. Press `Ctrl+C` to stop
following output; this does not stop Mission Mode.

Stop a managed session in the safe order:

```bash
./deploy/dogzilla-map mission stop
```

`stop` stops the web gateway before navigation, releases the serial ports, and
runs the existing LiDAR motor-off fallback. If no managed Mission Mode state
exists, it reports that nothing was stopped and leaves manually started
services alone. The ordinary `./deploy/dogzilla-map stop` command recognizes a
managed mission and uses the same web-first order.

Before queuing a delivery, confirm that the displayed scan aligns with the
saved walls and that localization is stable. On the Pi monitor,
`./deploy/dogzilla-map rviz` can open RViz for **2D Pose Estimate** while the
headless mission is running. The browser then previews the real Nav2 path and
dispatches its validated pickup and drop-off points one at a time. Keep the
robot supervised and the software emergency stop visible during testing.

### Serial telemetry

The single serial owner publishes standard ROS messages in mapping, drive, and
navigation modes:

- `/battery_state` (`sensor_msgs/BatteryState`), with percentage in 0.0–1.0.
- `/joint_states` (`sensor_msgs/JointState`), with 12 encoder-reported servo
  angles converted from degrees to radians.
- `/imu/data_uncalibrated` when the IMU option is enabled.

At 25% battery or below the manager sends movement stop and ignores further ROS
velocity commands. It does not fight the controller's built-in lying-down
behavior and re-enables ROS movement only after a reading of at least 28%.

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

In mapping mode, `stop` gives ROS up to 20 seconds to shut down, sends a second
explicit MS200 motor-off command, and removes temporary X11 permission. In
controller-only mode it stops the legs and releases `/dev/ttyAMA0` without
touching the already-unused LiDAR port. Saved maps remain on the Pi. To stop
only a spinning LiDAR:

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

## Current limitations and next validation

The system now implements mapping, pure localization, scan-derived odometry,
and Nav2 planning/control. `test1` is explicitly unfinished, so planning is
restricted to its known free cells. Before unattended operation, record
repeatable rosbag trials, measure localization recovery in similar-looking
rooms, physically measure the footprint in the widest stance, and tune DWB and
inflation values from controlled runs. The first Nav2 tests must remain
supervised; software cannot replace a physical emergency stop or protect
against sudden battery disconnection.
