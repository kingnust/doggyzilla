# DOGZILLA S2 development workspace

This repository contains the Yahboom DOGZILLA software plus a Pi-only ROS 2
mapping prototype. The mapping runtime is packaged as a reproducible Docker
image and operated with one host-side command.

The complete developer handoff starts at
[docs/README.md](docs/README.md). New maintainers should read
[docs/DEVELOPER_HANDOFF.md](docs/DEVELOPER_HANDOFF.md) and the
[operations runbook](docs/OPERATIONS_RUNBOOK.md) before changing or deploying
the robot. The controller/serial boundary, including what is and is not known
about the embedded firmware, is documented in
[docs/FIRMWARE_AND_SERIAL.md](docs/FIRMWARE_AND_SERIAL.md).

The current component boundaries, hardware ownership, ROS frames, operational
status, and source layout are documented in
[docs/FRAMEWORK.md](docs/FRAMEWORK.md). End-to-end mapping, localization,
navigation, mission, patrol, vision, keepout, shadow, and deployment flows are
documented in [docs/PIPELINES.md](docs/PIPELINES.md).

## Mapping architecture

```text
/dev/ttyAMA1 -> MS200 driver -> /scan -> Cartographer -> /map
/dev/ttyAMA0 <- safe_base <- /cmd_vel <- keyboard teleop
base_link -> laser_frame                 static TF, z = 0.18 m
```

`safe_base` is the only mapping process that opens the DOGZILLA controller
serial port. It clamps movement commands and stops the robot at startup, after
a 0.6-second command timeout, and on normal shutdown.

## Recommended workflow

All commands below run on the Raspberry Pi host from this repository:

```bash
cd /home/pi/DOGZILLA
./deploy/dogzilla-map doctor
./deploy/dogzilla-map build
./deploy/dogzilla-map start --rviz
```

Use `--headless` instead of `--rviz` when starting from SSH. Then use:

```bash
./deploy/dogzilla-map teleop
./deploy/dogzilla-map save room1
./deploy/dogzilla-map stop
```

Maps persist in `maps/` and ROS logs persist in `logs/`, even after the
container is recreated. See [deploy/README.md](deploy/README.md) for the full
deployment guide and [ros2/dogzilla_slam/README.md](ros2/dogzilla_slam/README.md)
for ROS package details and the legacy manual-container workflow.

The deployment scripts do not commit or push to GitHub.

## Future URDF and visual mapping work

A source-only, disabled framework now describes the robot and prepares an
isolated RTAB-Map experiment using the monocular camera, MS200 scan, and
Cartographer motion estimate. The current image was not rebuilt, its runtime
dependencies are not pinned, and normal mapping commands do not invoke it. See
[docs/URDF_RTABMAP_MONO.md](docs/URDF_RTABMAP_MONO.md) for its status and the
measurements and camera calibration required before deployment.

## Web mission control

An optional local web dashboard renders the occupancy map and live robot pose,
builds click-only waypoint missions with up to ten ordered stops, previews
Nav2 paths, stores named locations, monitors task progress, and provides a
latched software emergency stop. Each intermediate stop can continue after a
timed wait or require an operator to press the prominent Continue button. The
dashboard runs in a separate container with no serial-device access.

After mapping is saved and stopped, Mission Mode starts Nav2 and the dashboard
together:

```bash
./deploy/dogzilla-map mission MAP --headless
```

It verifies readiness, shares one log session, and rolls back safely if
startup fails. It does not queue a goal or move the robot automatically.

See [web/README.md](web/README.md) for setup, safety behavior, and the API
architecture.
