# DOGZILLA S2 development workspace

This repository contains the Yahboom DOGZILLA software plus a Pi-only ROS 2
mapping prototype. The mapping runtime is packaged as a reproducible Docker
image and operated with one host-side command.

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

## Web mission control

An optional local web dashboard can monitor battery, pose, ROS/Nav2 state, and
task progress, as well as queue pickup/drop-off deliveries and latch a software
emergency stop. It runs in a separate container with no serial-device access.

See [web/README.md](web/README.md) for setup, safety behavior, and the API
architecture.
