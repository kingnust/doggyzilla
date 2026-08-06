# DOGZILLA S2 Pi-only mapping

This package replaces the separate Yahboom mapping VM with Cartographer running
inside the Raspberry Pi 5 ROS 2 Humble Docker container.

It includes a focused mapping-time hardware launch. It starts the MS200 LiDAR,
a single-owner safe controller bridge, and required static transforms. The
bridge clamps commands and calls the controller's stop command directly at
startup, after a 0.6-second command timeout, and during shutdown. When enabled,
the same bridge also reads the controller IMU; it never starts Yahboom's second
`/dev/ttyAMA0` owner.

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
- `dogzilla_slam save_map`: finishes the trajectory and writes PBStream,
  PGM, and YAML files. PBStream is written by Cartographer; PGM/YAML are saved
  from `/map` with Nav2 because Yahboom's ARM64 Cartographer converter crashes
  inside Cairo.

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
