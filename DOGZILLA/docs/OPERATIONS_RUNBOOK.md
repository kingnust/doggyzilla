# DOGZILLA S2 operations runbook

This is the operator procedure for the Raspberry Pi. Run these commands in a
terminal **on the Pi host**, not inside a Docker container. Commands that can
move the robot are marked **MOVEMENT POSSIBLE**.

The examples use the installed shell alias `dogzilla`. If the alias is not
available, replace `dogzilla` with:

```bash
/home/pi/DOGZILLA/deploy/dogzilla-map
```

The absolute path runs the same host wrapper from any directory. Do not use
`docker exec` for ordinary operation; the wrapper supplies the correct ROS
domain, devices, mounts, logging, service handling and cleanup.

## Physical preflight

Before any mode that can move DOGZILLA:

1. Put it on a clear, level, non-slip floor.
2. Keep hands, cables and loose objects out of the legs.
3. Keep the physical power button accessible.
4. Confirm the battery is charged and not in firmware low-battery rest.
5. Confirm no Yahboom mobile app, notebook or manual Python process is using
   `/dev/ttyAMA0`, `/dev/ttyAMA1` or `/dev/video0`.
6. Start with speed level 1 after any controller, navigation or pose change.

The browser emergency stop is useful but is not a certified physical E-stop.
If motion is unsafe, use the physical power control.

## Setup and health checks

Open a terminal on the Pi and run:

```bash
cd /home/pi/DOGZILLA
dogzilla doctor
```

`cd` changes the terminal to the repository. `dogzilla doctor` checks Docker,
Compose, the pinned base image, devices, disk space and desktop display. It
does not start ROS or move the robot.

If `dogzilla` is not recognized, load the alias once:

```bash
source /home/pi/.bash_aliases
```

`source` evaluates the file in the current shell. The absolute filename avoids
depending on the current directory. A new terminal normally loads the alias
automatically.

Inspect the current deployment without starting a mode:

```bash
dogzilla status
```

`status` lists relevant Compose containers and, when one is active, important
ROS topics. It does not command movement.

## Build and deployment model

After source changes and tests, build the local image:

```bash
dogzilla build
```

`build` builds `dogzilla-mapping:humble` from the pinned Dockerfile. It changes
the local Docker image but does not start a robot mode, commit Git, push GitHub
or upload the image. Source edits are not used by an existing container until
this build completes and the affected mode is restarted.

There is no separate generic `deploy` command. Deployment means:

1. build the image;
2. stop the old mode;
3. start the intended mode;
4. verify status and logs.

Do not rebuild while the robot is walking or while a map save is in progress.

## Mapping a room

### Normal posture

**MOVEMENT POSSIBLE after teleop starts.** Start LiDAR-only mapping:

```bash
dogzilla start normal --headless
```

`start` launches the mapping service. `normal` leaves the controller at its
normal firmware body height. `--headless` suppresses RViz, which is appropriate
over SSH. On the Pi monitor, replace it with `--rviz` to open RViz. Omitting
both flags automatically uses the detected display.

### Low posture

For a more stable, fixed scan plane:

```bash
dogzilla start low --headless
```

`low` performs a guarded change to 75 mm before mapping. Use the same posture
when later localizing on this map. It is not a rest or servo-release command.

Add corrected IMU input only after a valid calibration and comparison trial:

```bash
dogzilla start normal --headless --imu
```

`--imu` enables the calibrated IMU Cartographer profile. It is optional; the
current LiDAR-only profile is the operational baseline.

### Drive and observe

Open a second Pi/SSH terminal and run:

```bash
dogzilla teleop 1
```

`teleop` opens the keyboard controller. `1` is the initial linear speed level;
number keys `1` through `9` change it while running. The `-` and `+` keys lower
or raise turning level. Read the on-screen key guide. Release keys and stop
immediately if the scan plane changes or motion becomes unstable.

Cover the room with slow loops and revisit distinctive areas. Avoid carrying
the robot, changing body height, pushing it against walls or spinning rapidly.

On a monitor, view mapping in another terminal:

```bash
dogzilla rviz
```

`rviz` opens RViz against the active ROS domain. It requires a working local
desktop display. It does not work in a display-less SSH session.

### Save and stop

Save the completed map while mapping is still active:

```bash
dogzilla save room1
```

`save` requests final Cartographer state and occupancy output. `room1` is the
map basename; use only letters, numbers, dot, underscore or dash. The command
creates `maps/room1.pbstream`, `maps/room1.pgm` and `maps/room1.yaml`.

Then stop safely:

```bash
dogzilla stop
```

`stop` sends the layered controller stop, shuts down the active services and
turns off the LiDAR through the patched driver path. It does not delete the
saved map.

Verify the bundle:

```bash
ls -lh maps/room1.pbstream maps/room1.pgm maps/room1.yaml
```

`ls` lists the three files. `-l` shows details and `-h` uses readable sizes.
All three are needed: PBStream for Cartographer localization and PGM/YAML for
Nav2 and the web map.

## View an existing map without ROS

On the Pi desktop:

```bash
xdg-open /home/pi/DOGZILLA/maps/room1.pgm
```

`xdg-open` asks the desktop to open the occupancy image in its normal image
viewer. The absolute path identifies the map. This shows the saved map but not
live robot position.

To see the robot pose on the map, run localization, navigation or Mission Mode
and use RViz or the web dashboard.

## Localization without autonomous navigation

Start a saved map for pose checking:

```bash
dogzilla localize room1 --headless
```

`localize` loads `room1` without Nav2 goal execution. `--headless` disables
RViz. The operator must set an approximate initial pose through the supported
UI before localization is accepted. This mode should not autonomously walk.

Use global Cartographer matching instead of the operator pose gate only when
the environment is distinctive enough:

```bash
dogzilla localize room1 --headless --match
```

`--match` asks Cartographer to search globally. It may be slower or ambiguous
in repeated corridors; it is an explicit alternative, not proof of location.

Stop with `dogzilla stop`.

## Mission Mode

### Start

**MOVEMENT POSSIBLE after a task is queued and localization is accepted.**

```bash
dogzilla mission start room1 --headless
```

`mission start` coordinates navigation, camera perception and the web
dashboard. `room1` selects the PBStream/PGM/YAML bundle. `--headless` disables
RViz. Startup checks ROS topics, localization components, Nav2, perception and
web health and rolls the session back if a required component fails.

Equivalent short form:

```bash
dogzilla mission room1 --headless
```

The map name immediately after `mission` is treated as `mission start room1`.

For corrected IMU localization:

```bash
dogzilla mission start room1 --headless --imu
```

`--imu` selects the calibrated IMU profile. Do not use it merely because the
sensor exists; compare it with the LiDAR-only baseline.

For global scan matching:

```bash
dogzilla mission start room1 --headless --match
```

`--match` bypasses the normal approximate-pose workflow and enables global
matching. The robot must remain supervised until the map/scan result is
credible.

### Open the dashboard

Get the current browser password and service status:

```bash
dogzilla mission password
dogzilla mission status
```

`mission password` prints the local dashboard password. `mission status`
reports navigation, perception and web state. Neither command moves the robot.
Use the Pi's address displayed by the startup/status output in a browser on the
same trusted network.

### Set initial pose

In the map panel:

1. select **Initial pose**;
2. pan/zoom to the approximate robot location;
3. click the approximate position and choose a heading, or custom heading;
4. start pose matching;
5. accept only when the corrected pose and scan overlay match the room;
6. cancel matching and reposition the estimate if the correction/warning is
   unreasonable.

The initial click is a search centre, not an exact odometry value. The matcher
can refine nearby translation and yaw, but it must not be trusted to turn a
wrong room or repeated corridor into a correct pose.

### Queue missions

The web UI supports:

- a waypoint mission with one to ten ordered stops, each timed or manually
  continued (stored internally as task kind `delivery` for compatibility);
- polygon patrol with generated coverage points;
- map-specific keepout polygons;
- autonomous walking and turning speed sliders, rounded to levels 1–9.

For manual continuation, reaching a point changes the task to `waiting`. Press
the large **Continue** button to send the next goal. A waiting task still owns
the executor and blocks another queued task. **Pause** stops progress without
discarding the task; **Continue** resumes it. **Cancel** ends it.

### Patrol-specific procedure

Before queuing a patrol:

1. open Vision in the dashboard;
2. select `patrol` and apply the mode;
3. wait for object, person and face-detection readiness;
4. confirm `action output: disabled`;
5. create/select a patrol polygon and spacing on the active map;
6. ensure no delivery/route is running, paused or waiting;
7. queue the patrol.

Patrol detections notify the web and can save a photo; they do not currently
stop the mission. Duplicate events are suppressed for the configured cooldown,
and only the latest 25 alert records/photos are retained.

### Switch maps

When no task is moving the robot:

```bash
dogzilla mission switch-map room2
```

`mission switch-map` leaves web/camera services up and restarts the
localization/navigation part with `room2`. A new initial pose and localization
verification are required. Map-specific keepouts, locations and patrol areas
are loaded for the selected map.

### Logs and stop

Follow the complete mission session:

```bash
dogzilla mission logs
```

`mission logs` follows navigation, perception and web logs together. Press
`Ctrl+C` to stop following logs; it does not stop the mission.

Stop Mission Mode:

```bash
dogzilla mission stop
```

`mission stop` cancels task execution, stops web and perception, sends the
navigation/controller stop path and shuts down navigation. It does not delete
maps or task history.

## Low-level navigation development

Use only when the web task layer is not needed:

```bash
dogzilla navigate room1 --headless
```

`navigate` starts localization plus conservative Nav2. `room1` selects the map
and `--headless` disables RViz. It can execute Nav2 goals and therefore can
move. Add `--imu` or `--match` only with the same meaning and cautions described
above.

## Camera and vision

Test the camera without legs or LiDAR:

```bash
dogzilla camera-check 10
```

`camera-check` opens only `/dev/video0` in an isolated test service. `10` is
the sample duration in seconds. It validates image rate, timestamps and
CameraInfo and then closes the camera.

Start non-actuating raw vision:

```bash
dogzilla vision raw
```

`vision` starts camera-only perception and the web view. `raw` shows the
unaltered camera mode. This service has no controller serial device.

Start camera-only patrol detection:

```bash
dogzilla vision patrol
```

`patrol` selects danger, person and face detection. It reports observations
but cannot move the robot.

Change a running vision/mission detector without reopening the camera:

```bash
dogzilla vision-mode patrol
```

`vision-mode` sends a mode request to the active perception node. `patrol` is
the requested detector mode. Mode readiness should be checked in the web UI.

`vision-control` is a separate, armed hardware mode and can move the robot. It
requires an interactive confirmation. Do not use it as a shortcut for Mission
Mode or camera testing.

## IMU calibration and validation

Stop all DOGZILLA modes and place the robot so it can be held still in each
requested pose. Then run:

```bash
dogzilla imu-calibrate
```

`imu-calibrate` temporarily owns the controller serial port and guides a
six-pose calibration. It pauses the OLED service, refuses another serial owner,
and atomically writes `calibration/imu.json` only after validation. Support the
robot on a rigid fixture if hand shake prevents stable samples.

Validate for ten seconds:

```bash
dogzilla imu-check 10
```

`imu-check` starts the corrected IMU validation path. `10` is the duration in
seconds. It checks rate, timestamp gaps/age and gravity magnitude. A mean near
9.6 m/s² is plausible for this sensor calibration and close to physical
gravity; the pass/fail limits matter more than an exact 9.80665 reading.

## Stand, rest and LiDAR power

**MOVEMENT POSSIBLE.** With clear legs and valid battery telemetry:

```bash
dogzilla stand
```

`stand` loads all motors and invokes the controller's public animated stand-up
action with a guard delay. It is not safe while the robot is constrained or
held by the legs.

Do not use `dogzilla rest` as a power-button substitute. It is intentionally
disabled because the public lie-down action and immediate torque release have
not been proven identical to the private firmware power-button/low-battery
sequence. `rest-capture` is observation-only and captured profiles are marked
non-replayable.

Turn off a spinning LiDAR when no ROS mode should own it:

```bash
dogzilla lidar-off
```

`lidar-off` sends the MS200 motor-off path without starting the ROS stack. It
does not turn off the leg servos. Do not run it while active mapping/navigation
owns `/dev/ttyAMA1`; use `dogzilla stop` instead.

## Controller-only drive

**MOVEMENT POSSIBLE.** For slow controller tests without LiDAR:

```bash
dogzilla drive
dogzilla teleop 1
```

`drive` starts only the guarded controller bridge, so the LiDAR remains off.
`teleop 1` starts at the slowest level. Finish with `dogzilla stop`.

## Logs and diagnostics

Follow logs for the active non-mission mode:

```bash
dogzilla logs
```

`logs` follows the current service output. `Ctrl+C` exits the viewer without
stopping the service.

Open a correctly configured diagnostic shell inside the active image:

```bash
dogzilla shell
```

`shell` opens Bash with ROS Humble, the built workspace and `ROS_DOMAIN_ID=12`
already configured. Prefer it to a raw `docker exec`. Exit with `exit` or
`Ctrl+D`; exiting the shell does not stop the robot service.

Inside that shell, useful read-only checks include:

```bash
ros2 topic hz /scan
ros2 topic echo /battery_state --once
ros2 run tf2_ros tf2_echo map base_link
```

`ros2 topic hz /scan` measures LiDAR publication frequency until `Ctrl+C`.
`ros2 topic echo` displays messages; `/battery_state` is the topic and `--once`
exits after one message. `ros2 run` starts an installed executable;
`tf2_ros` is its package, `tf2_echo` is the executable, and `map base_link` are
the target and source frames whose transform is printed until `Ctrl+C`.

Check host device owners when startup reports a conflict:

```bash
sudo fuser -v /dev/ttyAMA0 /dev/ttyAMA1 /dev/video0
```

`sudo` requests host privileges. `fuser` lists processes using files; `-v`
adds user/PID/access details. The three explicit device paths are the
controller, LiDAR and camera. Inspect the PID before stopping anything; do not
kill an unknown process blindly.

## Failure recovery

If startup fails:

1. keep the robot stationary;
2. read the exact terminal error;
3. run `dogzilla mission status` for Mission Mode, otherwise
   `dogzilla status`;
4. run the matching `logs` command;
5. check device owners;
6. use the matching safe stop command;
7. correct the reported dependency/configuration problem;
8. restart once and recheck readiness before queuing a goal.

Do not repeatedly restart around a serial conflict. Do not treat a Nav2 result
status alone as the cause; inspect localization, TF/scan timing, controller
logs and the task error from the same session.

If the terminal is lost while the robot mode remains active, reconnect to the
Pi and run the status command, then the safe stop command. Docker services are
not tied to the lifetime of an SSH terminal.

## Safe shutdown

Stop the active mode first:

```bash
dogzilla stop
```

For Mission Mode use `dogzilla mission stop`. Confirm the LiDAR is no longer
spinning and the status output has no active robot service before turning off
the Pi or robot.

The optional OS shutdown guard is installed once with:

```bash
dogzilla shutdown-install
```

`shutdown-install` installs the host shutdown integration that asks the robot
stack to stop during OS shutdown. It changes host service configuration and is
not a replacement for supervised shutdown.

## Backup checklist

Git backs up source and documentation, not Docker images or all runtime data.
Before major changes, separately copy:

- `maps/`;
- `calibration/`;
- `web/data/tasks.sqlite3` when task history/locations matter;
- required model weights and their provenance;
- selected `logs/sessions/` reports;
- the output of `docker image inspect dogzilla-mapping:humble` if exact image
  identity matters.

Never put passwords, `.env`, private images or raw private datasets into the
Git repository.
