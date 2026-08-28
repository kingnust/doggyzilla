# DOGZILLA S2 developer handoff

This is the starting point for the next developer. It describes what is
operational, where each responsibility lives, how changes reach the robot and
which boundaries must not be crossed.

## Read these documents in order

1. This handoff.
2. [FRAMEWORK.md](FRAMEWORK.md) for component boundaries and current status.
3. [PIPELINES.md](PIPELINES.md) for end-to-end data and command flows.
4. [FIRMWARE_AND_SERIAL.md](FIRMWARE_AND_SERIAL.md) before touching movement,
   posture, battery, actions, IMU or servos.
5. [ROS_INTERFACES.md](ROS_INTERFACES.md) before adding a node or topic.
6. [OPERATIONS_RUNBOOK.md](OPERATIONS_RUNBOOK.md) before deploying or running
   a physical test.
7. [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md) before selecting the next
   feature.

Specialized references:

- [COMPUTER_VISION.md](COMPUTER_VISION.md)
- [URDF_RTABMAP_MONO.md](URDF_RTABMAP_MONO.md)
- [calibration/README.md](../calibration/README.md)
- [web/README.md](../web/README.md)
- [deploy/README.md](../deploy/README.md)

## System summary

The robot is a Yahboom DOGZILLA S2 controlled by a Raspberry Pi. This
repository adds a reproducible ROS 2 Humble stack around the vendor controller:

```text
MS200 LiDAR + controller telemetry + mono camera
                  |
                  v
        ROS perception and Cartographer
                  |
                  v
      map, localization and scan odometry
                  |
                  v
 web missions -> Nav2 -> command filters -> safe_base -> controller
```

The operational navigation system is 2-D LiDAR based. Cartographer provides
mapping and scan-matched localization. Nav2 provides path planning, live
costmaps and path following. The camera provides observation and patrol alerts;
it is not the operational localization authority. Monocular RTAB-Map remains a
separate experimental shadow pipeline.

DOGZILLA has no wheel encoders and no contact sensor. `/odom` is calculated
from Cartographer's scan-matched TF. Stall detection is therefore an inference
from commanded motion, LiDAR and scan odometry, not proof that the robot hit an
object.

## Non-negotiable invariants

1. **One controller serial owner.** Only one process may open
   `/dev/ttyAMA0`. Extend `safe_base`; never add a competing controller node.
2. **One LiDAR owner.** Mapping or navigation owns `/dev/ttyAMA1`.
3. **One camera owner.** A camera service or the Yahboom app may own
   `/dev/video0`, never both.
4. **Final movement passes through `safe_base`.** Browser, camera, mission and
   diagnostic code must not open serial or command motors directly.
5. **Stop is layered.** Nav2 cancellation, zero velocity, Twist Mux, the
   steering guard, `safe_base` clamps/watchdog and controller stop each cover a
   different failure.
6. **Localization precedes autonomy.** A task cannot dispatch until the live
   scan agrees with the selected map and pose.
7. **LiDAR geometry remains fixed.** Body height/look controls are disabled
   during mapping and localization.
8. **Low battery wins.** Do not override the controller's 25% protection.
9. **No guessed rest trajectory.** `rest` remains disabled until a real,
   independently reviewed firmware trajectory and torque sequence are proven.
10. **No automatic Git or registry push.** Build and deploy are local actions;
    commit/push require a separate explicit operator decision.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `README.md` | Project entry point and common workflow |
| `docs/` | Architecture, interfaces, operations and specialized design documents |
| `deploy/dogzilla-map` | Main host operator command; use this instead of raw Compose for normal work |
| `deploy/dogzilla-mission` | Starts/stops navigation, mission perception and web as one managed session |
| `deploy/dogzilla-web` | Internal web-container helper |
| `deploy/compose.yaml` | Container boundaries, devices, volumes, commands and health checks |
| `deploy/Dockerfile` | Reproducible ARM64 runtime image |
| `deploy/ros-packages.lock` | Exact apt packages installed in the image |
| `deploy/patches/` | Reviewed upstream compatibility/safety patches |
| `ros2/dogzilla_slam/launch/` | Combined ROS launch descriptions |
| `ros2/dogzilla_slam/config/` | Cartographer, Nav2, camera, Twist Mux and RTAB configuration |
| `ros2/dogzilla_slam/dogzilla_slam/` | ROS nodes and pure policy/state modules |
| `ros2/dogzilla_slam/behavior_trees/` | Fail-closed Nav2 behavior trees |
| `ros2/dogzilla_slam/urdf/` | Provisional visualization model |
| `ros2/dogzilla_slam/test/` | Package unit and deployment-contract tests |
| `deploy/test/` | Host mission coordinator tests |
| `app_dogzilla/` | Original Yahboom mobile app, joystick, camera and OLED code |
| `Samples/` | Original Yahboom notebooks and examples; not the safe runtime |
| `maps/` | Persistent map bundles |
| `calibration/` | Robot-specific IMU and camera measurements |
| `models/` | Deployed detection models and labels |
| `training/` | Offline model export/training helpers |
| `profiles/` | Controller captures; captures are not replay approval |
| `web/data/` | Runtime SQLite task database and bounded alert images |
| `logs/` | Timestamped runtime sessions and diagnostic reports |
| `plans/navigation-stability/` | Prepared, reversible navigation candidate; inactive unless explicitly applied |

## Runtime modes

| Operator mode | Containers | Moves robot? | Purpose |
| --- | --- | --- | --- |
| `start` | mapping | Teleop only | Build a Cartographer map |
| `drive` | drive | Teleop | Controller-only testing; LiDAR off |
| `localize` | navigation without Nav2 | No autonomous movement | Check pose against a saved map |
| `navigate` | navigation with Nav2 | Goal/teleop capable | Low-level navigation development |
| `mission` | navigation + perception + web | Web tasks | Normal delivery, route and patrol workflow |
| `vision` | vision + web | No | Detection-only camera lessons/lab |
| `vision-control` | vision_control | Yes, explicitly armed | Narrow allowlisted Yahboom camera behaviors |
| `shadow` | mapping + visual_shadow | Teleop only | Experimental Cartographer plus RTAB recording |

Only the operator wrapper should combine these modes. Raw launch files do not
enforce all host service, device, log and cleanup checks.

## Source versus deployed code

Editing files under `ros2/dogzilla_slam/` does not change a running container.
The deployment path is:

```text
edit source -> run tests -> build image -> stop old mode -> start mode -> verify
```

The image copies `ros2/dogzilla_slam` into
`/root/yahboomcar_ws/src/dogzilla_slam` and runs `colcon build`. There is no
source bind mount for the package. A stale image can therefore look healthy
while still running older code.

Before work, always inspect:

```bash
cd /home/pi/DOGZILLA
git status --short
```

`cd` changes the terminal's working directory to the repository. `git status`
reports tracked and untracked changes. `--short` uses the compact two-column
format. Existing changes may belong to another developer; do not reset or
overwrite them.

## Development and test workflow

The bare Pi host does not contain the complete pytest and ROS Python test
environment. After a local image exists, run the full source suite in a
disposable, network-disabled container with the repository read-only and no
robot devices:

```bash
sudo docker run --rm --network none \
  --volume /home/pi/DOGZILLA:/workspace:ro \
  --workdir /workspace dogzilla-mapping:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && source /root/yahboomcar_ws/install/setup.bash && export PYTHONPATH=/workspace/ros2/dogzilla_slam:$PYTHONPATH && python3 -m pytest -q -p no:cacheprovider ros2/dogzilla_slam/test deploy/test'
```

`sudo` grants access to the Docker daemon. `docker run` creates a one-off
container. `--rm` removes it after the test, `--network none` prevents network
access, `--volume ...:/workspace:ro` mounts the current source read-only and
`--workdir /workspace` selects its working directory. The next value is the
local image. `bash -lc` runs the quoted configured shell commands. The two
`source` commands load ROS Humble and the built workspace. `PYTHONPATH` places
the mounted current package ahead of the image's older installed copy.
`python3 -m pytest` runs the test runner, `-q` shortens output and
`-p no:cacheprovider` prevents writes to the read-only repository. The final
paths select the package and deployment tests. No `--device` flag is supplied,
so this test cannot open the robot controller, LiDAR or camera.

The deployment coordinator's dependency-light subset can also run on the host:

```bash
python3 -m unittest discover -s deploy/test -p 'test_*.py'
```

`python3 -m unittest discover` uses the standard-library test discovery. `-s`
sets the starting directory and `-p` sets the filename pattern. This does not
replace the complete ROS-container suite.

Check formatting damage and unresolved merge text before a build:

```bash
git diff --check
grep -RInE '^(<<<<<<<|=======|>>>>>>>)' ros2 deploy docs
```

`git diff --check` detects whitespace errors in tracked changes. `grep` searches
recursively; `-R` recurses, `-I` ignores binary files, `-n` prints line numbers,
and `-E` enables the conflict-marker expression.

Then verify the host and build:

```bash
dogzilla doctor
dogzilla build
```

`doctor` checks Docker, Compose, serial devices, the pinned base image, display
and free storage. `build` creates the local `dogzilla-mapping:humble` image; it
does not deploy a mode, push Git, or upload an image.

Use the shortest physical test that covers the change. A web validation change
normally needs API/unit tests and a stationary mission start before motion. A
controller change needs fake-controller tests, stationary telemetry, then a
slow open-floor trial.

## Configuration sources of truth

| Concern | Source |
| --- | --- |
| Controller speed levels | `dogzilla_slam/speed_control.py` |
| Controller clamps/watchdog/battery | `dogzilla_slam/safe_base.py` |
| Mapping behavior | `config/dogzilla_2d.lua` or `dogzilla_2d_imu.lua` |
| Localization behavior | `config/dogzilla_localization.lua` or IMU variant |
| Nav2 controller/costmaps/footprint | `config/nav2_test1.yaml` |
| Nav2 recovery policy | `behavior_trees/*.xml` |
| Velocity arbitration | `config/twist_mux.yaml` |
| Camera profile | `config/mono_camera.yaml` |
| Vision detection policy | `vision_core.py`, `object_detector.py`, `vision_node.py` |
| Firmware action allowlist | `vision_action_policy.py` |
| Web validation/state/database | `web_core.py` |
| ROS and task orchestration | `web_gateway.py` |
| HTTP routes/authentication | `web_http.py` |
| Container/device boundary | `deploy/compose.yaml` |
| Host lifecycle and safety checks | `deploy/dogzilla-map`, `deploy/dogzilla-mission` |

Avoid duplicating a number in another layer unless the layer has a genuinely
different physical meaning. When duplication is required, add a contract test
that detects divergence.

## Mapping and localization model

Each operational map is a three-file bundle:

- `.pbstream`: Cartographer frozen trajectory, submaps and pose-graph state;
- `.pgm`: occupancy pixels;
- `.yaml`: Nav2 resolution, origin and occupancy metadata.

Mapping generates all three. Localization loads PBStream as frozen state and
starts a live trajectory relative to it. The map server publishes PGM/YAML for
Nav2 and the web.

The normal mission start waits for an operator initial pose. The web gateway
uses that pose as a search centre, evaluates nearby position/angle candidates,
publishes the best accepted `/initialpose`, then checks stable TF, correction
size and live scan/map agreement. `--match` is an explicit alternative that
starts Cartographer's global matching without the operator pose gate.

Furniture can reduce endpoint agreement or create ray contradictions. The
validator is evidence of geometric consistency, not a proof that every saved
wall is currently visible. Threshold changes require recorded comparison data,
not one convenient room trial.

## Navigation model

The global planner uses the static map, live LiDAR obstacle layer, inflation and
the active map's keepout mask. The regulated pure-pursuit controller follows
the path using a local costmap. The velocity smoother and steering guard reduce
abrupt command changes before Twist Mux and `safe_base`.

The default measured footprint is approximately 260 x 145 mm with 30 mm
padding. Automatic spin and backup recovery are disabled because they are high
risk on the quadruped and were not physically accepted.

The tuning recorder captures only data needed to diagnose navigation: planned
paths, raw/smoothed/final commands, scan odometry, map/odom poses, cross-track
error, LiDAR sectors, timing, diagnostics, result and parameter changes. Use
this evidence before changing controller gains.

## Web task model

The SQLite store persists deliveries, routes, patrols, named locations,
map-specific keepouts, hazard observations and vision alerts. On gateway
restart, any task that was active is marked failed; movement never silently
resumes.

Task flow:

```text
queued -> running -------------------------------------> completed
             |  |                                           
             |  +-> pausing -> paused ------------------+   
             |  +------------> waiting -----------------+-> running
             +-> cancelling ---------------------------> cancelled
             `------------------------------------------> failed
```

Only one task is active. `waiting` for manual continuation is still active and
blocks the next queued task. Pause/continue and manual checkpoints apply to the
one-to-ten-stop `delivery`/waypoint-mission type; the generic route and patrol
types do not inherit those controls automatically.

Patrol adds a perception gate. It will remain queued unless Vision reports
mode `patrol`, complete model coverage, multi-frame confirmation settings and
`action_output: disabled`. At present the operator must select Patrol vision;
queuing a patrol does not automatically change the mode.

## Persistent and generated data

Git intentionally ignores credentials, runtime logs, task databases, captured
profiles and local build products. A Git backup alone is not a complete robot
backup.

Back up these robot-specific assets separately:

- `maps/` map bundles;
- `calibration/imu.json`;
- real `calibration/camera.yaml` and measured camera extrinsics;
- `web/data/tasks.sqlite3` if mission history/locations matter;
- selected log sessions and validation reports;
- any reviewed model weights not reproducible from documented downloads;
- controller firmware version and physical measurements.

Never add API keys, `.env`, browser passwords or raw private datasets to Git.

## Logging and diagnosis

Every mode creates a timestamped directory under `logs/sessions/` and updates
`logs/latest`. Mission Mode keeps navigation, perception and web in the same
logical session.

Begin diagnosis with:

1. operator status;
2. the exact task state/error;
3. localization state and live scan rate;
4. Nav2 action availability;
5. controller/battery state;
6. container logs from the same session;
7. tuning report for motion-quality problems.

Do not tune Nav2 when the underlying pose or scan timestamps are unstable. Do
not tune Cartographer to hide a wrong initial pose. Do not add more speed to
compensate for a controller oscillation.

## Safe change ownership

| Change | Primary files | Minimum verification |
| --- | --- | --- |
| Browser layout | `web_static/*` | dashboard tests, desktop/mobile browser check |
| API validation/task state | `web_core.py`, `web_gateway.py`, `web_http.py` | unit/API tests, stationary mission state check |
| Patrol/object classes | vision modules, model labels | model validation, camera-only acceptance, patrol gate test |
| Nav2 tuning | `nav2_test1.yaml`, behavior trees, steering guard | contract tests, stationary timing, recorded open-floor trial |
| Cartographer tuning | localization/mapping Lua | scan/TF timing, known-map replay or controlled trial |
| IMU correction | calibration and IMU modules | six-pose calibration and `imu-check` |
| Controller behavior | `safe_base.py`, policy modules | firmware doc review, fake controller, battery gate, physical safety trial |
| Docker dependencies | Dockerfile and package lock | clean image build, package discovery, health checks |
| URDF/camera transform | Xacro and measured calibration | physical measurement, TF validation; no control use |

## Current limitations

- Embedded controller firmware source is unavailable.
- Firmware-identical rest/torque release is not implemented.
- Operational odometry depends on LiDAR scan matching.
- IMU fusion is optional and has not improved every route.
- Monocular vision does not provide dependable depth by itself.
- RTAB shadow is not operational navigation authority.
- URDF leg geometry and joint conventions remain provisional.
- No physical bumper/contact sensor confirms a collision.
- Patrol vision mode does not yet switch automatically with a patrol task.
- Navigation stability candidate settings are prepared but not automatically
  active; inspect `plans/navigation-stability/README.md` before using them.

## Handoff checklist

Before accepting responsibility for the robot:

- read the seven documents listed at the top;
- confirm the physical E-stop/power procedure with the owner;
- run `dogzilla doctor`;
- record the installed `DOGZILLALib` and controller firmware versions;
- verify the current Git branch and dirty files;
- confirm which Docker image ID is deployed;
- make a robot-specific backup of maps and calibration;
- perform a stationary `status` check;
- run teleop level 1 on a clear floor before autonomous testing;
- validate initial pose and one short waypoint before patrol/delivery trials;
- do not enable rest replay, URDF control or aggressive recoveries without a
  new physical acceptance plan.
