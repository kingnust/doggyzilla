# DOGZILLA S2 development roadmap

This roadmap orders future work by safety and dependency, not by novelty. The
robot already maps, localizes, navigates, runs web tasks and reports camera
observations. The next developer should improve repeatability and evidence
before adding more autonomous behavior.

Status terms used here:

- **operational**: deployed and exercised on this robot;
- **implemented**: source/tests exist, but more physical acceptance is needed;
- **prepared**: reversible source candidate exists and is inactive;
- **experimental**: isolated from operational control;
- **blocked**: required evidence or hardware is missing.

## Safety rules for every milestone

Every change that can affect motion follows the same gates:

1. write a measurable acceptance criterion before changing parameters;
2. preserve a known-good source/image and robot-specific data backup;
3. run pure tests and configuration contracts;
4. build once from clean pinned inputs;
5. check stationary topics, TF, battery and command outputs;
6. trial at speed level 1 on a clear floor with physical power accessible;
7. change one variable group at a time;
8. save the complete session and compare it with the baseline;
9. promote, revise or revert based on evidence;
10. document the deployed image ID and configuration.

Never weaken the command watchdog, low-battery protection, serial ownership,
goal/map validation or physical supervision to make an acceptance test pass.

## P0: establish a dependable baseline

### P0.1 Navigation stability trial

Status: **prepared, inactive**.

Problem: observed missions showed Cartographer work-queue growth, reduced scan
processing and unnecessary replanning. Steering oscillation must not be tuned
blindly while localization timing is degraded.

Prepared source lives in `plans/navigation-stability/`. It proposes:

- 0.5 Hz instead of 1 Hz global replanning;
- fewer Cartographer background threads and constraints;
- less frequent pose-graph optimization and state/submap publication;
- two retained localization submaps;
- 0.50 second Nav2 transform tolerance.

It intentionally leaves scan rate, walking speed, obstacle layers, footprint,
collision handling and emergency stop unchanged.

Acceptance evidence:

- `/scan` remains near the MS200's expected 10 Hz without growing age;
- Cartographer work queue stays bounded over a representative route;
- `map -> odom -> base_link` remains continuous;
- controller deadline warnings and Nav2 transform failures do not increase;
- cross-track error and left/right reversal count improve over the baseline;
- obstacle marking and stopping remain intact;
- apply and revert produce reproducible source states.

Do not combine the first trial with IMU, speed, footprint or camera changes.

### P0.2 Navigation recorder and result classification

Status: **implemented; expand evidence library**.

The recorder should continue to capture only tuning evidence:

- planned path and replans;
- raw, smoothed, guarded and final velocity commands;
- scan/TF/odometry timing;
- map pose, odom pose and cross-track error;
- front/side LiDAR clearances;
- task/Nav2 action result and cancellation source;
- active Nav2/Cartographer/speed parameters.

Next work is not “record more topics.” Add a small report that labels each
trial as pose loss, planner failure, controller oscillation, obstacle stop,
stall inference, operator cancel or successful arrival. Store the test layout,
initial pose and image/config revision with every report.

Acceptance evidence:

- the same log produces the same summary;
- report timestamps align with ROS time;
- no recorder output can publish velocity, cancel a goal or alter parameters;
- reports remain bounded and cannot fill the Pi filesystem.

### P0.3 Physical collision/stall protection

Status: **software warning implemented; collision certainty blocked by
hardware**.

The robot has no wheel encoders, bumper or leg contact sensor. Comparing a
command with scan-matched motion can warn that it is stalled, but it cannot
prove contact and can be wrong during scan loss, carpet slip or body sway.

Recommended design:

```text
commanded motion + scan odometry + LiDAR clearance + localization health
                              |
                              v
                warning / confidence state
                              |
             operator-visible alert and bounded stop policy
```

For a dependable contact stop, add and independently power-test a bumper,
foot/contact switch, motor-current/servo-load signal or other physical sensor.
Do not create a sensitive “cancel on small odometry difference” rule: false
positives can leave the robot stranded and false negatives can permit pushing.

Acceptance evidence:

- no warning while standing, turning normally or walking over representative
  floor transitions;
- repeatable warning for a safely restrained forward-motion test;
- sensor/timing failure is distinguishable from contact;
- stopping is latched, visible and requires deliberate recovery;
- the policy never bypasses `safe_base`.

### P0.4 Automatic patrol perception handshake

Status: **manual workflow operational; automatic handshake not implemented**.

Currently a patrol remains queued until the operator sets Vision mode to
`patrol`. Implement a gateway state machine, not a browser-only convenience:

1. a patrol is selected;
2. request `patrol` through `/vision/mode_command`;
3. wait for matching `/vision/status` with a bounded timeout;
4. verify full detector coverage, confirmation thresholds and
   `action_output: disabled`;
5. dispatch only while that exact readiness remains healthy;
6. fail visibly if perception changes or becomes stale.

Delivery and ordinary route tasks may default to raw vision, but must not
silently arm a camera action mode. Confirmed dangers continue to notify and
save a photo; they do not stop a mission unless a separately reviewed policy
is added.

Acceptance evidence:

- patrol starts from raw mode without manual detector selection;
- timeout/missing model leaves the task queued or failed and stationary;
- stale status cannot satisfy readiness;
- changing out of patrol during execution causes an explicit safe task state;
- no camera event directly commands motion.

### P0.5 Backup and restore drill

Status: **documented, not yet proven as a full restore**.

Git is only the source backup. Create a dated, checksummed robot backup for:

- map bundles;
- calibration files;
- mission database and selected alert images;
- model weights plus source/version/license metadata;
- deployed Git revision, dirty diff and Docker image ID;
- selected acceptance logs.

Test restore into a temporary directory and validate SQLite and map bundles.
Do not include passwords, environment secrets or private raw datasets in Git.

Acceptance evidence:

- a second operator can restore a map/calibration/database without guessing;
- checksums detect truncation;
- PBStream/PGM/YAML basenames remain consistent;
- restore cannot overwrite live data without an explicit backup and choice.

## P1: calibrate the physical model

### P1.1 Camera intrinsics

Status: **temporary calibration exists; real checkerboard calibration still
required**.

OpenCV is the image-processing library and calibration implementation. A
camera model is the measured result OpenCV needs to remove lens distortion and
project image rays. They work together; installing OpenCV cannot replace
measuring this DCX-BAT53G2V2 camera/lens at the deployed 640 x 480 mode.

Use a rigid checkerboard with measured square size, fill the image area and
angles, commit only a low-error calibration and repeat after focus, lens or
resolution changes.

Acceptance evidence:

- calibration is for `dogzilla_mono`, 640 x 480;
- reprojection error is recorded and rectified straight lines look straight;
- CameraInfo timestamps/frame IDs match the images;
- the result replaces `camera.temporary.yaml` only after review.

### P1.2 Camera extrinsics

Status: **framework present; physical values must be measured/verified**.

Measure `base_link -> camera_link` XYZ and RPY from the base-link definition in
`calibration/BASE_LINK.md`. Do not optimize guessed extrinsics until they “look
right”: that can absorb time offset or intrinsic errors into a false transform.

Acceptance evidence:

- mounting reference and uncertainty are written down;
- TF has one publisher and no cycle;
- projected stable features do not shift incorrectly during controlled turns;
- visual shadow remains non-controlling.

### P1.3 URDF physical measurements

Status: **experimental visualization framework**.

The current URDF provides body, LiDAR, IMU and camera frame structure. Before
using legs for collision, kinematics or control, measure:

- body/link dimensions and masses;
- hip positions and axis directions;
- upper/lower leg lengths;
- servo zero offsets, signs and joint limits;
- motor-ID-to-corner/joint mapping;
- center of mass and contact geometry.

Acceptance evidence:

- a measurement table includes tool, method and uncertainty;
- live joint states move the correct rendered joint in the correct direction;
- all four legs match physical poses through their safe range;
- a reviewer signs off before URDF is used for control or collision stopping.

### P1.4 IMU comparison

Status: **calibration and correction operational; fusion optional**.

The calibrated sensor passed rate/gravity/timestamp checks, but IMU-enabled
navigation has not consistently outperformed LiDAR-only localization. Keep the
two profiles. Record matched routes under the same map, posture, speed and
initial pose.

Acceptance evidence:

- six-pose calibration passes without hand-motion contamination;
- stationary gyro bias and gravity direction remain within limits;
- no timestamp jumps or stale samples;
- heading/cross-track error improves over LiDAR-only in repeated trials;
- if it does not improve, LiDAR-only remains the default.

### P1.5 Initial-pose validation dataset

Status: **wide nearby search implemented; thresholds need multi-room evidence**.

Record correct/incorrect candidate poses in:

- open rooms;
- repeated corridors;
- furniture changes;
- partial occlusion;
- several yaw errors and position offsets up to the supported radius.

Evaluate endpoint match, mapped-ray contradiction, coverage, correction size
and repeat stability. A 30% alignment threshold alone is not proof of location.
The matcher should refine an approximate area and warn on excessive correction,
not teleport between ambiguous rooms.

Acceptance evidence:

- documented false-accept and false-reject rates;
- furniture does not force unreasonable rejection;
- a clearly wrong location is rejected in every selected test;
- the UI shows searched area, best correction and confidence limitations;
- cancellation immediately stops matching without stopping Mission Mode.

## P2: mature autonomous missions

### P2.1 Task state and manual continuation

Status: **implemented; physical workflow acceptance needed**.

Exercise deliveries and up-to-ten-point routes with timed waits and manual
continuation. Validate pause, continue, cancel, web reconnect and process
restart at every step. A waiting task must remain the only active task; restart
must never silently resume motion.

Acceptance evidence:

- every transition is persisted and visible;
- repeated Continue clicks cannot skip a point;
- cancel sends zero movement and ends the action;
- browser disconnect does not create duplicate goals;
- invalid/stale battery telemetry is visible but does not masquerade as 0%; a
  confirmed low valid battery still prevents unsafe movement.

### P2.2 Dynamic obstacle behavior

Status: **live LiDAR costmaps operational; recovery deliberately limited**.

Test people, chairs and movable furniture at LiDAR height. The local costmap is
temporary and rebuilt each mission; static map and per-map keepouts persist.
Tune observation persistence, inflation and progress checking only with log
evidence. Keep automatic spin and backup disabled until separately accepted.

Objects above or below the 2-D scan plane are not dependable LiDAR obstacles.
The monocular camera can classify an item but cannot provide safety-grade depth
from a single frame.

### P2.3 Map switching

Status: **implemented; transactional acceptance needed**.

`mission switch-map` preserves web and camera while replacing localization and
navigation. Validate rollback when a map bundle is incomplete, require a new
initial pose, clear old pose/costmaps, and load only the selected map's
locations, patrols and keepouts.

Acceptance evidence:

- no old-map goal can dispatch after switch;
- failed switch returns to a clearly named safe state;
- current map/revision is visible in API and UI;
- active/waiting tasks block or deliberately resolve before switching.

### P2.4 Alert semantics and retention

Status: **implemented baseline**.

Keep duplicate suppression based on label, spatial overlap and time window;
retain at most 25 alert photos. Add operator acknowledgement/export only if it
has a real use case. Face detection must remain presence detection, not identity
recognition, unless privacy, consent and retention are redesigned.

Pretrained open-vocabulary labels are best-effort observations. Small screws,
shards, wires and transparent glass are difficult at distance and must never be
advertised as guaranteed floor safety.

## P3: experiments isolated from control

### P3.1 Monocular RTAB-Map shadow

Status: **experimental and calibration-gated**.

RTAB shadow may record visual features, database statistics and loop closures
beside Cartographer. It must keep:

- no controller or LiDAR serial device;
- no `/cmd_vel` publication;
- no operational TF authority;
- its own namespace/database;
- explicit camera intrinsics/extrinsics gates.

Do not replace the LiDAR localization stack unless repeated route tests show a
clear benefit and all control consumers are migrated through a reviewed design.
Monocular imagery does not magically produce metric depth; scale presently
comes from external motion/scan information.

### P3.2 Semantic map annotations

Status: **future**.

If furniture or engineering objects need to appear on the map, store semantic
annotations separately from the immutable occupancy map. Each annotation
should include map name/revision, class, pose, uncertainty, source, timestamp
and expiry. Movable furniture belongs in live costmaps or expiring annotations,
not permanently painted into PBStream/PGM as a wall.

## Firmware work boundary

The microcontroller firmware source is not present. The repository controls it
through the installed DOGZILLALib serial protocol. Future public-protocol work
must extend the single `safe_base` owner and document register/scaling evidence.

Blocked until vendor/source/analyzer evidence exists:

- firmware-identical power-button or low-battery rest trajectory;
- safe timed torque release after that trajectory;
- internal servo loops, current limits and thermal behavior;
- private battery thresholds/state machine;
- flash/upgrade implementation.

Do not replay captured serial packets to discover these behaviors while the
robot is loaded. A profile capture is evidence, not execution approval.

## Work explicitly not recommended

- a second `/dev/ttyAMA0` reader for IMU, battery or joints;
- immediate `unload_allmotor()` after public action 1;
- enabling movement from ordinary camera detections;
- treating monocular class boxes as measured obstacle distance;
- tuning speed to hide localization/controller instability;
- painting movable furniture into the static occupancy map;
- enabling URDF leg control from provisional geometry;
- automatic spin/backup recovery without physical trials;
- weakening localization acceptance because one furnished room fails;
- committing Docker layers, credentials, raw private datasets or generated
  runtime databases to Git.

## Suggested next three development cycles

### Cycle 1: measurement

1. Run the inactive navigation-stability candidate as an A/B trial.
2. Complete the real camera checkerboard and mount calibration.
3. Build a multi-room initial-pose validation dataset.
4. Perform and document a backup/restore drill.

### Cycle 2: reliability

1. Promote or revert the navigation candidate from evidence.
2. Implement the automatic patrol perception handshake.
3. Complete task pause/wait/reconnect physical acceptance.
4. Design a real physical contact/stall sensor interface.

### Cycle 3: capability

1. Validate transactional map switching.
2. Add semantic annotations without modifying the occupancy authority.
3. Evaluate calibrated RTAB shadow route/loop-closure reports.
4. Decide whether the IMU profile earns promotion or remains optional.

Each cycle should end with updated documentation, test results, deployed image
identity, physical acceptance notes and an explicit rollback path.
