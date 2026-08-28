# DOGZILLA controller firmware and serial interface

This document explains the controller boundary used by this repository. It is
intended for developers who need to extend DOGZILLA without creating a second
serial owner or assuming undocumented firmware behavior.

## What “firmware” means on this robot

DOGZILLA has four distinct software layers:

1. **Embedded controller firmware** runs on the motor controller. It performs
   gait generation, inverse kinematics, servo control, action sequences,
   battery protection and the physical power-button behavior.
2. **Yahboom `DOGZILLALib`** runs on the Raspberry Pi. It converts Python
   method calls into controller serial packets.
3. **The original Yahboom applications** provide the mobile-app server,
   joystick, camera lessons and OLED display.
4. **This repository's ROS 2 layer** provides single-owner serial access,
   command limits, telemetry, mapping, navigation, vision and web missions.

Only layers 2 through 4 are visible as source on this Pi. The embedded
controller source and action trajectories are not in this repository. The
controller can therefore be documented only from its public host protocol,
readbacks and observed physical behavior.

## Versions present on this Pi

| Component | Version or status |
| --- | --- |
| Installed host library | `DOGZILLALib` 3.1.9, modified 2023-05-23 |
| Historical repository archive | `DOGZILLALib.zip` contains 2.0.7 |
| Original app header | Reports V2.0.7 |
| Embedded firmware binary/source | Not stored in this repository |
| Embedded version | Queryable with `read_version()` when the serial port is free |

The Docker image inherits the host library from the pinned Yahboom base image.
Do not replace it with the older archive merely because the archive is inside
the repository: 3.1.9 adds `read_imu_raw()` and has different timing behavior.

To inspect the installed host-library version without opening the controller:

```bash
python3 -c "import DOGZILLALib.DOGZILLALib as d; print(d.__version__)"
```

`python3` runs the host Python interpreter. `-c` executes the quoted one-line
program. The program imports the implementation module and prints its declared
version; it does not instantiate `DOGZILLA` and therefore does not open
`/dev/ttyAMA0`.

## Physical connection and ownership

The host library opens:

```text
/dev/ttyAMA0, 115200 baud, 8 data bits, no parity, one stop bit
```

Only one process may own this port. Read operations flush the input buffer
before sending a request, so two readers do not merely share bandwidth: each
can erase the other process's reply and misassociate subsequent packets.

During ROS modes, `dogzilla_safe_base` is the only permitted owner. The
operator wrapper:

- gracefully stops the original `app_dogzilla.py` process;
- pauses `yahboom_oled.service`, which otherwise opens its own controller;
- verifies that the serial devices are free;
- starts one Compose service with `/dev/ttyAMA0` passed through;
- restores the OLED service after the ROS owner stops.

Do not run the original notebooks, OLED script, joystick script, a standalone
IMU reader, or an interactive `DOGZILLALib` shell while mapping, navigation,
drive or armed vision control is active.

## Serial frame format

The host library uses a small binary register protocol.

### Write packet

```text
55 00 LEN 01 ADDR DATA... CHECK 00 AA
```

`LEN` is the full packet length, equal to the payload length plus 8. `01` is
the write type. `ADDR` is the register address. `CHECK` is:

```text
255 - ((LEN + TYPE + ADDR + sum(DATA)) modulo 256)
```

### Read request

```text
55 00 09 02 ADDR READ_LEN CHECK 00 AA
```

`02` is the read type and `READ_LEN` is the requested payload size. The reply
uses the same `55 00 ... 00 AA` envelope. The library validates the length and
checksum before exposing values.

This protocol has no transaction identifier and the sensor packet has no
hardware timestamp. That is another reason one process must serialize every
request and reply.

## Public register map

The following addresses come from installed `DOGZILLALib` 3.1.9. A range means
one address per axis, leg coordinate or motor.

| Address | Library name | Purpose |
| --- | --- | --- |
| `0x01` | `BATTERY` | Read integer battery percentage |
| `0x03` | `PERFORM` | Enable or disable repeating performance mode |
| `0x04` | `CALIBRATION` | Enter or leave software calibration |
| `0x05` | `UPGRADE` | Firmware upgrade register; not used here |
| `0x06` | `MOVE_TEST` | Vendor movement test register; not used here |
| `0x07` | `FIRMWARE_VERSION` | Read controller firmware string |
| `0x09` | `GAIT_TYPE` | Trot, walk or high-walk selection |
| `0x13` | `BT_NAME` | Bluetooth-name storage in the public map |
| `0x20` | `LOAD/UNLOAD_MOTOR` | Torque/load operation selected by payload |
| `0x30` | `VX` | Forward/backward command |
| `0x31` | `VY` | Lateral command |
| `0x32` | `VYAW` | Turning command |
| `0x33–0x35` | `TRANSLATION` | Body x, y and z translation |
| `0x36–0x38` | `ATTITUDE` | Body roll, pitch and yaw |
| `0x39–0x3B` | `PERIODIC_ROT` | Periodic roll, pitch and yaw |
| `0x3C` | `MarkTime` | Stationary stepping height |
| `0x3D` | `MOVE_MODE` | Slow, normal or high pace |
| `0x3E` | `ACTION` | Run a firmware action group |
| `0x40–0x4B` | `LEG_POS` | Three coordinates for each of four legs |
| `0x50–0x5B` | `MOTOR_ANGLE` | Command or read twelve servo angles |
| `0x5C` | `MOTOR_SPEED` | Speed for direct servo-angle control |
| `0x61` | `IMU` | Enable or disable controller stabilization |
| `0x62–0x64` | `ROLL/PITCH/YAW` | Read fused controller attitude |
| `0x65` | `IMU_RAW` | Read 24-byte acceleration, gyro and RPY payload |
| `0x80–0x82` | `PERIODIC_TRAN` | Periodic x, y and z translation |

Registers described by the library are not automatically safe to expose in
ROS. Upgrade, calibration, direct motor, load/unload and high-walk functions
require separate physical test plans.

## Host-library scaling and limits

The library maps physical-looking values into unsigned bytes. Values outside
the configured range are saturated.

| Command | Public range or wrapper behavior |
| --- | --- |
| `move_x()` | Clamped to -20 through 20; protocol scale limit is 25 |
| `move_y()` | Clamped to -18 through 18 |
| `turn()` | Clamped to -70 through 70; non-zero magnitudes below 30 become 30 |
| Body translation x/y/z | ±35, ±19.5, and 75–115 respectively |
| Body roll/pitch/yaw | ±20, ±22 and ±16 respectively |
| Mark-time height | 10–35 when non-zero |
| Direct motor speed | 1–255; input zero is converted to 1 |
| Motor IDs | 11–13, 21–23, 31–33 and 41–43 |

The controller command values are not SI velocities. `safe_base` provides a
ROS-facing scale and clamps `/cmd_vel`, but DOGZILLA has no wheel encoder that
proves commanded metres per second. Scan-matched odometry is the measured
motion source used by navigation.

`turn()` has a firmware-library dead zone: any non-zero command becomes at
least 30 controller units. The separate 1–9 turn profile and steering guard
exist partly to manage this discontinuity.

## Public Python API by function

| Group | Methods |
| --- | --- |
| Velocity | `move`, `move_x`, `move_y`, `forward`, `back`, `left`, `right`, `turn`, `turnleft`, `turnright`, `stop` |
| Body pose | `translation`, `attitude`, `periodic_tran`, `periodic_rot`, `mark_time` |
| Gait/mode | `pace`, `gait_type`, `imu`, `perform` |
| Preset actions | `action`, `reset` |
| Direct limbs | `leg`, `motor`, `motor_speed` |
| Torque | `load_motor`, `load_allmotor`, `unload_motor`, `unload_allmotor` |
| Telemetry | `read_battery`, `read_motor`, `read_version`, `read_roll`, `read_pitch`, `read_yaw`, `read_imu_raw` |
| Maintenance | `calibration` |

`stop()` writes zero x velocity, zero y velocity, zero mark-time command and
zero yaw command. It does not unload the servos or power down the controller.

`reset()` sends action ID 255. The stock application treats this as stop and
return to its initial posture.

## Preset action groups

The fixed action names below are the public mapping used by Yahboom samples and
the guarded vision policy.

| ID | Name | ID | Name |
| --- | --- | --- | --- |
| 1 | lie down | 11 | pee |
| 2 | stand up | 12 | sit down |
| 3 | crawl | 13 | wave hand |
| 4 | turn around | 14 | stretch |
| 5 | mark time | 15 | wave body |
| 6 | squat | 16 | swing |
| 7 | turn roll | 17 | pray |
| 8 | turn pitch | 18 | seek |
| 9 | turn yaw | 19 | handshake |
| 10 | three axis | 255 | reset/default posture |

The stock Raspberry Pi app sends motor speed 50 and action 14 at startup as its
stretch display. This is an application behavior, not evidence that the
embedded controller automatically performs action 14 on every power-up.

Action IDs identify firmware-owned trajectories, but their joint curves,
timing, torque transitions and completion acknowledgements are not published.
The host sends a command and normally waits a conservative guard time.

## IMU packet behavior

`read_imu_raw()` requests 24 bytes and returns nine values:

```text
accX, accY, accZ, gyroX, gyroY, gyroZ, roll, pitch, yaw
```

The host library converts signed 16-bit acceleration using `9.8 / 16384` and
gyro using `1 / 16.4`. Gyro values are degrees per second. The last three
values are controller-provided floats converted toward radians.

The ROS boundary intentionally uses only raw acceleration and gyro:

- gyro is converted from degrees/s to radians/s;
- the message is timestamped after the complete packet arrives because the
  packet has no sensor clock;
- controller orientation is marked unavailable because its world-frame and
  fusion convention are undocumented;
- `imu_corrector` applies the robot-specific axis mapping, gravity convention,
  scale, gyro bias and measured covariance from `calibration/imu.json`.

## Battery and low-power behavior

The public readback is an integer percentage. It does not expose voltage,
current, cell balance, temperature, remaining capacity or a hardware timestamp.
A zero value means the read failed; it is not a confirmed empty battery.

Observed/controller behavior and repository behavior are deliberately split:

| Condition | Embedded controller | ROS `safe_base` |
| --- | --- | --- |
| Valid battery above 25% | Normal controller behavior | Movement permitted unless E-stop is latched |
| Valid battery at or below 25% | Controller's private low-battery rest is expected to win | Sends stop, inhibits ROS movement and waits for recovery to at least 28% |
| Invalid battery read | Unknown; no new fact is available | Retains the last confirmed safety state; does not treat zero as a real 0% reading |
| New autonomous task | No task concept | Web requires a fresh valid reading at or above its configured task threshold, default 28% |
| Active autonomous task | No task concept | A stale/invalid reading alone does not stop motion; a confirmed valid low reading does |

The physical power-button shutdown and low-battery rest appear smooth because
they are controller-owned routines. Their exact joint sequence and torque-off
timing are not exposed by `DOGZILLALib`.

## Rest, stand and servo release

These operations are not interchangeable:

- Action 1 is the public **lie-down** animation.
- `unload_allmotor()` immediately requests servo release.
- The power-button/low-battery routine is a private safety trajectory followed
  by controller-managed torque behavior.

This repository does not claim action 1 plus unload is equivalent to the
private routine. `dogzilla rest` is therefore disabled and sends no movement or
torque command. This protects the robot from an abrupt fall or joint collision.

`dogzilla stand` is supported with a narrow path:

1. require a valid battery strictly above 25%;
2. send controller stop;
3. load all motors at their current positions;
4. send public stand-up action 2;
5. hold a four-second action guard before closing the port.

The passive rest recorder reads only battery and twelve motor angles. It can
capture a naturally occurring low-battery trajectory with pre-roll and a
stationary tail. Every capture is marked `replay_enabled: false`; a captured
curve is evidence, not an approved command profile.

## Original Yahboom application

`app_dogzilla/app_dogzilla.py` is the stock-style mobile-app server. It:

- opens `DOGZILLALib` directly;
- hosts a TCP/app and camera interface;
- creates joystick and OLED helpers around the controller object;
- sets motor speed 50 and plays stretch action 14 at startup.

Standalone `joystick_dogzilla.py` and `oled_dogzilla.py` also open the serial
port directly. The provided systemd OLED service is therefore incompatible
with a simultaneous ROS serial owner. The deployment wrapper handles this
service explicitly.

The original `kill_dogzilla.sh` uses process matching and `kill -9`. The new
operator wrapper does not use that as its normal path: it first requests
Ctrl-C and refuses to take the serial port if the vendor application does not
close cleanly.

## Extending controller support safely

When adding a controller feature:

1. Add it to `dogzilla_safe_base` or another component inside the same
   serial-owning process. Do not create a second `/dev/ttyAMA0` node.
2. Validate every parameter before opening the port.
3. Stop walking before posture, mode, action, direct motor or torque changes.
4. Require a valid battery for any new action that can move or load servos.
5. Use bounded serial timeouts. The project wraps the vendor's private
   one-second read loop down to a configured maximum of 0.08 seconds.
6. Publish explicit state so the web UI and logs show whether a request was
   accepted, blocked or timed out.
7. Add fake-controller unit tests for packet-independent policy and failure
   handling.
8. Test one new physical operation at a time on a clear, level floor with an
   operator ready to use the physical power control.

Do not introduce raw register writes into the browser, vision node or mission
executor. Those components are intentionally hardware-free.

## Unknowns that must stay labelled unknown

The following cannot be reconstructed from the public Python library alone:

- embedded inverse kinematics and gait phase logic;
- servo PID gains and current/torque limits;
- action trajectory curves and completion state;
- the battery percentage estimator;
- the physical power-button state machine;
- low-battery rest and servo-release timing;
- controller self-stabilization coordinate conventions;
- whether Bluetooth control arbitrates with serial commands internally;
- firmware upgrade format and recovery behavior.

Do not fill these gaps with guessed motor sequences. Obtain official controller
documentation, capture read-only evidence, or build a separately reviewed
physical test before changing the safety boundary.

