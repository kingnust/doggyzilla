# DOGZILLA computer vision

This package turns Yahboom's Jupyter camera demonstrations into one reusable
ROS 2 pipeline. The algorithms are based on the installed DOGZILLA samples and
the lesson order on the official DOGZILLA study page.

## Implemented lessons

| Yahboom lesson | DOGZILLA mode | Current output |
| --- | --- | --- |
| Camera display | `raw` | Shared live camera and annotated JPEG |
| 8.1 Color recognition | `color` | Target centre, radius, area, and image offset |
| 8.2 Color tracking | `color-track` | Same continuous tracking measurements |
| 8.3 Color action | `color-action` | Allowlisted firmware-action proposal |
| 8.4 Face detection | `face` | Face boxes and normalized target offset |
| 8.5 Face tracking | `face-track` | Same continuous tracking measurements |
| 8.6 Watchdog | `watchdog` | Face-triggered handshake proposal from Yahboom's notebook |
| 8.7 QR recognition | `qr` | QR box and decoded text |
| 8.8 QR action | `qr-action` | Exact-phrase allowlisted action proposal |
| 8.11 Visual tracking | `line` | Lower-image line contour and steering offset |
| 8.11-8.12 Line following | `line-follow` | Steering/forward intent proposal |

The red, green, blue, and yellow HSV presets come from Yahboom lessons
8.1-8.3. The default line HSV range comes from the installed
`Samples/3_AI_Visual/11_12.followline/LineFollowHSV.text` file.

The camera processor always publishes observations and read-only intent. It
never opens `/dev/ttyAMA0`, sends velocity, runs an action group, or releases a
servo. Every result contains `action_output: disabled`; every proposal contains
`executed: false` and `requires_explicit_arming: true`. Arbitrary QR text is
never interpreted as a command: only Yahboom's exact 19 labels can create a
proposal.

## Architecture

```text
/dev/video0
    |
    v
usb_cam (one owner) ---> /camera/image_raw ---> dogzilla_vision
                                                |             |
                                                v             v
                                    /vision/detections   annotated JPEG
                                                |             |
                                                +------ web gateway
                                                        (token required)

Explicitly armed vision-control only:

/vision/detections ---> serial-manager policy ---> DOGZILLALib action
                              |                         /dev/ttyAMA0
                              +--> bounded line speed
```

Standalone Vision starts `usb_cam` and `dogzilla_vision` in one camera-only
container. Visual-shadow mode already owns the camera for RTAB-Map, so it adds
the processor to that same launch instead of starting another camera. Neither
path gives the vision processor a serial device.

The separate `vision_control` service owns the camera and controller serial
port in one guarded launch. Only the serial manager can execute a proposal.
The launch is serial-free by default (`armed:=false`); the operator command is
the only supported way to start it armed.

## Operator commands

Start a detection-only session and its authenticated dashboard:

```bash
dogzilla vision color red
```

`vision` selects the camera-only mode. `color` is the detector. `red` is the
HSV preset. The command starts no legs, LiDAR, IMU, or motor serial process.

Start an explicitly armed Yahboom-style behavior:

```bash
dogzilla vision-control color-action red
dogzilla vision-control watchdog
dogzilla vision-control qr-action
dogzilla vision-control line-follow
```

The command explains the hazard and requires the exact typed confirmation
`ARM VISION` before either device is opened. Armed control starts no LiDAR. It
locks the slow profile, accepts no external velocity topic, requires five
stable proposal frames, requires target release before repetition, enforces an
eight-second action cooldown and guard, stops immediately when a line is lost,
and retains the normal 0.6-second movement watchdog. Valid battery telemetry
strictly above 25% is required.

Change a running standalone or RTAB visual-shadow detector:

```bash
dogzilla vision-mode face
dogzilla vision-mode color-track blue
dogzilla vision-mode color-action red
dogzilla vision-mode watchdog
dogzilla vision-mode qr
dogzilla vision-mode qr-action
dogzilla vision-mode line
dogzilla vision-mode line-follow
```

Open the dashboard URL printed by the command. Obtain its token with:

```bash
./deploy/dogzilla-web show-token
```

Stop the camera, disarm control, stop movement, and close the dashboard:

```bash
dogzilla stop
```

When Vision is the only active device, `stop` does not open the LiDAR serial
port. Visual-shadow mode still follows its existing calibrated RTAB shutdown
and database-save path.

## Safety boundary and remaining lessons

Lessons 8.3, 8.6, 8.8, and basic 8.11 line following now have a guarded path
through the single `/dev/ttyAMA0` manager. The camera and web processes still
cannot execute anything directly. The dashboard shows a red ARMED state when
that separate session is active.

Lesson 8.12 obstacle crossing is not armed: it changes gait and continues
forward after detecting an obstacle, so it requires its own physical clearance
and stop tests. Lessons 8.9 climb, 8.10 kick/play-ball, 8.13 action learning,
and 8.14 synchronized teaching also remain outside automated control. The
original samples open `DOGZILLALib` directly, bypassing the ROS battery lockout,
limits, and watchdog; that architecture is intentionally not copied.

Source and synthetic validation do not replace physical acceptance. Test one
firmware action at a time on a supported, clear floor before treating armed
vision control as production-ready.
