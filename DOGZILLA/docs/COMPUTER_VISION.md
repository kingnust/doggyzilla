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
| 8.4 Face detection | `face` | Face boxes and normalized target offset |
| 8.5 Face tracking | `face-track` | Same continuous tracking measurements |
| 8.7 QR recognition | `qr` | QR box and decoded text |
| 8.11 Visual tracking | `line` | Lower-image line contour and steering offset |

The red, green, blue, and yellow HSV presets come from Yahboom lessons
8.1-8.3. The default line HSV range comes from the installed
`Samples/3_AI_Visual/11_12.followline/LineFollowHSV.text` file.

These modes publish observations only. They do not open `/dev/ttyAMA0`, send
velocity, change posture, run an action group, or release a servo. Every result
contains `action_output: disabled`, and unsupported names such as `qr-action`
are rejected.

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
```

Standalone Vision starts `usb_cam` and `dogzilla_vision` in one camera-only
container. Visual-shadow mode already owns the camera for RTAB-Map, so it adds
the processor to that same launch instead of starting another camera. Neither
path gives the vision processor a serial device.

## Operator commands

Start a detection-only session and its authenticated dashboard:

```bash
dogzilla vision color red
```

`vision` selects the camera-only mode. `color` is the detector. `red` is the
HSV preset. The command starts no legs, LiDAR, IMU, or motor serial process.

Change a running standalone or RTAB visual-shadow detector:

```bash
dogzilla vision-mode face
dogzilla vision-mode color-track blue
dogzilla vision-mode qr
dogzilla vision-mode line
```

Open the dashboard URL printed by the command. Obtain its token with:

```bash
./deploy/dogzilla-web show-token
```

Stop the camera and standalone dashboard:

```bash
dogzilla stop
```

When Vision is the only active device, `stop` does not open the LiDAR serial
port. Visual-shadow mode still follows its existing calibrated RTAB shutdown
and database-save path.

## Not yet enabled

Yahboom lessons 8.3 (color action), 8.8 (QR action), 8.12 (line tracking plus
obstacle crossing), and 8.14 (teaching/synchronized action) can move joints or
the whole robot. The original notebooks call `DOGZILLALib` directly and bypass
the ROS serial owner, low-battery lockout, speed limits, and command watchdog.
They must not be copied into the long-running web process.

Their eventual integration needs an allowlisted action interface inside the
single `/dev/ttyAMA0` manager, explicit arming, current battery validation,
rate/debounce limits, cancel/stop behavior, and physical acceptance tests for
each firmware action. In particular, QR text must never be treated as an
unrestricted command.
