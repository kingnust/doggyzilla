# DOGZILLA Coding Reference

This note is for fast future coding help in this workspace. It summarizes the local Raspberry Pi DOGZILLA app, samples, and the official Yahboom manual/repo index checked on 2026-06-18.

## Official Context

- Official study/manual index: https://www.yahboom.net/study/DOGZILLA
- Official GitHub manual/code index: https://github.com/YahboomTechnology/DOGZILLA
- Product family: Yahboom DOGZILLA S1/S2 12DOF robot dog for Raspberry Pi 5 / ROS2 Humble.
- Manual sections relevant to coding: Raspberry Pi system configuration, DOGZILLA control course, advanced OpenCV course, ROS2 courses, lidar course for S2, voice control, AI LLM course.
- Local bundle version: `Version.txt` says `Card NO.53, Version:2.0.0, Released 20250619`.

## Local Structure

- `DOGZILLA/app_dogzilla/`
  - `app_dogzilla.py`: Flask/gevent web app plus TCP app protocol server and joystick/OLED background tasks.
  - `joystick_dogzilla.py`: direct Linux joystick event reader for `/dev/input/js0`.
  - `camera_dogzilla.py`: OpenCV camera wrapper, tries video 0 then 1, MJPG/XVID, 640x480 default.
  - `oled_dogzilla.py`: SSD1306 status display, battery from DOGZILLALib, CPU/RAM/disk/IP from shell commands.
  - `start_app.sh`: sleeps 8 seconds, starts `python3 /home/pi/DOGZILLA/app_dogzilla/app_dogzilla.py`.
  - `my_service/*.service`: systemd units for joystick, OLED, and JupyterLab.
- `DOGZILLA/DOGZILLALib.zip`
  - Contains the source for `DOGZILLALib/DOGZILLALib.py`, version `2.0.7`.
  - This is the serial command layer imported by every DOGZILLA script.
- `DOGZILLA/Samples/2_Control/`
  - Jupyter notebooks for basic movement, gait/pace, body control, actions, IMU stabilization, motor, leg, and data reading.
- `DOGZILLA/Samples/3_AI_Visual/`
  - OpenCV examples: color recognition/tracking/action, face detection/tracking/handshake, QR code action, obstacle crossing, line following, action learning, sync movement.
- `DOGZILLA/Samples/4_Big_Modle/`
  - Voice/LLM/vision agent examples. `dog_agent/dog_base_control.py` wraps raw DOGZILLA calls into timed primitives.
- `run_humble.sh`
  - Starts Yahboom ROS Humble Docker image with host networking and mounted serial, video, joystick, speech, and USB devices.

## DOGZILLALib API

Instantiate with:

```python
from DOGZILLALib import DOGZILLA
dog = DOGZILLA()  # default serial port: /dev/ttyAMA0, 115200 baud
```

Movement:

- `stop()`: sets X/Y/mark-time/yaw to zero.
- `move('x'|'y', step)`, `move_x(step)`, `move_y(step)`.
- `forward(step)`, `back(step)`, `left(step)`, `right(step)`.
- `turn(step)`, `turnleft(step)`, `turnright(step)`.

Body pose:

- `translation('x'|'y'|'z', value)` or list form, e.g. `translation(['x','z'], [0, 95])`.
- `attitude('r'|'p'|'y', value)` or list form. `r` = roll, `p` = pitch, `y` = yaw/body shoulder angle.
- `periodic_rot(direction, period)` and `periodic_tran(direction, period)`.

Actions and gait:

- `action(action_id)`: preset action. `reset()` calls `action(255)`.
- `perform(0|1)`: toggle continuous action performance.
- `pace('slow'|'normal'|'high')`.
- `gait_type('trot'|'walk'|'high_walk')`.
- `mark_time(value)`.
- `imu(0|1)`: self-stabilization on/off.

Servo/leg control:

- `motor(motor_id, angle)` or list form. Valid motor IDs: `11,12,13,21,22,23,31,32,33,41,42,43`.
- `motor_speed(1..255)`, input `0` is converted to `1`.
- `leg(leg_id, [x, y, z])`, valid legs `1..4`.
- `unload_motor(leg_id)`, `load_motor(leg_id)`, `unload_allmotor()`, `load_allmotor()`.

Readback:

- `read_battery()`: integer percent-like value.
- `read_version()`: firmware version string.
- `read_motor(out_int=False)`: 12 motor angles.
- `read_roll(out_int=False)`, `read_pitch(out_int=False)`, `read_yaw(out_int=False)`.

Calibration:

- `calibration(state)` exists, but comments say use carefully.

## Important Limits

From `DOGZILLALib.PARAM` and clamping code:

- Body translation: X `[-35, 35]`, Y `[-18, 18]`, Z `[75, 115]`.
- Attitude: roll `[-20, 20]`, pitch `[-15, 15]`, yaw/body angle `[-11, 11]`.
- Leg position: X `[-35, 35]`, Y `[-18, 18]`, Z `[75, 115]`.
- Motor joint limits by joint position: lower `[-73, 57]`, middle `[-66, 93]`, upper `[-31, 31]`.
- `move_x` clamps to `[-20, 20]`.
- `move_y` clamps to `[-18, 18]`.
- `turn` clamps to `[-70, 70]`; any nonzero magnitude below 30 becomes `+/-30`.
- Periodic rotation/translation period: `[1.5, 8]`.
- Mark-time value: `[10, 35]`.

## Preset Action IDs Seen Locally

- `1`: lie down / get down
- `2`: stand up
- `3`: crawl
- `4`: turn around
- `5`: mark time
- `6`: squat
- `7`: turn roll
- `8`: turn pitch
- `9`: turn yaw
- `10`: 3-axis motion
- `11`: pee
- `12`: sit down
- `13`: wave hand
- `14`: stretch
- `15`: wave body
- `16`: swing
- `17`: pray
- `18`: foraging
- `19`: handshake
- `20`: app-level push-up special case in `app_dogzilla.py`
- `21`: push-up in `dog_base_control.py`
- `23`: dance in `dog_base_control.py`
- `255` / `0xff`: reset/default pose

## App Control Server

`DOGZILLA/app_dogzilla/app_dogzilla.py`:

- Flask HTTP listens on `0.0.0.0:6500`.
- TCP command server listens on local IP port `6000`.
- `/` renders `templates/index.html`, which displays `/video_feed`.
- `/init` starts the TCP server once and renders `templates/init.html`.
- `/video_feed` streams MJPEG only when `g_mode` is `Standard` or `Fullscreen`.
- Startup starts threads for push-up task, joystick handling, maybe OLED, initializes TCP, sets `motor_speed(50)`, then runs `action(14)` stretch.

TCP protocol:

- Frames are ASCII hex wrapped in `$...#`.
- Parser takes the last `$` and last `#` from a received TCP chunk.
- Byte layout is effectively: `$ TYPE CMD LEN DATA... CHECK #`.
- `LEN` must equal `len(frame) - 8`.
- Checksum is modulo-256 sum of every hex byte from after `$` up to before checksum.

Key app command IDs:

- `0F`: page/mode switch. `func 0` home/reset+battery, `1` standard, `2` fullscreen, `3` action page, `4` motor page, `5` leg page.
- `02`: return battery.
- `11`: joystick vector from app, maps signed X/Y percentages to `move_x`/`move_y`.
- `12`: button direction, `1` forward, `2` back, `3` left, `4` right, `5` turn left, `6` turn right, `7` reset, `0` stop.
- `13`: step width, clamped `20..100`.
- `14`: pace frequency, `1` slow plus Z 75, `2` normal, `3` high.
- `15`: IMU self-stabilization.
- `21`: body attitude roll/pitch.
- `22`: body height. Only applies when pace frequency > 1; clamps upper to 110.
- `23`: shoulder/body yaw. Input is negated and limited inside `(-11, 11)`.
- `31`: action. `0` reset, `20` app push-up, otherwise `dog.action(id)`.
- `32`: continuous perform toggle.
- `33`: reset leg pose or full reset.
- `41`: motor group for leg number 1..4.
- `51`: single leg XYZ control.
- `AA`: calibration if verify byte is `0x55`.

## Joystick Mapping

`DOGZILLA/app_dogzilla/joystick_dogzilla.py`:

- Opens `/dev/input/js<id>`, default `js0`.
- Reads Linux joystick events as `struct.unpack('IhBB', evbuf)`.
- Function map includes buttons `A,B,X,Y,L1,R1,SELECT,START,MODE,BTN_RK1,BTN_RK2` and axes `RK1_*`, `RK2_*`, `L2`, `R2`, `WSAD_*`.
- Initial internal values: step control `70`, pace frequency `2`, height `105`.
- Scales: X `0.2`, Y `0.2`, yaw `0.7`.

Controls:

- Left stick horizontal: `move('y', ...)`.
- Left stick vertical: `move('x', ...)`.
- Right stick horizontal: yaw turn only at full left/right due exact `value == 1 or -1` check.
- Right stick vertical: pitch attitude.
- `A`: lower height by 10 down to 75 when pace frequency > 1.
- `Y`: raise height by 10 up to 105 when pace frequency > 1.
- `B`: body yaw `-11` while pressed, then reset yaw/roll to 0.
- `X`: body yaw `+11` while pressed, then reset yaw/roll to 0.
- `L1`: action `10`.
- `R1`: kick-ball motor sequence on leg 2.
- `START`: reset dog, step control to 60, pace frequency 2, height 105.
- `BTN_RK1`: cycles step control by +30, wraps above 100 to 40.
- `BTN_RK2`: cycles pace slow/normal/high; slow also sets Z 75.
- `L2`: action `16` when fully pressed.
- `R2`: action `11` when fully pressed.
- D-pad WSAD axes duplicate X/Y movement.
- `SELECT` has obstacle-crossing logic commented out.

## Camera/OLED Notes

Camera:

- `Dogzilla_Camera(video_id=0, width=640, height=480)`.
- Falls back between `/dev/video0` and `/dev/video1`.
- MJPG for OpenCV 4+, XVID for OpenCV 3.
- `get_frame()` returns `(success, image)`; failure image is `bytes({1})`.
- `get_frame_jpg(text='')` can overlay short text and return JPEG bytes.

OLED:

- Uses `Adafruit_SSD1306.SSD1306_128_32`, PIL image/draw/font.
- Shows CPU, time, battery, RAM, disk, IP.
- Main loop calls `begin()` every pass for hot-plug behavior.
- `setBatteryShow()` reads battery every 30 cycles, but only when `battery_index == 1`.

## Sample Patterns Worth Reusing

- Basic control notebooks use `from DOGZILLALib import DOGZILLA; g_dog = DOGZILLA()`.
- Movement examples favor wrapper calls like `forward`, `back`, `left`, `right`, `turnleft`, `turnright`, and `stop`.
- Body control notebooks use `translation` and `attitude` sliders with last-value tracking to avoid redundant serial sends.
- Servo and leg notebooks call `load_allmotor()` before reset and set `motor_speed(50)` for direct limb control.
- Visual examples use OpenCV camera 0, 640x480 or 320x240, MJPG, threads, and `cv2.imencode('.jpg', frame)`.
- Follow-line and football examples set low walking posture: Z 75, pitch 15, `pace('slow')`.
- Agent wrappers in `dog_base_control.py` use timed movement plus `stop()` and action sleep delays.

## Practical Warnings For Future Edits

- Most modules instantiate `DOGZILLA()` at import time, which opens `/dev/ttyAMA0`; avoid importing robot modules on non-Pi test machines unless guarded or mocked.
- Direct servo/motor calls can fight gait/body commands. Reset or stop before switching modes.
- `gait_type()` has no invalid-mode `else`; invalid input would use a stale/local undefined `value`.
- `motor()` with a bad scalar motor ID can call `__motor(-1, data)`, because only list mode checks `-1`.
- App TCP parse catches broad exceptions, so command bugs can fail silently unless `debug` is enabled.
- Current Windows workspace has no usable local Python interpreter through `python`/`py`; rely on PowerShell for inspection here, but target runtime is Raspberry Pi Python 3.
