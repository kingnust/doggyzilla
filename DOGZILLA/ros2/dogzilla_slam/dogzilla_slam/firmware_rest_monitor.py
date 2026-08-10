"""Read-only serial monitor for capturing DOGZILLA firmware safety rest."""

import argparse
import time

import DOGZILLALib as dog

from .firmware_rest_capture import FirmwareRestRecorder
from .firmware_rest_capture import save_capture_atomic


JOINT_NAMES = tuple(
    f'leg{leg}_motor{motor}_joint'
    for leg in range(1, 5)
    for motor in range(1, 4)
)


def _print_events(recorder):
    for level, message in recorder.take_events():
        print(f'{level.upper()}: {message}', flush=True)


def monitor(controller, recorder, *, clock=time.monotonic, sleep=time.sleep):
    """Poll only battery and joint readback registers until capture finishes."""
    next_battery = clock()
    next_joints = next_battery

    while True:
        now = clock()
        if now >= next_battery:
            try:
                battery = int(controller.read_battery())
            except Exception as exc:
                print(f'WARNING: battery read failed: {exc}', flush=True)
                battery = 0
            if 1 <= battery <= 100:
                recorder.observe_battery(battery, now=clock())
                print(f'Battery: {battery}%', flush=True)
            else:
                print('WARNING: invalid battery readback', flush=True)
            _print_events(recorder)
            next_battery = clock() + 1.0

        if recorder.wants_high_joint_rate and now >= next_joints:
            try:
                angles = controller.read_motor()
            except Exception as exc:
                print(f'WARNING: joint read failed: {exc}', flush=True)
                angles = []
            recorder.observe_joints(angles, now=clock())
            _print_events(recorder)
            next_joints = clock() + 0.2
        elif not recorder.wants_high_joint_rate:
            next_joints = now

        if recorder.last_payload is not None:
            return (
                0
                if recorder.last_payload['status'] == 'captured_unvalidated'
                else 2
            )

        next_event = next_battery
        if recorder.wants_high_joint_rate:
            next_event = min(next_event, next_joints)
        sleep(max(0.01, min(0.10, next_event - clock())))


def close_controller(controller):
    serial_port = getattr(controller, 'ser', None)
    if serial_port is not None and serial_port.is_open:
        serial_port.close()


def main(args=None):
    parser = argparse.ArgumentParser(
        description=(
            'Passively record the controller-driven low-battery rest motion.'
        ),
    )
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument(
        '--output-directory',
        default='/profiles/captures',
    )
    parser.add_argument('--low-battery-percent', type=int, default=25)
    parser.add_argument('--arm-margin-percent', type=int, default=5)
    parsed = parser.parse_args(args)

    print(
        'READ-ONLY CAPTURE: this process sends only supported battery and '
        '12-joint read requests. It never sends movement, action, motor-angle, '
        'motor-speed, load, or unload commands.',
        flush=True,
    )
    print(
        'Do not press the physical power button; let the existing low-battery '
        'firmware routine occur naturally so the Raspberry Pi stays powered.',
        flush=True,
    )

    recorder = FirmwareRestRecorder(
        joint_names=JOINT_NAMES,
        low_battery_percent=parsed.low_battery_percent,
        arm_margin_percent=parsed.arm_margin_percent,
        save_callback=lambda payload: save_capture_atomic(
            payload,
            parsed.output_directory,
        ),
    )
    controller = dog.DOGZILLA(port=parsed.port)
    try:
        return monitor(controller, recorder)
    except KeyboardInterrupt:
        print('Capture cancelled; no movement or torque command was sent.')
        return 130
    finally:
        close_controller(controller)


if __name__ == '__main__':
    raise SystemExit(main())
