"""Guided six-pose calibration for the DOGZILLA controller IMU."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import DOGZILLALib as dog

from .imu_calibration import create_calibration, vector_mean


POSE_PROMPTS = (
    ('upright', 'body level, feet downward'),
    ('nose_up', 'nose pointing upward; support the body securely'),
    ('nose_down', 'nose pointing downward; support the body securely'),
    ('left_side_up', 'left side pointing upward'),
    ('right_side_up', 'right side pointing upward'),
    ('upside_down', 'body upside down without resting weight on the LiDAR'),
)


def bounded_serial_unpack(controller, timeout_s):
    """Bound Yahboom's private one-second busy wait for safer control."""
    original_unpack = getattr(controller, '_DOGZILLA__unpack')

    def bounded(timeout=timeout_s):
        return original_unpack(timeout=min(float(timeout), timeout_s))

    setattr(controller, '_DOGZILLA__unpack', bounded)


def capture_samples(controller, count, rate_hz):
    samples = []
    stamps = []
    period = 1.0 / rate_hz
    failures = 0
    while len(samples) < count:
        started = time.monotonic()
        raw = controller.read_imu_raw()
        completed = time.monotonic()
        if len(raw) >= 6 and any(abs(value) > 1e-9 for value in raw[:6]):
            samples.append([float(value) for value in raw[:6]])
            stamps.append(completed)
            print(f'\r  samples: {len(samples):3d}/{count}', end='', flush=True)
            failures = 0
        else:
            failures += 1
            if failures >= 10:
                raise RuntimeError('Ten consecutive IMU reads failed; check /dev/ttyAMA0')
        remaining = period - (time.monotonic() - started)
        if remaining > 0:
            time.sleep(remaining)
    print()
    return samples, stamps


def atomic_write_json(path, document, owner_uid, owner_gid):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write('\n')
    os.replace(temporary, output)
    if owner_uid is not None and owner_gid is not None:
        os.chown(output, owner_uid, owner_gid)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Safely collect a guided six-pose DOGZILLA IMU calibration.',
    )
    parser.add_argument('--output', default='/calibration/imu.json')
    parser.add_argument('--samples-per-pose', type=int, default=50)
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--serial-timeout', type=float, default=0.08)
    parser.add_argument('--owner-uid', type=int)
    parser.add_argument('--owner-gid', type=int)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.samples_per_pose < 20:
        raise SystemExit('--samples-per-pose must be at least 20')
    if not 5.0 <= arguments.rate <= 50.0:
        raise SystemExit('--rate must be between 5 and 50 Hz')

    print('DOGZILLA guided IMU calibration')
    print('The legs will receive STOP before any samples are read.')
    print('Use two hands or a stable padded support for every tilted pose.')
    print('Do not let the robot or its LiDAR bear weight in an unsafe pose.')
    input('Press Enter when the robot is supported and ready, or Ctrl+C to cancel: ')

    controller = dog.DOGZILLA()
    bounded_serial_unpack(controller, arguments.serial_timeout)
    pose_samples = {}
    upright_stamps = []
    try:
        controller.stop()
        for name, description in POSE_PROMPTS:
            input(f'\nPlace the robot {description}. Press Enter when motionless: ')
            time.sleep(1.0)
            samples, stamps = capture_samples(
                controller,
                arguments.samples_per_pose,
                arguments.rate,
            )
            pose_samples[name] = samples
            if name == 'upright':
                upright_stamps = stamps
            mean = vector_mean([sample[:3] for sample in samples])
            print(
                '  raw acceleration mean: '
                f'x={mean[0]:+.3f}, y={mean[1]:+.3f}, z={mean[2]:+.3f} m/s^2'
            )

        document = create_calibration(
            pose_samples,
            upright_stamps,
            datetime.now(timezone.utc).isoformat(),
        )
        atomic_write_json(
            arguments.output,
            document,
            arguments.owner_uid,
            arguments.owner_gid,
        )
        print(f'\nCalibration saved to {arguments.output}')
        print(
            'Axis RMS error: '
            f'{document["quality"]["axis_rms_unit_error"]:.4f}; '
            'axes validated.'
        )
        print(
            'Measured gyro bias (rad/s): '
            + ', '.join(
                f'{value:+.6f}'
                for value in document['angular_velocity']['bias_rad_s']
            )
        )
    except (KeyboardInterrupt, EOFError):
        print('\nCalibration cancelled; the previous file was not replaced.')
        raise SystemExit(130)
    finally:
        try:
            controller.stop()
        finally:
            controller.ser.close()


if __name__ == '__main__':
    main()
