"""Safely run DOGZILLA's native lie-down and stand-up animations."""

import argparse
import time

import DOGZILLALib as dog


LOW_BATTERY_PERCENT = 25


class ServoPowerSafetyError(RuntimeError):
    """Raised when a pose command would bypass a safety invariant."""


def read_battery_percent(controller):
    """Read and validate Yahboom's integer battery percentage."""
    try:
        battery = int(controller.read_battery())
    except (TypeError, ValueError, OSError) as exc:
        raise ServoPowerSafetyError(
            'Battery reading failed; STAND is blocked to protect the robot.'
        ) from exc
    if not 1 <= battery <= 100:
        raise ServoPowerSafetyError(
            'Battery reading failed; STAND is blocked to protect the robot.'
        )
    return battery


def apply_servo_mode(
    mode,
    controller,
    sleep=time.sleep,
    low_battery_percent=LOW_BATTERY_PERCENT,
):
    """Apply a vendor pose action to an open controller."""
    battery = None
    if mode == 'stand':
        battery = read_battery_percent(controller)
        if battery <= low_battery_percent:
            raise ServoPowerSafetyError(
                f'Battery is {battery}%, at or below the '
                f'{low_battery_percent}% protection threshold. '
                'Yahboom low-battery rest remains active; charge first.'
            )

    controller.stop()
    sleep(0.20)

    if mode == 'rest':
        # Yahboom action group 1 is the normal animated lie-down sequence.
        controller.action(1)
        sleep(3.00)
        # Release torque only after the body is safely in the low pose.
        controller.unload_allmotor()
        sleep(0.20)
        return battery
    if mode == 'stand':
        # Recover cleanly if an older rest command left servo torque unloaded.
        controller.load_allmotor()
        sleep(0.25)
        # Yahboom action group 2 is the normal animated stand-up sequence.
        controller.action(2)
        sleep(3.00)
        return battery
    raise ValueError('mode must be rest or stand')


def close_controller(controller):
    """Close the controller serial handle after queued bytes are written."""
    serial_port = getattr(controller, 'ser', None)
    if serial_port is not None and serial_port.is_open:
        serial_port.close()


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Run DOGZILLA lie-down or stand-up animation.',
    )
    parser.add_argument('mode', choices=('rest', 'stand'))
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parsed = parser.parse_args(args)

    controller = dog.DOGZILLA(port=parsed.port)
    try:
        battery = apply_servo_mode(parsed.mode, controller)
    except ServoPowerSafetyError as exc:
        print(f'SAFETY: {exc}')
        return 2
    finally:
        close_controller(controller)

    if parsed.mode == 'rest':
        print(
            'DOGZILLA lie-down animation completed; all leg-servo torque '
            'is now released.'
        )
    else:
        print(
            f'DOGZILLA battery: {battery}%. Stand-up animation completed.'
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
