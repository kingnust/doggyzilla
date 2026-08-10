"""Safely run supported DOGZILLA servo-power operations."""

import argparse
import time

import DOGZILLALib as dog


LOW_BATTERY_PERCENT = 25
STAND_ACTION_GUARD_SECONDS = 4.0


class ServoPowerSafetyError(RuntimeError):
    """Raised when a pose command would bypass a safety invariant."""


FIRMWARE_REST_UNAVAILABLE = (
    'REST is disabled: Yahboom action 1 is only the public preset lie-down '
    'action and has not been verified as the controller firmware\'s '
    'low-battery/power-button safety trajectory. No servo command was sent. '
    'A captured and validated 12-joint firmware-rest profile is required.'
)


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
    if mode == 'rest':
        # Do not substitute public preset action 1 for the controller's private
        # low-battery/power-button sequence. The host protocol does not
        # establish that their trajectories or torque timing are identical.
        raise ServoPowerSafetyError(FIRMWARE_REST_UNAVAILABLE)

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

    if mode == 'stand':
        # Yahboom documents that this loads at the joints' current positions,
        # so the folded legs are held before the stand-up action begins.
        controller.load_allmotor()
        sleep(0.50)
        # Yahboom action group 2 is the normal animated stand-up sequence.
        controller.action(2)
        sleep(STAND_ACTION_GUARD_SECONDS)
        return battery
    raise ValueError('mode must be rest or stand')


def close_controller(controller):
    """Close the controller serial handle after queued bytes are written."""
    serial_port = getattr(controller, 'ser', None)
    if serial_port is not None and serial_port.is_open:
        serial_port.close()


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Run supported DOGZILLA servo-power operations.',
    )
    parser.add_argument('mode', choices=('rest', 'stand'))
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parsed = parser.parse_args(args)

    if parsed.mode == 'rest':
        print(f'SAFETY: {FIRMWARE_REST_UNAVAILABLE}')
        return 2

    controller = dog.DOGZILLA(port=parsed.port)
    try:
        battery = apply_servo_mode(parsed.mode, controller)
    except ServoPowerSafetyError as exc:
        print(f'SAFETY: {exc}')
        return 2
    finally:
        close_controller(controller)

    print(f'DOGZILLA battery: {battery}%. Stand-up animation completed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
