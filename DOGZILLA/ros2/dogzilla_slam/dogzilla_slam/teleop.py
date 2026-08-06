"""Safety-oriented keyboard teleoperation for DOGZILLA."""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


SPEED_PROFILES = {
    'slow': (0.10, 0.30),
    'normal': (0.25, 1.125),
    'high': (0.50, 1.75),
}

PROFILE_KEYS = {
    '1': 'slow',
    '2': 'normal',
    '3': 'high',
}

MOTION_KEYS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}

MENU = r'''
DOGZILLA keyboard control
-------------------------
        w              forward
   a         d         strafe left / right
        s              backward
   q         e         turn left / right

   Space or k           stop now
   1 / 2 / 3            slow / normal / high
   x                    stop and exit

Keep pressing a movement key to continue moving. Releasing it lets the
hardware watchdog stop the robot within 0.6 seconds.
'''


class DogzillaTeleop(Node):
    """Publish bounded commands and change the safe-base speed profile."""

    def __init__(self):
        super().__init__('dogzilla_teleop')
        self.declare_parameter('initial_profile', 'normal')
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._parameter_client = self.create_client(
            SetParameters,
            '/dogzilla_safe_base/set_parameters',
        )
        self._profile = 'slow'

    def set_profile(self, profile):
        """Apply one profile to both this publisher and the serial bridge."""
        if profile not in SPEED_PROFILES:
            raise ValueError('profile must be slow, normal, or high')
        self.stop()
        if not self._parameter_client.wait_for_service(timeout_sec=2.0):
            print('Cannot reach /dogzilla_safe_base; speed was not changed.')
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'speed_profile',
                Parameter.Type.STRING,
                profile,
            ).to_parameter_msg(),
        ]
        future = self._parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            print('Speed-profile request timed out.')
            return False
        results = future.result().results
        if not results or not all(result.successful for result in results):
            reason = results[0].reason if results else 'no response'
            print(f'Speed-profile change failed: {reason}')
            return False

        self._profile = profile
        linear, angular = SPEED_PROFILES[profile]
        print(
            f'Profile: {profile} '
            f'(linear {linear:.2f} m/s, angular {angular:.3f} rad/s)'
        )
        return True

    def move(self, linear_x, linear_y, angular_z):
        """Publish one command; safe_base applies final clamps and timeout."""
        max_linear, max_angular = SPEED_PROFILES[self._profile]
        message = Twist()
        message.linear.x = linear_x * max_linear
        message.linear.y = linear_y * max_linear
        message.angular.z = angular_z * max_angular
        self._publisher.publish(message)

    def stop(self):
        """Publish a zero velocity command."""
        self._publisher.publish(Twist())


def read_key(timeout=0.10):
    """Read one terminal key while allowing periodic interruption checks."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return ''
    return sys.stdin.read(1)


def main(args=None):
    if not sys.stdin.isatty():
        raise SystemExit('DOGZILLA teleop requires an interactive terminal')

    rclpy.init(args=args)
    node = DogzillaTeleop()
    terminal_settings = termios.tcgetattr(sys.stdin)
    exit_status = 0

    try:
        initial_profile = str(
            node.get_parameter('initial_profile').value
        ).lower()
        if not node.set_profile(initial_profile):
            raise RuntimeError('safe-base speed setup failed')
        print(MENU)
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            key = read_key().lower()
            if not key:
                continue
            if key in MOTION_KEYS:
                node.move(*MOTION_KEYS[key])
            elif key in (' ', 'k'):
                node.stop()
                print('Stopped.')
            elif key in PROFILE_KEYS:
                node.set_profile(PROFILE_KEYS[key])
            elif key == 'x':
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_status = 1
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nDOGZILLA stopped; teleop closed.')

    raise SystemExit(exit_status)


if __name__ == '__main__':
    main()
