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

POSTURE_MENU = r'''
Body height and look direction (controller-only drive mode)
   r / f                body higher / lower
        i                look up (whole-body pitch)
   j         l           look left / right (whole-body yaw)
        ,                look down (whole-body pitch)
   c                    center look and restore 105 mm height
'''


def next_posture(key, height, pitch, yaw):
    """Return one bounded posture step for a keyboard key."""
    if key == 'r':
        height = min(110.0, height + 5.0)
    elif key == 'f':
        height = max(75.0, height - 5.0)
    elif key == 'i':
        pitch = max(-15.0, pitch - 5.0)
    elif key == ',':
        pitch = min(15.0, pitch + 5.0)
    elif key == 'j':
        yaw = min(11.0, yaw + 5.0)
    elif key == 'l':
        yaw = max(-11.0, yaw - 5.0)
    elif key == 'c':
        height, pitch, yaw = 105.0, 0.0, 0.0
    else:
        raise ValueError(f'unknown posture key: {key}')
    return height, pitch, yaw


class DogzillaTeleop(Node):
    """Publish bounded commands and change the safe-base speed profile."""

    def __init__(self):
        super().__init__('dogzilla_teleop')
        self.declare_parameter('initial_profile', 'normal')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('posture_controls', False)
        self._publisher = self.create_publisher(
            Twist,
            self.get_parameter('output_topic').value,
            10,
        )
        self._parameter_client = self.create_client(
            SetParameters,
            '/dogzilla_safe_base/set_parameters',
        )
        self._profile = 'slow'
        self._posture_controls = bool(
            self.get_parameter('posture_controls').value
        )
        self._body_height = 105.0
        self._head_pitch = 0.0
        self._head_yaw = 0.0

    def set_remote_parameters(self, parameters, label):
        """Apply a parameter batch to the single serial-owner node."""
        self.stop()
        if not self._parameter_client.wait_for_service(timeout_sec=2.0):
            print(f'Cannot reach /dogzilla_safe_base; {label} was not changed.')
            return False

        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        future = self._parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            print(f'{label} request timed out.')
            return False
        results = future.result().results
        if not results or not all(result.successful for result in results):
            reason = results[0].reason if results else 'no response'
            print(f'{label} change failed: {reason}')
            return False
        return True

    def set_profile(self, profile):
        """Apply one profile to both this publisher and the serial bridge."""
        if profile not in SPEED_PROFILES:
            raise ValueError('profile must be slow, normal, or high')
        if not self.set_remote_parameters([
            Parameter(
                'speed_profile',
                Parameter.Type.STRING,
                profile,
            ),
        ], 'speed profile'):
            return False

        self._profile = profile
        linear, angular = SPEED_PROFILES[profile]
        print(
            f'Profile: {profile} '
            f'(linear {linear:.2f} m/s, angular {angular:.3f} rad/s)'
        )
        return True

    def change_posture(self, key):
        """Apply one bounded height or look-direction keyboard step."""
        if not self._posture_controls:
            print(
                'Posture controls are disabled during mapping/localization '
                'because the LiDAR transform must remain fixed.'
            )
            return False

        try:
            height, pitch, yaw = next_posture(
                key,
                self._body_height,
                self._head_pitch,
                self._head_yaw,
            )
        except ValueError:
            return False

        changed = []
        for name, previous, value in (
            ('body_height', self._body_height, height),
            ('head_pitch', self._head_pitch, pitch),
            ('head_yaw', self._head_yaw, yaw),
        ):
            if value != previous:
                changed.append(Parameter(name, Parameter.Type.DOUBLE, value))
        if not changed:
            return True
        if not self.set_remote_parameters(changed, 'posture'):
            return False

        self._body_height = height
        self._head_pitch = pitch
        self._head_yaw = yaw
        print(
            'Posture: '
            f'height {height:.0f} mm, pitch {pitch:.0f}°, yaw {yaw:.0f}°'
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
        if node._posture_controls:
            print(POSTURE_MENU)
        else:
            print(
                'Posture keys are disabled in mapping/localization mode to '
                'keep the LiDAR transform fixed.\n'
            )
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
            elif key in ('r', 'f', 'i', 'j', 'l', ',', 'c'):
                node.change_posture(key)
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
