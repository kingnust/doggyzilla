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

from .speed_control import MAXIMUM_SPEED_LEVEL
from .speed_control import MINIMUM_SPEED_LEVEL
from .speed_control import NORMAL_SPEED_LEVEL
from .speed_control import normalize_speed_level
from .speed_control import SPEED_LEVELS
from .speed_control import TURN_LEVELS

SPEED_KEYS = frozenset(str(level) for level in SPEED_LEVELS)
TURN_KEYS = {'-': -1, '=': 1, '+': 1}

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
   1 ... 9              set movement speed; 1 slow, 5 normal, 9 fast
   - / = or +           decrease / increase turning speed
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


def next_turn_level(level, direction):
    """Return one bounded turn-level step for -1 or +1."""
    level = normalize_speed_level(level)
    if direction not in (-1, 1):
        raise ValueError('turn-level direction must be -1 or 1')
    return max(
        MINIMUM_SPEED_LEVEL,
        min(MAXIMUM_SPEED_LEVEL, level + direction),
    )


class DogzillaTeleop(Node):
    """Publish bounded commands and change the safe-base speed level."""

    def __init__(self):
        super().__init__('dogzilla_teleop')
        self.declare_parameter('initial_level', NORMAL_SPEED_LEVEL)
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
        self._speed_level = 1
        self._turn_level = 1
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

    def set_speed_level(self, value):
        """Apply one 1-9 translation level to the serial bridge."""
        level = normalize_speed_level(value)
        if not self.set_remote_parameters([
            Parameter(
                'speed_level',
                Parameter.Type.INTEGER,
                level,
            ),
        ], 'speed level'):
            return False

        self._speed_level = level
        setting = SPEED_LEVELS[level]
        print(
            f'Speed {level}: {setting.label} '
            f'(step {setting.controller_step}, '
            f'linear {setting.max_linear:.3f} m/s)'
        )
        return True

    def set_initial_levels(self, value):
        """Apply the same startup level to translation and turning."""
        level = normalize_speed_level(value)
        if not self.set_remote_parameters([
            Parameter('speed_level', Parameter.Type.INTEGER, level),
            Parameter('turn_level', Parameter.Type.INTEGER, level),
        ], 'initial motion levels'):
            return False

        self._speed_level = level
        self._turn_level = level
        speed_setting = SPEED_LEVELS[level]
        turn_setting = TURN_LEVELS[level]
        print(
            f'Initial levels: movement {level}, turning {level} '
            f'(linear {speed_setting.max_linear:.3f} m/s, '
            f'angular {turn_setting.max_angular:.3f} rad/s)'
        )
        return True

    def adjust_turn_level(self, direction):
        """Increase or decrease the independent turning level by one."""
        level = next_turn_level(self._turn_level, direction)
        if level == self._turn_level:
            print(f'Turning level {level}.')
            return True
        if not self.set_remote_parameters([
            Parameter('turn_level', Parameter.Type.INTEGER, level),
        ], 'turning level'):
            return False

        self._turn_level = level
        setting = TURN_LEVELS[level]
        print(
            f'Turning level {level}: '
            f'{setting.max_angular:.3f} rad/s'
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
        speed_setting = SPEED_LEVELS[self._speed_level]
        turn_setting = TURN_LEVELS[self._turn_level]
        message = Twist()
        message.linear.x = linear_x * speed_setting.max_linear
        message.linear.y = linear_y * speed_setting.max_linear
        message.angular.z = angular_z * turn_setting.max_angular
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
        initial_level = normalize_speed_level(
            node.get_parameter('initial_level').value
        )
        if not node.set_initial_levels(initial_level):
            raise RuntimeError('safe-base motion-level setup failed')
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
            elif key in SPEED_KEYS:
                node.set_speed_level(key)
            elif key in TURN_KEYS:
                node.adjust_turn_level(TURN_KEYS[key])
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
