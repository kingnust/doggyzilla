"""Suppress short Nav2 steering reversals without delaying stop commands."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class SteeringGuardFilter:
    """Apply an angular deadband and require persistent sign reversals."""

    def __init__(
        self,
        *,
        deadband_rps=0.04,
        reversal_hold_seconds=0.25,
        neutral_reset_seconds=0.50,
        bypass_angular_rps=0.50,
    ):
        self.deadband_rps = float(deadband_rps)
        self.reversal_hold_seconds = float(reversal_hold_seconds)
        self.neutral_reset_seconds = float(neutral_reset_seconds)
        self.bypass_angular_rps = float(bypass_angular_rps)
        if not 0.0 <= self.deadband_rps <= 0.20:
            raise ValueError('deadband_rps must be between 0 and 0.20')
        if not 0.0 <= self.reversal_hold_seconds <= 1.0:
            raise ValueError(
                'reversal_hold_seconds must be between 0 and 1.0'
            )
        if not 0.10 <= self.neutral_reset_seconds <= 2.0:
            raise ValueError(
                'neutral_reset_seconds must be between 0.10 and 2.0'
            )
        if not self.deadband_rps < self.bypass_angular_rps <= 2.0:
            raise ValueError(
                'bypass_angular_rps must exceed the deadband and be at most 2.0'
            )
        self.reset()

    def reset(self):
        """Forget accepted and pending directions after a long neutral period."""
        self._accepted_sign = 0
        self._pending_sign = 0
        self._pending_since = None
        self._neutral_since = None
        self._last_time = None

    def apply(self, angular_z, now=None):
        """Return one guarded angular command at monotonic time ``now``."""
        value = float(angular_z)
        current = time.monotonic() if now is None else float(now)
        if not math.isfinite(value) or not math.isfinite(current):
            self.reset()
            return 0.0

        if self._last_time is not None and (
            current < self._last_time
            or current - self._last_time >= self.neutral_reset_seconds
        ):
            self.reset()
        self._last_time = current

        if abs(value) <= self.deadband_rps:
            self._pending_sign = 0
            self._pending_since = None
            if self._neutral_since is None:
                self._neutral_since = current
            elif current - self._neutral_since >= self.neutral_reset_seconds:
                self._accepted_sign = 0
            return 0.0

        self._neutral_since = None
        sign = 1 if value > 0.0 else -1
        if self._accepted_sign in (0, sign):
            self._accepted_sign = sign
            self._pending_sign = 0
            self._pending_since = None
            return value

        if abs(value) >= self.bypass_angular_rps:
            self._accepted_sign = sign
            self._pending_sign = 0
            self._pending_since = None
            return value

        if self._pending_sign != sign or self._pending_since is None:
            self._pending_sign = sign
            self._pending_since = current
            return 0.0
        if current - self._pending_since < self.reversal_hold_seconds:
            return 0.0

        self._accepted_sign = sign
        self._pending_sign = 0
        self._pending_since = None
        return value


class SteeringGuardNode(Node):
    """Guard autonomous steering between Nav2 smoothing and twist_mux."""

    def __init__(self):
        super().__init__('dogzilla_steering_guard')
        self.declare_parameter('input_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('output_topic', '/cmd_vel_nav')
        self.declare_parameter('deadband_rps', 0.04)
        self.declare_parameter('reversal_hold_seconds', 0.25)
        self.declare_parameter('neutral_reset_seconds', 0.50)
        self.declare_parameter('bypass_angular_rps', 0.50)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if not input_topic or not output_topic or input_topic == output_topic:
            raise ValueError('steering guard topics must be non-empty and distinct')
        self._filter = SteeringGuardFilter(
            deadband_rps=self.get_parameter('deadband_rps').value,
            reversal_hold_seconds=(
                self.get_parameter('reversal_hold_seconds').value
            ),
            neutral_reset_seconds=(
                self.get_parameter('neutral_reset_seconds').value
            ),
            bypass_angular_rps=(
                self.get_parameter('bypass_angular_rps').value
            ),
        )
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._command_received,
            10,
        )
        self.get_logger().info(
            'Steering guard active: '
            f'deadband={self._filter.deadband_rps:.2f} rad/s, '
            f'reversal hold={self._filter.reversal_hold_seconds:.2f}s'
        )

    @staticmethod
    def _finite_twist(message):
        return all(math.isfinite(value) for value in (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        ))

    def _command_received(self, message):
        output = Twist()
        if not self._finite_twist(message):
            self._filter.reset()
            self._publisher.publish(output)
            self.get_logger().error(
                'Rejected a non-finite Nav2 velocity command'
            )
            return

        output.linear.x = message.linear.x
        output.linear.y = message.linear.y
        output.linear.z = message.linear.z
        output.angular.x = message.angular.x
        output.angular.y = message.angular.y
        output.angular.z = self._filter.apply(message.angular.z)
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
