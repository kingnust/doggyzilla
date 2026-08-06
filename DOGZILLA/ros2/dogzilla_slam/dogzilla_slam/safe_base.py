"""Safe, single-owner DOGZILLA movement and raw-IMU serial bridge."""

import DOGZILLALib as dog
from geometry_msgs.msg import Twist
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class SafeBase(Node):
    """Clamp velocity commands and stop the controller after a short timeout."""

    def __init__(self):
        super().__init__('dogzilla_safe_base')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.30)
        self.declare_parameter('command_timeout', 0.60)
        self.declare_parameter('controller_scale', 40.0)
        self.declare_parameter('publish_imu', False)
        self.declare_parameter('raw_imu_topic', '/imu/data_uncalibrated')
        self.declare_parameter('raw_imu_frame', 'imu_link_raw')
        self.declare_parameter('imu_rate_hz', 20.0)
        self.declare_parameter('serial_read_timeout', 0.08)

        input_topic = self.get_parameter('input_topic').value
        self._max_linear = float(self.get_parameter('max_linear').value)
        self._max_angular = float(self.get_parameter('max_angular').value)
        self._command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self._controller_scale = float(
            self.get_parameter('controller_scale').value
        )
        self._publish_imu_enabled = bool(
            self.get_parameter('publish_imu').value
        )

        # This node is the only /dev/ttyAMA0 owner during mapping.
        self._dog = dog.DOGZILLA()
        self._bound_vendor_read_timeout(
            float(self.get_parameter('serial_read_timeout').value)
        )
        self._dog.stop()
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._apply_command,
            10,
        )
        self._last_command_time = None
        self._stopped = True
        self._timer = self.create_timer(0.10, self._watchdog)

        self._imu_publisher = None
        self._imu_timer = None
        self._imu_failures = 0
        if self._publish_imu_enabled:
            raw_imu_topic = self.get_parameter('raw_imu_topic').value
            self._raw_imu_frame = self.get_parameter('raw_imu_frame').value
            imu_rate_hz = float(self.get_parameter('imu_rate_hz').value)
            if not 5.0 <= imu_rate_hz <= 50.0:
                raise ValueError('imu_rate_hz must be between 5 and 50 Hz')
            self._imu_publisher = self.create_publisher(
                Imu,
                raw_imu_topic,
                qos_profile_sensor_data,
            )
            self._imu_timer = self.create_timer(
                1.0 / imu_rate_hz,
                self._publish_raw_imu,
            )
            self.get_logger().info(
                f'Uncalibrated IMU enabled at {imu_rate_hz:.1f} Hz on '
                f'{raw_imu_topic}'
            )

        self.get_logger().info(
            'Safe base active: '
            f'linear <= {self._max_linear:.2f}, '
            f'angular <= {self._max_angular:.2f}, '
            f'timeout = {self._command_timeout:.2f}s'
        )

    def _bound_vendor_read_timeout(self, timeout_s):
        """Limit Yahboom's private one-second serial busy wait."""
        if not 0.01 <= timeout_s <= 0.20:
            raise ValueError('serial_read_timeout must be between 0.01 and 0.20s')
        original_unpack = getattr(self._dog, '_DOGZILLA__unpack')

        def bounded_unpack(timeout=timeout_s):
            return original_unpack(timeout=min(float(timeout), timeout_s))

        setattr(self._dog, '_DOGZILLA__unpack', bounded_unpack)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def stop(self):
        """Stop movement directly through the serial controller."""
        try:
            self._dog.stop()
        except Exception as exc:  # Hardware errors must not block shutdown.
            if rclpy.ok():
                self.get_logger().error(f'Failed to stop DOGZILLA: {exc}')
        self._stopped = True

    def _apply_command(self, source):
        linear_x = self._clamp(source.linear.x, self._max_linear)
        linear_y = self._clamp(source.linear.y, self._max_linear)
        angular_z = self._clamp(source.angular.z, self._max_angular)

        self._last_command_time = self.get_clock().now()
        if (
            abs(linear_x) < 0.01
            and abs(linear_y) < 0.01
            and abs(angular_z) < 0.01
        ):
            self.stop()
            return

        self._dog.move('x', linear_x * self._controller_scale)
        self._dog.move('y', linear_y * self._controller_scale)
        self._dog.turn(angular_z * self._controller_scale)
        self._stopped = False

    def _watchdog(self):
        if self._last_command_time is None or self._stopped:
            return

        elapsed = (
            self.get_clock().now() - self._last_command_time
        ).nanoseconds / 1e9
        if elapsed <= self._command_timeout:
            return

        self.get_logger().warn('Command timeout: stopping DOGZILLA')
        self.stop()

    def _publish_raw_imu(self):
        """Read the controller IMU without opening a second serial owner."""
        raw = self._dog.read_imu_raw()
        if len(raw) < 6 or not any(abs(value) > 1e-9 for value in raw[:6]):
            self._imu_failures += 1
            if self._imu_failures == 1 or self._imu_failures % 20 == 0:
                self.get_logger().warn(
                    f'IMU serial read failed ({self._imu_failures} failures)'
                )
            if self._imu_failures == 10:
                self._imu_timer.cancel()
                self.get_logger().error(
                    'Disabling IMU reads after 10 consecutive failures; '
                    'movement watchdog remains active'
                )
            return

        self._imu_failures = 0
        message = Imu()
        # The packet has no hardware clock, so stamp it immediately after the
        # complete packet is received. imu_validate checks monotonicity/gaps.
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._raw_imu_frame

        # Yahboom reports gyro values in degrees/s. Convert at the hardware
        # boundary so every ROS Imu message uses the required radians/s.
        message.angular_velocity.x = math.radians(float(raw[3]))
        message.angular_velocity.y = math.radians(float(raw[4]))
        message.angular_velocity.z = math.radians(float(raw[5]))
        message.linear_acceleration.x = float(raw[0])
        message.linear_acceleration.y = float(raw[1])
        message.linear_acceleration.z = float(raw[2])

        # The controller's fused RPY world convention is undocumented. Mark
        # orientation unavailable rather than publishing a misleading value.
        message.orientation_covariance[0] = -1.0
        self._imu_publisher.publish(message)

    def close(self):
        """Stop motion and release the single serial handle."""
        self.stop()
        if self._dog.ser.is_open:
            self._dog.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = SafeBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # This is a direct serial command, so it still works after the ROS
        # context has begun shutting down.
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
