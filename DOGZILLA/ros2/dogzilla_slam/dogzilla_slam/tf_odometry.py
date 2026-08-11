"""Publish scan-matched odometry from Cartographer's odom-to-base TF."""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_yaw(quaternion):
    """Return planar yaw from a geometry quaternion."""
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TfOdometry(Node):
    """Convert Cartographer TF motion into the /odom topic Nav2 expects."""

    def __init__(self):
        super().__init__('dogzilla_tf_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('velocity_filter_alpha', 0.35)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._alpha = float(
            self.get_parameter('velocity_filter_alpha').value
        )
        if not 5.0 <= publish_rate <= 50.0:
            raise ValueError('publish_rate_hz must be between 5 and 50 Hz')
        if not 0.0 < self._alpha <= 1.0:
            raise ValueError('velocity_filter_alpha must be in (0, 1]')

        self._publisher = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            qos_profile_sensor_data,
        )
        self._buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._listener = TransformListener(self._buffer, self)
        self._previous = None
        self._filtered_twist = [0.0, 0.0, 0.0]
        self._waiting_logged = False
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)

    def _publish(self):
        try:
            transform = self._buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            if not self._waiting_logged:
                self.get_logger().info(
                    f'Waiting for scan-matched odometry TF: {exc}'
                )
                self._waiting_logged = True
            return
        self._waiting_logged = False

        stamp = transform.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1e9
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_yaw(rotation)

        if self._previous is not None:
            previous_stamp, previous_x, previous_y, previous_yaw = self._previous
            elapsed = stamp_seconds - previous_stamp
            if 0.001 < elapsed < 1.0:
                world_dx = (translation.x - previous_x) / elapsed
                world_dy = (translation.y - previous_y) / elapsed
                raw_twist = (
                    math.cos(yaw) * world_dx + math.sin(yaw) * world_dy,
                    -math.sin(yaw) * world_dx + math.cos(yaw) * world_dy,
                    normalize_angle(yaw - previous_yaw) / elapsed,
                )
                self._filtered_twist = [
                    self._alpha * raw
                    + (1.0 - self._alpha) * filtered
                    for raw, filtered in zip(raw_twist, self._filtered_twist)
                ]
        self._previous = (
            stamp_seconds,
            translation.x,
            translation.y,
            yaw,
        )

        message = Odometry()
        message.header = transform.header
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = translation.x
        message.pose.pose.position.y = translation.y
        message.pose.pose.position.z = translation.z
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x = self._filtered_twist[0]
        message.twist.twist.linear.y = self._filtered_twist[1]
        message.twist.twist.angular.z = self._filtered_twist[2]

        # Non-zero covariance prevents consumers from treating scan-derived
        # motion as perfect wheel odometry.
        message.pose.covariance[0] = 0.02 ** 2
        message.pose.covariance[7] = 0.02 ** 2
        message.pose.covariance[14] = 0.05 ** 2
        message.pose.covariance[21] = 0.10 ** 2
        message.pose.covariance[28] = 0.10 ** 2
        message.pose.covariance[35] = 0.04 ** 2
        message.twist.covariance[0] = 0.04 ** 2
        message.twist.covariance[7] = 0.04 ** 2
        message.twist.covariance[35] = 0.08 ** 2
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TfOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
