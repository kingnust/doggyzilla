"""Deterministic ROS fixture for no-hardware shadow integration testing."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ShadowFixture(Node):
    def __init__(self):
        super().__init__('dogzilla_shadow_test_fixture')
        self._image_publisher = self.create_publisher(
            Image, '/camera/image_rect', qos_profile_sensor_data
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_sensor_data
        )
        self._scan_publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data
        )
        self._odom_publisher = self.create_publisher(
            Odometry,
            '/rtabmap_shadow/odom_input',
            qos_profile_sensor_data,
        )
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._transform_broadcaster = TransformBroadcaster(self)
        self._publish_static_transforms()
        self._image_data = self._checkerboard_image()
        self._tick = 0
        self._timer = self.create_timer(0.05, self._publish)

    @staticmethod
    def _checkerboard_image():
        rows = []
        for y in range(480):
            rows.append(bytes(
                220 if ((x // 24) + (y // 24)) % 2 else 30
                for x in range(640)
            ))
        return b''.join(rows)

    def _publish_static_transforms(self):
        camera = TransformStamped()
        camera.header.stamp = self.get_clock().now().to_msg()
        camera.header.frame_id = 'base_link'
        camera.child_frame_id = 'camera_optical_frame'
        camera.transform.translation.x = 0.15
        camera.transform.translation.z = 0.075
        rotation = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        (
            camera.transform.rotation.x,
            camera.transform.rotation.y,
            camera.transform.rotation.z,
            camera.transform.rotation.w,
        ) = rotation

        laser = TransformStamped()
        laser.header = camera.header
        laser.child_frame_id = 'laser_frame'
        laser.transform.translation.z = 0.18
        laser.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform([camera, laser])

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera_optical_frame'
        image.height = 480
        image.width = 640
        image.encoding = 'mono8'
        image.is_bigendian = 0
        image.step = 640
        image.data = self._image_data
        self._image_publisher.publish(image)

        camera_info = CameraInfo()
        camera_info.header = image.header
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [
            420.0, 0.0, 320.0,
            0.0, 420.0, 240.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            420.0, 0.0, 320.0, 0.0,
            0.0, 420.0, 240.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._camera_info_publisher.publish(camera_info)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.orientation.w = 1.0
        odometry.pose.covariance[0] = 0.02 ** 2
        odometry.pose.covariance[7] = 0.02 ** 2
        odometry.pose.covariance[14] = 0.05 ** 2
        odometry.pose.covariance[21] = 0.10 ** 2
        odometry.pose.covariance[28] = 0.10 ** 2
        odometry.pose.covariance[35] = 0.04 ** 2
        odometry.twist.covariance[0] = 0.04 ** 2
        odometry.twist.covariance[7] = 0.04 ** 2
        odometry.twist.covariance[35] = 0.08 ** 2
        self._odom_publisher.publish(odometry)

        odom_transform = TransformStamped()
        odom_transform.header = odometry.header
        odom_transform.child_frame_id = 'base_link'
        odom_transform.transform.rotation.w = 1.0
        self._transform_broadcaster.sendTransform(odom_transform)

        if self._tick % 2 == 0:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'laser_frame'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 360.0
            scan.time_increment = 0.1 / 360.0
            scan.scan_time = 0.1
            scan.range_min = 0.05
            scan.range_max = 12.0
            scan.ranges = [
                2.0 + 0.25 * math.sin(index * math.pi / 45.0)
                for index in range(360)
            ]
            self._scan_publisher.publish(scan)
        self._tick += 1


def main():
    rclpy.init()
    node = ShadowFixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
