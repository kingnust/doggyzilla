"""Apply the measured DOGZILLA axis, gravity, bias, and noise calibration."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu_calibration import load_calibration, matvec


class ImuCorrector(Node):
    """Publish calibrated REP-145 IMU data for Cartographer."""

    def __init__(self):
        super().__init__('dogzilla_imu_corrector')

        self.declare_parameter('input_topic', '/imu/data_uncalibrated')
        self.declare_parameter('output_topic', '/imu/data_corrected')
        self.declare_parameter('calibration_file', '/calibration/imu.json')
        self.declare_parameter('output_frame', 'imu_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        calibration_file = self.get_parameter('calibration_file').value
        calibration = load_calibration(calibration_file)
        self._output_frame = self.get_parameter('output_frame').value
        self._acceleration_matrix = calibration['acceleration']['matrix']
        self._acceleration_scale = float(calibration['acceleration']['scale'])
        self._acceleration_covariance = [
            float(value) for value in calibration['acceleration']['covariance']
        ]
        self._gyro_matrix = calibration['angular_velocity']['matrix']
        self._gyro_bias = [
            float(value)
            for value in calibration['angular_velocity']['bias_rad_s']
        ]
        self._gyro_covariance = [
            float(value)
            for value in calibration['angular_velocity']['covariance']
        ]
        self._last_stamp_ns = None

        self._publisher = self.create_publisher(
            Imu,
            output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Imu,
            input_topic,
            self._correct_imu,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Applying IMU calibration {calibration_file}: '
            f'{input_topic} -> {output_topic}'
        )

    def _correct_imu(self, source):
        stamp_ns = (
            source.header.stamp.sec * 1_000_000_000
            + source.header.stamp.nanosec
        )
        if stamp_ns <= 0:
            self.get_logger().warn('Dropping IMU packet with an invalid timestamp')
            return
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self.get_logger().warn('Dropping non-monotonic IMU timestamp')
            return
        self._last_stamp_ns = stamp_ns

        acceleration = matvec(
            self._acceleration_matrix,
            [
                source.linear_acceleration.x,
                source.linear_acceleration.y,
                source.linear_acceleration.z,
            ],
        )
        acceleration = [
            self._acceleration_scale * value for value in acceleration
        ]
        angular_velocity = matvec(
            self._gyro_matrix,
            [
                source.angular_velocity.x,
                source.angular_velocity.y,
                source.angular_velocity.z,
            ],
        )
        angular_velocity = [
            angular_velocity[index] - self._gyro_bias[index]
            for index in range(3)
        ]

        corrected = Imu()
        corrected.header = source.header
        corrected.header.frame_id = self._output_frame
        corrected.orientation_covariance[0] = -1.0
        corrected.angular_velocity.x = angular_velocity[0]
        corrected.angular_velocity.y = angular_velocity[1]
        corrected.angular_velocity.z = angular_velocity[2]
        corrected.angular_velocity_covariance = self._gyro_covariance
        corrected.linear_acceleration.x = acceleration[0]
        corrected.linear_acceleration.y = acceleration[1]
        corrected.linear_acceleration.z = acceleration[2]
        corrected.linear_acceleration_covariance = self._acceleration_covariance

        self._publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
