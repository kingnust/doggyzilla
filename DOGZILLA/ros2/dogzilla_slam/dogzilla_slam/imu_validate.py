"""Runtime checks for corrected DOGZILLA IMU data and timestamps."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuValidator(Node):
    def __init__(self, topic):
        super().__init__('dogzilla_imu_validator')
        self.messages = []
        self.received_monotonic = []
        self.received_ros_ns = []
        self.create_subscription(
            Imu,
            topic,
            self._receive,
            qos_profile_sensor_data,
        )

    def _receive(self, message):
        self.messages.append(message)
        self.received_monotonic.append(time.monotonic())
        self.received_ros_ns.append(self.get_clock().now().nanoseconds)


def stamp_nanoseconds(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/imu/data_corrected')
    parser.add_argument('--duration', type=float, default=8.0)
    return parser.parse_args(args)


def validate(messages, received, received_ros_ns, duration):
    failures = []
    if len(messages) < max(10, int(duration * 5)):
        failures.append(f'too few messages ({len(messages)})')
    if len(messages) < 2:
        return failures or ['fewer than two messages']

    stamps = [stamp_nanoseconds(message) for message in messages]
    if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
        failures.append('header timestamps are not strictly monotonic')

    timestamp_ages = [
        (received_stamp - message_stamp) / 1e9
        for received_stamp, message_stamp in zip(received_ros_ns, stamps)
    ]
    if max(timestamp_ages) > 0.25:
        failures.append(f'largest timestamp age is {max(timestamp_ages):.3f}s')
    if min(timestamp_ages) < -0.02:
        failures.append(
            f'a timestamp is {-min(timestamp_ages):.3f}s in the future'
        )

    stamp_gaps = [
        (later - earlier) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
    ]
    receive_gaps = [later - earlier for earlier, later in zip(received, received[1:])]
    if max(stamp_gaps) > 0.25:
        failures.append(f'largest timestamp gap is {max(stamp_gaps):.3f}s')
    if max(receive_gaps) > 0.25:
        failures.append(f'largest receive gap is {max(receive_gaps):.3f}s')

    accelerations = [
        math.sqrt(
            message.linear_acceleration.x ** 2
            + message.linear_acceleration.y ** 2
            + message.linear_acceleration.z ** 2
        )
        for message in messages
    ]
    gravity_mean = sum(accelerations) / len(accelerations)
    if not 9.3 <= gravity_mean <= 10.3:
        failures.append(f'mean gravity magnitude is {gravity_mean:.3f} m/s^2')

    for label, values in (
        ('angular velocity', messages[-1].angular_velocity_covariance),
        ('linear acceleration', messages[-1].linear_acceleration_covariance),
    ):
        if any(values[index] <= 0.0 for index in (0, 4, 8)):
            failures.append(f'{label} covariance diagonal is not positive')
    if any(message.header.frame_id != 'imu_link' for message in messages):
        failures.append('frame_id is not consistently imu_link')

    rate = (len(messages) - 1) / (received[-1] - received[0])
    print(f'Messages: {len(messages)}')
    print(f'Rate: {rate:.2f} Hz')
    print(f'Max timestamp gap: {max(stamp_gaps):.4f} s')
    print(f'Max receive gap: {max(receive_gaps):.4f} s')
    print(f'Max timestamp age: {max(timestamp_ages):.4f} s')
    print(f'Mean gravity magnitude: {gravity_mean:.4f} m/s^2')
    return failures


def main(args=None):
    arguments = parse_arguments(args)
    rclpy.init()
    node = ImuValidator(arguments.topic)
    deadline = time.monotonic() + arguments.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        failures = validate(
            node.messages,
            node.received_monotonic,
            node.received_ros_ns,
            arguments.duration,
        )
        if failures:
            print('IMU validation: FAILED')
            for failure in failures:
                print(f'  - {failure}')
            raise SystemExit(1)
        print('IMU validation: PASSED')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
