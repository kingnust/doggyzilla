"""Runtime checks for the DOGZILLA monocular ROS camera stream."""

import argparse
from dataclasses import dataclass
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from dogzilla_slam.camera_model import CameraModelError
from dogzilla_slam.camera_model import validate_intrinsics
from dogzilla_slam.validation_report import make_validation_report
from dogzilla_slam.validation_report import write_json_report


@dataclass(frozen=True)
class ImageSample:
    stamp_ns: int
    width: int
    height: int
    frame_id: str
    encoding: str


class CameraValidator(Node):
    def __init__(self, image_topic, camera_info_topic):
        super().__init__('dogzilla_camera_validator')
        self.images = []
        self.received_monotonic = []
        self.received_ros_ns = []
        self.camera_info = []
        self.create_subscription(
            Image,
            image_topic,
            self._receive_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info.append,
            qos_profile_sensor_data,
        )

    def _receive_image(self, message):
        self.images.append(ImageSample(
            stamp_ns=stamp_nanoseconds(message),
            width=message.width,
            height=message.height,
            frame_id=message.header.frame_id,
            encoding=message.encoding,
        ))
        self.received_monotonic.append(time.monotonic())
        self.received_ros_ns.append(self.get_clock().now().nanoseconds)


def stamp_nanoseconds(message):
    return (
        message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec
    )


def _matrix_matches(actual, expected, label):
    if len(actual) != len(expected) or any(
        not math.isclose(float(value), float(reference), abs_tol=1e-6)
        for value, reference in zip(actual, expected)
    ):
        return [f'CameraInfo {label} does not match camera.yaml']
    return []


def compare_camera_info(message, intrinsics):
    """Compare live CameraInfo against the validated on-disk calibration."""
    failures = []
    if message.distortion_model != intrinsics['distortion_model']:
        failures.append(
            'CameraInfo distortion model does not match camera.yaml'
        )
    failures.extend(_matrix_matches(
        message.k,
        intrinsics['camera_matrix']['data'],
        'K matrix',
    ))
    failures.extend(_matrix_matches(
        message.d,
        intrinsics['distortion_coefficients']['data'],
        'distortion coefficients',
    ))
    failures.extend(_matrix_matches(
        message.r,
        intrinsics['rectification_matrix']['data'],
        'R matrix',
    ))
    failures.extend(_matrix_matches(
        message.p,
        intrinsics['projection_matrix']['data'],
        'P matrix',
    ))
    return failures


def validate(
    images,
    camera_info,
    received,
    received_ros_ns,
    duration,
    require_calibration,
    expected_intrinsics=None,
):
    failures = []
    minimum_messages = max(10, int(duration * 8))
    if len(images) < minimum_messages:
        failures.append(
            f'too few images ({len(images)}; need {minimum_messages})'
        )
    if len(images) < 2:
        return failures or ['fewer than two images']

    stamps = [sample.stamp_ns for sample in images]
    if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
        failures.append('image timestamps are not strictly monotonic')
    ages = [
        (received_stamp - message_stamp) / 1e9
        for received_stamp, message_stamp in zip(received_ros_ns, stamps)
    ]
    stamp_gaps = [
        (later - earlier) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
    ]
    receive_gaps = [
        later - earlier for earlier, later in zip(received, received[1:])
    ]
    sorted_ages = sorted(ages)
    age_mean = sum(ages) / len(ages)
    age_std = math.sqrt(
        sum((age - age_mean) ** 2 for age in ages) / len(ages)
    )
    age_p95_index = min(len(sorted_ages) - 1, int(len(ages) * 0.95))
    age_p95 = sorted_ages[age_p95_index]
    age_drift = ages[-1] - ages[0]
    if max(ages) > 1.0:
        failures.append(f'largest image timestamp age is {max(ages):.3f}s')
    if age_p95 > 0.9:
        failures.append(f'95th percentile timestamp age is {age_p95:.3f}s')
    if age_mean > 0.8:
        failures.append(f'mean image timestamp age is {age_mean:.3f}s')
    if age_std > 0.15:
        failures.append(f'image timestamp age jitter is {age_std:.3f}s')
    if min(ages) < -0.05:
        failures.append(
            f'an image timestamp is {-min(ages):.3f}s in the future'
        )
    if max(stamp_gaps) > 0.5:
        failures.append(
            f'largest image timestamp gap is {max(stamp_gaps):.3f}s'
        )
    if max(receive_gaps) > 0.5:
        failures.append(
            f'largest image receive gap is {max(receive_gaps):.3f}s'
        )
    if any(
        (sample.width, sample.height) != (640, 480)
        for sample in images
    ):
        failures.append('image size is not consistently 640x480')
    if any(
        sample.frame_id != 'camera_optical_frame' for sample in images
    ):
        failures.append(
            'image frame_id is not consistently camera_optical_frame'
        )
    if any(not sample.encoding for sample in images):
        failures.append('an image has an empty encoding')

    if not camera_info:
        failures.append('no CameraInfo messages received')
    else:
        latest = camera_info[-1]
        if (latest.width, latest.height) != (640, 480):
            failures.append('CameraInfo size is not 640x480')
        if latest.header.frame_id != 'camera_optical_frame':
            failures.append('CameraInfo frame_id is not camera_optical_frame')
        if require_calibration and not (
            latest.k[0] > 0.0
            and latest.k[4] > 0.0
            and latest.p[0] > 0.0
            and latest.p[5] > 0.0
        ):
            failures.append('CameraInfo has no usable intrinsic calibration')
        if expected_intrinsics is not None:
            failures.extend(compare_camera_info(latest, expected_intrinsics))

    rate = (len(images) - 1) / (received[-1] - received[0])
    if rate < 10.0:
        failures.append(f'image rate is only {rate:.2f} Hz')
    print(f'Images: {len(images)}')
    print(f'Rate: {rate:.2f} Hz')
    print(f'Max timestamp gap: {max(stamp_gaps):.4f} s')
    print(f'Max receive gap: {max(receive_gaps):.4f} s')
    print(f'Max timestamp age: {max(ages):.4f} s')
    print(f'Min timestamp age: {min(ages):.4f} s')
    print(f'95th percentile timestamp age: {age_p95:.4f} s')
    print(f'Mean timestamp age: {age_mean:.4f} s')
    print(f'Timestamp age jitter: {age_std:.4f} s')
    print(f'Timestamp age drift: {age_drift:+.4f} s')
    print(f'CameraInfo messages: {len(camera_info)}')
    return failures


def report_measurements(node):
    measurements = {
        'image_messages': len(node.images),
        'camera_info_messages': len(node.camera_info),
    }
    if node.images:
        measurements.update({
            'image_width': node.images[-1].width,
            'image_height': node.images[-1].height,
            'image_frame_id': node.images[-1].frame_id,
            'image_encoding': node.images[-1].encoding,
        })
    if len(node.images) >= 2:
        elapsed = (
            node.received_monotonic[-1] - node.received_monotonic[0]
        )
        stamps = [sample.stamp_ns for sample in node.images]
        ages = [
            (received_stamp - message_stamp) / 1e9
            for received_stamp, message_stamp in zip(
                node.received_ros_ns,
                stamps,
            )
        ]
        measurements.update({
            'rate_hz': (
                (len(node.images) - 1) / elapsed if elapsed > 0.0 else 0.0
            ),
            'maximum_timestamp_age_seconds': max(ages),
            'mean_timestamp_age_seconds': sum(ages) / len(ages),
            'maximum_timestamp_gap_seconds': max(
                (later - earlier) / 1e9
                for earlier, later in zip(stamps, stamps[1:])
            ),
        })
    if node.camera_info:
        latest = node.camera_info[-1]
        measurements['camera_info_has_intrinsics'] = bool(
            latest.k[0] > 0.0
            and latest.k[4] > 0.0
            and latest.p[0] > 0.0
            and latest.p[5] > 0.0
        )
    return measurements


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--require-calibration', action='store_true')
    parser.add_argument('--intrinsics')
    parser.add_argument('--report-json')
    arguments = parser.parse_args(args)
    if arguments.duration <= 0.0:
        parser.error('--duration must be positive')
    return arguments


def main(args=None):
    arguments = parse_arguments(args)
    expected_intrinsics = None
    if arguments.intrinsics:
        try:
            expected_intrinsics = validate_intrinsics(arguments.intrinsics)
        except CameraModelError as exc:
            failure = f'Camera validation setup failed: {exc}'
            if arguments.report_json:
                write_json_report(
                    arguments.report_json,
                    make_validation_report(
                        'camera',
                        {
                            'duration_seconds': arguments.duration,
                            'calibration_required': (
                                arguments.require_calibration
                            ),
                            'intrinsics_file': arguments.intrinsics,
                        },
                        {
                            'image_messages': 0,
                            'camera_info_messages': 0,
                        },
                        [failure],
                    ),
                )
            raise SystemExit(
                failure
            ) from exc
    rclpy.init()
    node = CameraValidator(
        arguments.image_topic,
        arguments.camera_info_topic,
    )
    deadline = time.monotonic() + arguments.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        failures = validate(
            node.images,
            node.camera_info,
            node.received_monotonic,
            node.received_ros_ns,
            arguments.duration,
            arguments.require_calibration,
            expected_intrinsics,
        )
        if arguments.report_json:
            write_json_report(
                arguments.report_json,
                make_validation_report(
                    'camera',
                    {
                        'duration_seconds': arguments.duration,
                        'calibration_required': (
                            arguments.require_calibration
                        ),
                        'intrinsics_file': arguments.intrinsics,
                    },
                    report_measurements(node),
                    failures,
                ),
            )
            print(f'Camera report: {arguments.report_json}')
        if failures:
            print('Camera validation: FAILED')
            for failure in failures:
                print(f'  - {failure}')
            raise SystemExit(1)
        print('Camera validation: PASSED')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
