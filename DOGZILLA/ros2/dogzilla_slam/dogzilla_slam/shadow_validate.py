"""Validate synchronized inputs and processing in visual shadow mode."""

import argparse
from bisect import bisect_left
from dataclasses import dataclass
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class TimedSample:
    stamp_ns: int
    received_monotonic: float
    received_ros_ns: int


@dataclass(frozen=True)
class ScanSample(TimedSample):
    frame_id: str
    finite_ranges: int
    total_ranges: int
    metadata_valid: bool


@dataclass(frozen=True)
class OdomSample(TimedSample):
    frame_id: str
    child_frame_id: str
    pose_valid: bool
    covariance_valid: bool


@dataclass(frozen=True)
class InfoSample(TimedSample):
    ref_id: int
    loop_closure_id: int
    working_memory_size: int


def stamp_nanoseconds(message):
    return (
        message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec
    )


def percentile(values, fraction):
    if not values:
        raise ValueError('cannot calculate a percentile of no values')
    ordered = sorted(values)
    index = max(0, math.ceil(len(ordered) * fraction) - 1)
    return ordered[index]


def nearest_errors(reference_stamps, candidate_stamps):
    """Return nearest absolute timestamp differences in seconds."""
    candidates = sorted(candidate_stamps)
    if not candidates:
        return []
    errors = []
    for stamp in reference_stamps:
        index = bisect_left(candidates, stamp)
        nearby = []
        if index < len(candidates):
            nearby.append(candidates[index])
        if index:
            nearby.append(candidates[index - 1])
        errors.append(min(abs(stamp - other) for other in nearby) / 1e9)
    return errors


class ShadowValidator(Node):
    def __init__(self, image_topic, scan_topic, odom_topic, info_topic):
        super().__init__('dogzilla_shadow_validator')
        self.images = []
        self.scans = []
        self.odometry = []
        self.info = []
        self.create_subscription(
            Image,
            image_topic,
            self._receive_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._receive_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            odom_topic,
            self._receive_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Info,
            info_topic,
            self._receive_info,
            qos_profile_sensor_data,
        )

    def _receipt(self):
        return time.monotonic(), self.get_clock().now().nanoseconds

    def _receive_image(self, message):
        received, ros_ns = self._receipt()
        self.images.append(TimedSample(
            stamp_nanoseconds(message),
            received,
            ros_ns,
        ))

    def _receive_scan(self, message):
        received, ros_ns = self._receipt()
        finite_ranges = sum(
            math.isfinite(value)
            and message.range_min <= value <= message.range_max
            for value in message.ranges
        )
        metadata_valid = (
            message.angle_increment != 0.0
            and message.range_min > 0.0
            and message.range_max > message.range_min
            and bool(message.ranges)
        )
        self.scans.append(ScanSample(
            stamp_nanoseconds(message),
            received,
            ros_ns,
            message.header.frame_id,
            finite_ranges,
            len(message.ranges),
            metadata_valid,
        ))

    def _receive_odom(self, message):
        received, ros_ns = self._receipt()
        position = message.pose.pose.position
        rotation = message.pose.pose.orientation
        quaternion_norm = math.sqrt(
            rotation.x ** 2
            + rotation.y ** 2
            + rotation.z ** 2
            + rotation.w ** 2
        )
        pose_valid = all(math.isfinite(value) for value in (
            position.x,
            position.y,
            position.z,
            quaternion_norm,
        )) and math.isclose(quaternion_norm, 1.0, abs_tol=0.02)
        pose_covariance = message.pose.covariance
        twist_covariance = message.twist.covariance
        covariance_valid = all(
            value > 0.0
            for value in (
                pose_covariance[0],
                pose_covariance[7],
                pose_covariance[35],
                twist_covariance[0],
                twist_covariance[35],
            )
        )
        self.odometry.append(OdomSample(
            stamp_nanoseconds(message),
            received,
            ros_ns,
            message.header.frame_id,
            message.child_frame_id,
            pose_valid,
            covariance_valid,
        ))

    def _receive_info(self, message):
        received, ros_ns = self._receipt()
        self.info.append(InfoSample(
            stamp_nanoseconds(message),
            received,
            ros_ns,
            message.ref_id,
            message.loop_closure_id,
            len(message.wm_state),
        ))


def _validate_timing(
    label,
    samples,
    duration,
    minimum_rate,
    maximum_gap,
    maximum_p95_age,
    allow_equal_stamps=False,
):
    failures = []
    minimum_count = max(2, int(duration * minimum_rate))
    if len(samples) < minimum_count:
        failures.append(
            f'{label} has too few messages '
            f'({len(samples)}; need {minimum_count})'
        )
    if len(samples) < 2:
        print(f'{label}: {len(samples)} messages')
        return failures

    stamps = [sample.stamp_ns for sample in samples]
    if any(stamp <= 0 for stamp in stamps):
        failures.append(f'{label} has a zero or negative timestamp')
    if allow_equal_stamps:
        timestamps_bad = any(
            later < earlier for earlier, later in zip(stamps, stamps[1:])
        )
    else:
        timestamps_bad = any(
            later <= earlier for earlier, later in zip(stamps, stamps[1:])
        )
    if timestamps_bad:
        qualifier = 'nondecreasing' if allow_equal_stamps else 'increasing'
        failures.append(f'{label} timestamps are not {qualifier}')

    receive_gaps = [
        later.received_monotonic - earlier.received_monotonic
        for earlier, later in zip(samples, samples[1:])
    ]
    stamp_gaps = [
        (later - earlier) / 1e9
        for earlier, later in zip(stamps, stamps[1:])
    ]
    ages = [
        (sample.received_ros_ns - sample.stamp_ns) / 1e9
        for sample in samples
    ]
    elapsed = samples[-1].received_monotonic - samples[0].received_monotonic
    rate = (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0
    age_p95 = percentile(ages, 0.95)
    if rate < minimum_rate:
        failures.append(f'{label} rate is only {rate:.2f} Hz')
    if max(receive_gaps) > maximum_gap:
        failures.append(
            f'{label} receive gap reached {max(receive_gaps):.3f}s'
        )
    if max(stamp_gaps) > maximum_gap:
        failures.append(
            f'{label} timestamp gap reached {max(stamp_gaps):.3f}s'
        )
    if age_p95 > maximum_p95_age:
        failures.append(
            f'{label} 95th percentile timestamp age is {age_p95:.3f}s'
        )
    if min(ages) < -0.05:
        failures.append(
            f'{label} timestamp is {-min(ages):.3f}s in the future'
        )

    print(
        f'{label}: {len(samples)} messages, {rate:.2f} Hz, '
        f'p95 age {age_p95:.4f} s, max gap {max(receive_gaps):.4f} s'
    )
    return failures


def _validate_alignment(label, reference_stamps, candidate_stamps):
    errors = nearest_errors(reference_stamps, candidate_stamps)
    if not errors:
        return [f'{label} alignment has no comparable timestamps']
    p95_error = percentile(errors, 0.95)
    max_error = max(errors)
    print(
        f'{label} alignment: p95 {p95_error:.4f} s, '
        f'max {max_error:.4f} s'
    )
    failures = []
    if p95_error > 0.10:
        failures.append(f'{label} p95 alignment error is {p95_error:.3f}s')
    if max_error > 0.25:
        failures.append(f'{label} max alignment error is {max_error:.3f}s')
    return failures


def validate(images, scans, odometry, info, duration):
    failures = []
    failures.extend(_validate_timing(
        'Images', images, duration, 10.0, 0.50, 0.25
    ))
    failures.extend(_validate_timing(
        'Scans', scans, duration, 5.0, 0.50, 0.35
    ))
    failures.extend(_validate_timing(
        'Odometry',
        odometry,
        duration,
        5.0,
        0.50,
        0.35,
        allow_equal_stamps=True,
    ))
    failures.extend(_validate_timing(
        'RTAB Info',
        info,
        duration,
        0.2,
        5.0,
        0.75,
        allow_equal_stamps=True,
    ))

    if scans:
        if any(not sample.metadata_valid for sample in scans):
            failures.append('a scan has invalid angle or range metadata')
        if any(sample.frame_id != 'laser_frame' for sample in scans):
            failures.append('scan frame_id is not consistently laser_frame')
        finite_ranges = sum(sample.finite_ranges for sample in scans)
        total_ranges = sum(sample.total_ranges for sample in scans)
        finite_ratio = finite_ranges / total_ranges if total_ranges else 0.0
        print(f'Finite LiDAR returns: {finite_ratio * 100.0:.1f}%')
        if finite_ratio < 0.01:
            failures.append('fewer than 1% of LiDAR ranges are usable')

    if odometry:
        if any(sample.frame_id != 'odom' for sample in odometry):
            failures.append('odometry frame_id is not consistently odom')
        if any(sample.child_frame_id != 'base_link' for sample in odometry):
            failures.append(
                'odometry child_frame_id is not consistently base_link'
            )
        if any(not sample.pose_valid for sample in odometry):
            failures.append('odometry contains an invalid pose or quaternion')
        if any(not sample.covariance_valid for sample in odometry):
            failures.append('odometry covariance is missing or non-positive')
        unique_stamps = len({sample.stamp_ns for sample in odometry})
        print(f'Unique odometry timestamps: {unique_stamps}')
        if unique_stamps < max(2, int(duration * 3.0)):
            failures.append('scan-matched odometry updates too slowly')

    if images and scans and odometry:
        common_start = max(
            images[0].stamp_ns,
            scans[0].stamp_ns,
            odometry[0].stamp_ns,
        )
        common_end = min(
            images[-1].stamp_ns,
            scans[-1].stamp_ns,
            odometry[-1].stamp_ns,
        )
        overlap = max(0.0, (common_end - common_start) / 1e9)
        print(f'Common sensor timestamp window: {overlap:.3f} s')
        reference_stamps = [
            sample.stamp_ns
            for sample in scans
            if common_start <= sample.stamp_ns <= common_end
        ]
        if overlap < max(1.0, duration * 0.5):
            failures.append('sensor timestamp windows overlap too little')
        failures.extend(_validate_alignment(
            'Scan-to-image',
            reference_stamps,
            [sample.stamp_ns for sample in images],
        ))
        failures.extend(_validate_alignment(
            'Scan-to-odometry',
            reference_stamps,
            [sample.stamp_ns for sample in odometry],
        ))

    if info:
        references = {sample.ref_id for sample in info if sample.ref_id >= 0}
        loop_closures = sum(
            sample.loop_closure_id > 0 for sample in info
        )
        maximum_memory = max(sample.working_memory_size for sample in info)
        print(f'RTAB reference IDs observed: {len(references)}')
        print(f'RTAB working-memory nodes: {maximum_memory}')
        print(f'RTAB loop closures observed: {loop_closures}')
        if not references:
            failures.append('RTAB reports no successfully processed node')
        if maximum_memory <= 0:
            failures.append('RTAB working memory is empty')
        if images:
            failures.extend(_validate_alignment(
                'RTAB-to-image',
                [sample.stamp_ns for sample in info],
                [sample.stamp_ns for sample in images],
            ))
    else:
        failures.append('RTAB produced no Info messages')

    return failures


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--image-topic', default='/camera/image_rect')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument(
        '--odom-topic',
        default='/rtabmap_shadow/odom_input',
    )
    parser.add_argument('--info-topic', default='/rtabmap_shadow/info')
    arguments = parser.parse_args(args)
    if arguments.duration <= 0.0:
        parser.error('--duration must be positive')
    return arguments


def main(args=None):
    arguments = parse_arguments(args)
    rclpy.init()
    node = ShadowValidator(
        arguments.image_topic,
        arguments.scan_topic,
        arguments.odom_topic,
        arguments.info_topic,
    )
    deadline = time.monotonic() + arguments.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        failures = validate(
            node.images,
            node.scans,
            node.odometry,
            node.info,
            arguments.duration,
        )
        if failures:
            print('Visual shadow validation: FAILED')
            for failure in failures:
                print(f'  - {failure}')
            raise SystemExit(1)
        print('Visual shadow validation: PASSED')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
