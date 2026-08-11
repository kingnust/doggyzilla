"""Unit tests for synchronized visual shadow runtime checks."""

from dataclasses import replace

from dogzilla_slam.shadow_validate import InfoSample
from dogzilla_slam.shadow_validate import nearest_errors
from dogzilla_slam.shadow_validate import OdomSample
from dogzilla_slam.shadow_validate import ScanSample
from dogzilla_slam.shadow_validate import TimedSample
from dogzilla_slam.shadow_validate import validate


def _stream(sample_type, rate, duration, **fields):
    count = int(rate * duration)
    base_stamp = 1_000_000_000_000
    samples = []
    for index in range(count):
        stamp = base_stamp + int(index / rate * 1e9)
        common = {
            'stamp_ns': stamp,
            'received_monotonic': 100.0 + index / rate,
            'received_ros_ns': stamp + 20_000_000,
        }
        samples.append(sample_type(**common, **fields))
    return samples


def _healthy_streams(duration=6.0):
    images = _stream(TimedSample, 20.0, duration)
    scans = _stream(
        ScanSample,
        10.0,
        duration,
        frame_id='laser_frame',
        finite_ranges=300,
        total_ranges=360,
        metadata_valid=True,
    )
    odometry = _stream(
        OdomSample,
        20.0,
        duration,
        frame_id='odom',
        child_frame_id='base_link',
        pose_valid=True,
        covariance_valid=True,
    )
    info = _stream(
        InfoSample,
        1.0,
        duration,
        ref_id=1,
        loop_closure_id=0,
        working_memory_size=4,
    )
    return images, scans, odometry, info


def test_healthy_synchronized_shadow_stream_passes():
    assert validate(*_healthy_streams(), duration=6.0) == []


def test_stale_scan_timestamps_are_rejected():
    images, scans, odometry, info = _healthy_streams()
    scans = [
        replace(sample, received_ros_ns=sample.stamp_ns + 900_000_000)
        for sample in scans
    ]
    failures = validate(images, scans, odometry, info, duration=6.0)
    assert any('Scans 95th percentile timestamp age' in item for item in failures)


def test_missing_rtab_activity_is_rejected():
    images, scans, odometry, _ = _healthy_streams()
    failures = validate(images, scans, odometry, [], duration=6.0)
    assert 'RTAB produced no Info messages' in failures


def test_bad_lidar_and_odometry_metadata_are_rejected():
    images, scans, odometry, info = _healthy_streams()
    scans = [replace(sample, finite_ranges=0) for sample in scans]
    odometry = [
        replace(sample, covariance_valid=False) for sample in odometry
    ]
    failures = validate(images, scans, odometry, info, duration=6.0)
    assert 'fewer than 1% of LiDAR ranges are usable' in failures
    assert 'odometry covariance is missing or non-positive' in failures


def test_nearest_timestamp_errors_select_both_sides():
    assert nearest_errors(
        [1_000_000_000, 2_000_000_000],
        [900_000_000, 2_200_000_000],
    ) == [0.1, 0.2]
