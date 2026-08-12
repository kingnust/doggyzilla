"""Unit tests for synchronized visual shadow runtime checks."""

from dataclasses import replace

from dogzilla_slam.shadow_validate import InfoSample
from dogzilla_slam.shadow_validate import nearest_errors
from dogzilla_slam.shadow_validate import OdomSample
from dogzilla_slam.shadow_validate import route_metrics
from dogzilla_slam.shadow_validate import ScanSample
from dogzilla_slam.shadow_validate import TimedSample
from dogzilla_slam.shadow_validate import validate
from dogzilla_slam.shadow_validate import validate_route


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


def test_latched_info_before_image_window_does_not_break_alignment():
    images, scans, odometry, info = _healthy_streams()
    latched = replace(
        info[0],
        stamp_ns=images[0].stamp_ns - 1_000_000_000,
        received_monotonic=info[0].received_monotonic - 1.0,
        received_ros_ns=images[0].stamp_ns - 980_000_000,
    )
    assert validate(
        images,
        scans,
        odometry,
        [latched, *info],
        duration=6.0,
    ) == []


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


def test_repeated_rtab_status_is_rejected():
    images, scans, odometry, info = _healthy_streams()
    stamp = info[0].stamp_ns
    info = [
        replace(
            sample,
            stamp_ns=stamp,
            received_ros_ns=stamp + 20_000_000,
        )
        for sample in info
    ]
    failures = validate(images, scans, odometry, info, duration=6.0)
    assert 'RTAB processing timestamp did not advance' in failures


def test_zero_rtab_reference_id_is_not_processed_work():
    images, scans, odometry, info = _healthy_streams()
    info = [replace(sample, ref_id=0) for sample in info]
    failures = validate(images, scans, odometry, info, duration=6.0)
    assert 'RTAB reports no successfully processed node' in failures


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


def _square_route(odometry):
    positions = []
    for start, end in (
        ((0.0, 0.0), (1.0, 0.0)),
        ((1.0, 0.0), (1.0, 1.0)),
        ((1.0, 1.0), (0.0, 1.0)),
        ((0.0, 1.0), (0.0, 0.0)),
    ):
        for index in range(5):
            fraction = index / 5.0
            positions.append((
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            ))
    positions.append((0.0, 0.0))
    positions.extend([positions[-1]] * (len(odometry) - len(positions)))
    return [
        replace(sample, x=position[0], y=position[1])
        for sample, position in zip(odometry, positions)
    ]


def test_route_metrics_ignore_duplicate_stamps_and_small_jitter():
    _, _, odometry, _ = _healthy_streams()
    samples = [
        replace(odometry[0], x=0.0, y=0.0),
        replace(odometry[0], x=0.5, y=0.0),
        replace(odometry[1], x=0.01, y=0.0),
        replace(odometry[2], x=0.03, y=0.0),
        replace(odometry[3], x=0.0, y=0.0),
    ]
    path_length, return_distance, maximum_step = route_metrics(samples)
    assert path_length == 0.06
    assert return_distance == 0.0
    assert maximum_step == 0.03


def test_completed_route_with_global_loop_closure_passes():
    _, _, odometry, info = _healthy_streams()
    odometry = _square_route(odometry)
    info[-1] = replace(
        info[-1],
        ref_id=12,
        loop_closure_id=2,
        proximity_detection_id=3,
    )
    assert validate_route(odometry, info, 3.5, 0.10) == []


def test_route_requires_travel_return_and_global_not_proximity_match():
    _, _, odometry, info = _healthy_streams()
    odometry = [replace(sample, x=0.0, y=0.0) for sample in odometry]
    odometry[-1] = replace(odometry[-1], x=1.0)
    info[-1] = replace(info[-1], proximity_detection_id=4)
    failures = validate_route(odometry, info, 2.0, 0.25)
    assert any('route travelled only' in failure for failure in failures)
    assert any('route ended' in failure for failure in failures)
    assert any('odometry jumped' in failure for failure in failures)
    assert 'RTAB detected no global loop closure on the route' in failures


def test_loop_closure_latched_before_route_does_not_satisfy_route():
    _, _, odometry, info = _healthy_streams()
    odometry = _square_route(odometry)
    stale_closure = replace(
        info[0],
        stamp_ns=odometry[0].stamp_ns - 1_000_000_000,
        ref_id=99,
        loop_closure_id=1,
    )
    failures = validate_route(
        odometry,
        [stale_closure, *info],
        3.5,
        0.10,
    )
    assert 'RTAB detected no global loop closure on the route' in failures
