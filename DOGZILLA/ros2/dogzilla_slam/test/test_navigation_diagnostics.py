import ast
import json
from pathlib import Path

from dogzilla_slam.navigation_diagnostics import BoundedJsonlRecorder
from dogzilla_slam.navigation_diagnostics import NavigationWarningTracker


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _healthy_sources(tracker, now):
    tracker.observe_scan(now, 0.01)
    tracker.observe_odometry(now, 0.0, 0.0, 0.0, 0.01)
    tracker.observe_tf(now, 0.01)


def test_stale_data_requires_persistence_and_stable_recovery():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=1.0,
        warning_persistence_seconds=1.0,
        recovery_seconds=2.0,
        scan_timeout_seconds=0.5,
        odom_timeout_seconds=0.5,
        tf_timeout_seconds=0.5,
    )
    _healthy_sources(tracker, 0.5)
    assert tracker.evaluate(0.5)['state'] == 'healthy'

    assert tracker.evaluate(1.1)['state'] == 'healthy'
    warning = tracker.evaluate(2.2)
    assert warning['state'] == 'warning'
    assert {item['code'] for item in warning['warnings']} >= {
        'scan_stale',
        'odom_stale',
        'tf_stale',
    }

    _healthy_sources(tracker, 2.3)
    assert tracker.evaluate(2.3)['state'] == 'warning'
    _healthy_sources(tracker, 4.4)
    assert tracker.evaluate(4.4)['state'] == 'healthy'


def test_turn_reversals_warn_only_after_repeated_pattern():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=0.1,
        warning_persistence_seconds=0.5,
        recovery_seconds=1.0,
        oscillation_count=5,
    )
    for index, angular in enumerate((0.1, -0.1, 0.1, -0.1, 0.1, -0.1)):
        now = 0.2 + index * 0.2
        _healthy_sources(tracker, now)
        tracker.observe_command(now, 0.05, angular)
    assert tracker.evaluate(1.2)['state'] == 'healthy'
    _healthy_sources(tracker, 1.8)
    result = tracker.evaluate(1.8)
    assert result['state'] == 'warning'
    assert result['warnings'][0]['code'] == 'angular_oscillation'


def test_linear_stall_requires_sustained_fresh_command_and_no_progress():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=0.1,
        warning_persistence_seconds=0.5,
        recovery_seconds=1.0,
        stall_window_seconds=1.0,
        stall_command_max_age_seconds=0.25,
    )
    result = None
    for index in range(17):
        now = 0.2 + index * 0.1
        tracker.observe_scan(now, 0.01)
        tracker.observe_tf(now, 0.01)
        tracker.observe_command(now, 0.06, 0.0)
        tracker.observe_odometry(
            now,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
        )
        result = tracker.evaluate(now)

    assert result['state'] == 'warning'
    assert [item['code'] for item in result['warnings']] == [
        'linear_stall_suspected'
    ]
    evidence = result['metrics']['stall_evidence']
    assert evidence['ready'] is True
    assert evidence['linear_candidate'] is True
    assert evidence['measured_displacement_m'] == 0.0
    assert result['metrics']['command']['linear_mps'] == 0.06


def test_stall_warning_recovers_only_after_stable_measured_progress():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=0.1,
        warning_persistence_seconds=0.4,
        recovery_seconds=0.8,
        stall_window_seconds=0.8,
        stall_command_max_age_seconds=0.25,
    )
    for index in range(14):
        now = 0.2 + index * 0.1
        tracker.observe_scan(now, 0.01)
        tracker.observe_tf(now, 0.01)
        tracker.observe_command(now, 0.06, 0.0)
        tracker.observe_odometry(
            now, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0
        )
        warning = tracker.evaluate(now)
    assert warning['state'] == 'warning'

    result = warning
    for index in range(16):
        now = 1.6 + index * 0.1
        distance = (index + 1) * 0.008
        tracker.observe_scan(now, 0.01)
        tracker.observe_tf(now, 0.01)
        tracker.observe_command(now, 0.06, 0.0)
        tracker.observe_odometry(
            now, distance, 0.0, 0.0, 0.01, 0.06, 0.0
        )
        result = tracker.evaluate(now)

    assert result['state'] == 'healthy'
    assert result['warnings'] == []
    assert result['metrics']['stall_evidence']['linear_candidate'] is False


def test_turn_stall_requires_consistent_turn_only_command():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=0.1,
        warning_persistence_seconds=0.4,
        recovery_seconds=1.0,
        stall_window_seconds=0.8,
        stall_command_max_age_seconds=0.25,
    )
    result = None
    for index in range(14):
        now = 0.2 + index * 0.1
        tracker.observe_scan(now, 0.01)
        tracker.observe_tf(now, 0.01)
        tracker.observe_command(now, 0.0, 0.15)
        tracker.observe_odometry(
            now, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0
        )
        result = tracker.evaluate(now)

    assert result['state'] == 'warning'
    assert [item['code'] for item in result['warnings']] == [
        'turn_stall_suspected'
    ]
    assert result['metrics']['stall_evidence']['turn_candidate'] is True


def test_short_pause_and_stale_command_do_not_create_stall_warning():
    tracker = NavigationWarningTracker(
        started_at=0.0,
        startup_grace_seconds=0.1,
        warning_persistence_seconds=0.4,
        recovery_seconds=1.0,
        stall_window_seconds=1.0,
        stall_command_max_age_seconds=0.25,
    )
    for index in range(7):
        now = 0.2 + index * 0.1
        tracker.observe_scan(now, 0.01)
        tracker.observe_tf(now, 0.01)
        tracker.observe_command(now, 0.06, 0.0)
        tracker.observe_odometry(
            now, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0
        )
        result = tracker.evaluate(now)
    assert result['state'] == 'healthy'

    now = 1.2
    tracker.observe_scan(now, 0.01)
    tracker.observe_tf(now, 0.01)
    tracker.observe_odometry(now, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0)
    result = tracker.evaluate(now)
    assert result['state'] == 'healthy'
    assert result['metrics']['stall_evidence']['ready'] is False


def test_bounded_jsonl_recorder_rotates_and_keeps_valid_records(tmp_path):
    recorder = BoundedJsonlRecorder(
        tmp_path / 'navigation.jsonl',
        maximum_bytes=4096,
    )
    for sequence in range(100):
        recorder.write({'sequence': sequence, 'detail': 'x' * 80})

    assert recorder.path.stat().st_size <= 4096
    assert recorder.previous_path.stat().st_size <= 4096
    for path in (recorder.previous_path, recorder.path):
        for line in path.read_text(encoding='utf-8').splitlines():
            assert isinstance(json.loads(line)['sequence'], int)


def test_monitor_has_no_robot_control_publishers_and_is_launch_guarded():
    source_path = PACKAGE_ROOT / 'dogzilla_slam' / 'navigation_diagnostics.py'
    source = source_path.read_text(encoding='utf-8')
    tree = ast.parse(source)
    publisher_types = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == 'create_publisher'
            and node.args
            and isinstance(node.args[0], ast.Name)
        ):
            publisher_types.append(node.args[0].id)

    assert publisher_types == ['String']
    assert "'/navigation/diagnostics'" in source
    for forbidden in ('NavigateToPose', 'SetParameters', '/safety/estop'):
        assert forbidden not in source

    launch = (
        PACKAGE_ROOT / 'launch' / 'full_navigation.launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='navigation_diagnostics'" in launch
    assert 'condition=IfCondition(use_nav2)' in launch
