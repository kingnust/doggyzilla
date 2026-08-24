import ast
import json
import math
import os
from pathlib import Path

from dogzilla_slam.navigation_tuning_recorder import BoundedTrialWriter
from dogzilla_slam.navigation_tuning_recorder import closest_path_error
from dogzilla_slam.navigation_tuning_recorder import laser_summary
from dogzilla_slam.navigation_tuning_recorder import load_tuning_profile
from dogzilla_slam.navigation_tuning_recorder import NavigationTuningMetrics
from dogzilla_slam.navigation_tuning_recorder import prune_artifacts


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _sample(elapsed, angular, measured_linear=0.0, pose_x=0.0):
    return {
        'elapsed_s': elapsed,
        'commands': {
            'raw': {'linear_x': 0.06, 'angular_z': angular},
            'smoothed': {'linear_x': 0.05, 'angular_z': angular * 0.8},
            'final': {'linear_x': 0.05, 'angular_z': angular * 0.8},
        },
        'measured': {
            'linear_x': measured_linear,
            'angular_z': angular * 0.4,
        },
        'tracking': {
            'global': {'cross_track_m': 0.08, 'heading_error_rad': 0.2},
            'local': {'cross_track_m': 0.04, 'heading_error_rad': 0.1},
        },
        'lidar': {
            'sectors': {
                'front': {'minimum_m': 0.31, 'p10_m': 0.45},
            },
        },
        'pose_map': {'x': pose_x, 'y': 0.0, 'yaw': 0.0},
        'ages_s': {
            name: 0.01 for name in (
                'raw_command',
                'smoothed_command',
                'final_command',
                'odometry',
                'scan',
                'map_tf',
                'global_path',
                'local_path',
            )
        },
    }


def test_path_error_uses_nearest_segment_and_heading():
    result = closest_path_error(
        1.0,
        0.25,
        0.1,
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
    )

    assert result['cross_track_m'] == 0.25
    assert result['heading_error_rad'] == 0.1
    assert result['nearest'] == {'x': 1.0, 'y': 0.0}


def test_laser_summary_keeps_clearance_and_quality_not_raw_scan():
    ranges = [0.4, 1.0, float('inf'), 2.0, 0.5, 0.05, 3.0, 0.8]
    result = laser_summary(
        -math.pi,
        math.pi / 4.0,
        ranges,
        0.1,
        6.0,
    )

    assert result['beam_count'] == 8
    assert result['valid_count'] == 6
    assert result['valid_fraction'] == 0.75
    assert result['sectors']['front']['minimum_m'] == 0.5
    assert 'ranges' not in result


def test_metrics_capture_oscillation_tracking_stall_and_clearance():
    metrics = NavigationTuningMetrics()
    metrics.observe(_sample(0.0, 0.12))
    metrics.observe(_sample(0.1, -0.12))
    metrics.observe(_sample(0.2, 0.12, pose_x=0.2))
    metrics.marker()

    summary = metrics.summary('aborted', 'a' * 32, {'profile': 'test'})
    assert summary['outcome'] == 'aborted'
    assert summary['sample_count'] == 3
    assert summary['turn_reversals'] == {
        'raw': 2,
        'smoothed': 2,
        'final': 2,
    }
    assert summary['stalled_seconds'] == 0.2
    assert summary['operator_markers'] == 1
    assert summary['metrics']['local_cross_track_m']['p95'] == 0.04
    assert summary['metrics']['front_minimum_m']['minimum'] == 0.31
    assert summary['control_action'] == 'none'


def test_writer_is_bounded_and_retention_deletes_only_exact_artifacts(tmp_path):
    root = tmp_path / 'navigation-tuning'
    unrelated = root / 'notes.txt'
    for index in range(4):
        writer = BoundedTrialWriter(
            root,
            f'20260824T12000{index}Z_AABBCCDD',
            maximum_bytes=64 * 1024,
        )
        for sequence in range(100):
            writer.write({'sequence': sequence, 'payload': 'x' * 1024})
        writer.finish({'kind': 'summary', 'index': index})
        os.utime(writer.summary_path, (index + 1, index + 1))
    unrelated.write_text('keep me', encoding='utf-8')

    removed = prune_artifacts(root, retain=2)
    assert len(list(root.glob('*.summary.json'))) == 2
    assert len(list(root.glob('*.jsonl'))) == 2
    assert unrelated.read_text(encoding='utf-8') == 'keep me'
    assert len(removed) == 4
    for path in root.glob('*.jsonl'):
        assert path.stat().st_size <= 64 * 1024
        for line in path.read_text(encoding='utf-8').splitlines():
            json.loads(line)


def test_profile_contains_only_controller_planner_and_costmap_sections():
    profile = load_tuning_profile(PACKAGE_ROOT / 'config' / 'nav2_test1.yaml')

    assert len(profile['sha256']) == 64
    assert set(profile['sections']) == {
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'local_costmap',
        'global_costmap',
    }
    assert 'bt_navigator' not in profile['sections']


def test_recorder_is_read_only_and_launch_guarded():
    source_path = (
        PACKAGE_ROOT / 'dogzilla_slam' / 'navigation_tuning_recorder.py'
    )
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
    assert "'/navigation/tuning/status'" in source
    assert "'/navigation/tuning/marker'" in source
    identifiers = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    }
    identifiers.update(
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    )
    for forbidden in (
        'ActionClient',
        'NavigateToPose',
        'SetParameters',
        'CompressedImage',
        'OccupancyGrid',
        'JointState',
    ):
        assert forbidden not in identifiers

    launch = (
        PACKAGE_ROOT / 'launch' / 'full_navigation.launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='navigation_tuning_recorder'" in launch
    assert 'condition=IfCondition(use_nav2)' in launch
