"""Record only the synchronized evidence needed to tune DOGZILLA Nav2."""

from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as PathMessage
from rcl_interfaces.msg import ParameterEvent
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


SCHEMA_VERSION = 1
ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
TERMINAL_GOAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'cancelled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}
PARAMETER_NODES = {
    '/controller_server',
    '/planner_server',
    '/velocity_smoother',
    '/local_costmap/local_costmap',
    '/global_costmap/global_costmap',
    '/dogzilla_safe_base',
}
PARAMETER_PREFIXES = (
    'FollowPath.',
    'GridBased.',
    'general_goal_checker.',
    'progress_checker.',
)
PARAMETER_NAMES = {
    'controller_frequency',
    'failure_tolerance',
    'footprint',
    'footprint_padding',
    'max_accel',
    'max_decel',
    'max_velocity',
    'min_velocity',
    'speed_level',
    'turn_level',
    'velocity_timeout',
}


def utc_now():
    """Return a stable UTC timestamp for artifacts and status messages."""
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace(
        '+00:00', 'Z'
    )


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    """Return planar yaw from a geometry quaternion."""
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


def finite_round(value, digits=5):
    """Return a rounded finite float, or None for unavailable data."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return round(number, digits)


def percentile(values, quantile):
    """Return a linearly interpolated percentile for a finite sequence."""
    ordered = sorted(float(value) for value in values if math.isfinite(value))
    if not ordered:
        return None
    if len(ordered) == 1:
        return round(ordered[0], 5)
    position = max(0.0, min(1.0, float(quantile))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    value = ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
    return round(value, 5)


def closest_path_error(x, y, yaw, points):
    """Measure cross-track and heading error to the nearest path segment."""
    if len(points) < 2:
        return None
    best = None
    for first, second in zip(points, points[1:]):
        x1, y1 = first
        x2, y2 = second
        dx = x2 - x1
        dy = y2 - y1
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        projection = ((x - x1) * dx + (y - y1) * dy) / length_squared
        projection = max(0.0, min(1.0, projection))
        nearest_x = x1 + projection * dx
        nearest_y = y1 + projection * dy
        distance = math.hypot(x - nearest_x, y - nearest_y)
        if best is None or distance < best[0]:
            heading = math.atan2(dy, dx)
            best = (
                distance,
                abs(normalize_angle(yaw - heading)),
                nearest_x,
                nearest_y,
            )
    if best is None:
        return None
    return {
        'cross_track_m': round(best[0], 5),
        'heading_error_rad': round(best[1], 5),
        'nearest': {'x': round(best[2], 5), 'y': round(best[3], 5)},
    }


def _sector_name(angle):
    degrees = math.degrees(normalize_angle(angle))
    if -35.0 <= degrees <= 35.0:
        return 'front'
    if 35.0 < degrees < 145.0:
        return 'left'
    if -145.0 < degrees < -35.0:
        return 'right'
    return 'rear'


def laser_summary(angle_min, angle_increment, ranges, range_min, range_max):
    """Reduce a full scan to robust clearance and data-quality evidence."""
    sectors = {name: [] for name in ('front', 'left', 'right', 'rear')}
    valid_count = 0
    for index, raw_range in enumerate(ranges):
        value = float(raw_range)
        if (
            not math.isfinite(value)
            or value < float(range_min)
            or value > float(range_max)
        ):
            continue
        valid_count += 1
        angle = float(angle_min) + index * float(angle_increment)
        sectors[_sector_name(angle)].append(value)
    result = {
        'beam_count': len(ranges),
        'valid_count': valid_count,
        'valid_fraction': round(valid_count / max(1, len(ranges)), 5),
        'sectors': {},
    }
    for name, values in sectors.items():
        result['sectors'][name] = {
            'minimum_m': finite_round(min(values)) if values else None,
            'p10_m': percentile(values, 0.10),
            'median_m': percentile(values, 0.50),
            'count': len(values),
        }
    return result


def _path_points(message, maximum_points=120):
    points = [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in message.poses
        if math.isfinite(float(pose.pose.position.x))
        and math.isfinite(float(pose.pose.position.y))
    ]
    if len(points) <= maximum_points:
        return points
    stride = max(1, math.ceil((len(points) - 1) / (maximum_points - 1)))
    reduced = points[::stride]
    if reduced[-1] != points[-1]:
        reduced.append(points[-1])
    return reduced[:maximum_points]


def _goal_identifier(status):
    return bytes(status.goal_info.goal_id.uuid).hex()


def is_tuning_parameter(node_name, parameter_name):
    """Return whether one runtime parameter can affect navigation tuning."""
    if node_name not in PARAMETER_NODES:
        return False
    return (
        parameter_name in PARAMETER_NAMES
        or parameter_name.startswith(PARAMETER_PREFIXES)
        or parameter_name.endswith('.inflation_radius')
        or parameter_name.endswith('.cost_scaling_factor')
    )


def load_tuning_profile(path):
    """Load only Nav2 sections that materially affect planning and control."""
    source = Path(path)
    raw = source.read_bytes()
    document = yaml.safe_load(raw) or {}
    selected = {}
    for name in (
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'local_costmap',
        'global_costmap',
    ):
        value = document.get(name)
        if value is not None:
            selected[name] = value
    return {
        'source': str(source),
        'sha256': hashlib.sha256(raw).hexdigest(),
        'sections': selected,
    }


class BoundedTrialWriter:
    """Write one bounded JSONL trial and an atomic compact summary."""

    def __init__(self, root, name, maximum_bytes=8 * 1024 * 1024):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        if not self.root.is_dir():
            raise ValueError('navigation tuning root is not a directory')
        if not name or any(
            character not in '0123456789T_ZABCDEF' for character in name
        ):
            raise ValueError('navigation tuning artifact name is unsafe')
        self.data_path = (self.root / f'{name}.jsonl').resolve()
        self.summary_path = (self.root / f'{name}.summary.json').resolve()
        if self.data_path.parent != self.root:
            raise ValueError('navigation tuning artifact escaped its root')
        self.maximum_bytes = int(maximum_bytes)
        if self.maximum_bytes < 64 * 1024:
            raise ValueError('navigation tuning artifact limit is too small')
        self.bytes_written = 0
        self.records_written = 0
        self.truncated = False
        self.write_error = None
        self._stream = self.data_path.open('xb')

    def write(self, value):
        if self.truncated:
            return False
        encoded = (
            json.dumps(
                value,
                sort_keys=True,
                separators=(',', ':'),
                allow_nan=False,
            ).encode('utf-8')
            + b'\n'
        )
        if self.bytes_written + len(encoded) > self.maximum_bytes:
            self.truncated = True
            return False
        try:
            self._stream.write(encoded)
            self._stream.flush()
        except OSError as exc:
            self.write_error = str(exc)
            self.truncated = True
            self.close()
            return False
        self.bytes_written += len(encoded)
        self.records_written += 1
        return True

    def finish(self, summary):
        if not self._stream.closed:
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()
        complete = {
            **summary,
            'artifact': {
                'data': str(self.data_path),
                'summary': str(self.summary_path),
                'bytes': self.bytes_written,
                'records': self.records_written,
                'truncated': self.truncated,
                'maximum_bytes': self.maximum_bytes,
                'write_error': self.write_error,
            },
        }
        temporary = self.summary_path.with_suffix('.summary.json.tmp')
        temporary.write_text(
            json.dumps(complete, indent=2, sort_keys=True, allow_nan=False)
            + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, self.summary_path)
        return complete

    def close(self):
        if not self._stream.closed:
            self._stream.close()


def prune_artifacts(root, retain):
    """Keep a bounded number of completed trials in one exact directory."""
    directory = Path(root).resolve()
    if not directory.is_dir():
        return []
    summaries = sorted(
        (
            path for path in directory.glob('*.summary.json')
            if not path.is_symlink() and path.resolve().parent == directory
        ),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
        reverse=True,
    )
    removed = []
    for summary in summaries[max(1, int(retain)):]:
        stem = summary.name.removesuffix('.summary.json')
        data = directory / f'{stem}.jsonl'
        for path in (summary, data):
            if (
                path.exists()
                and not path.is_symlink()
                and path.resolve().parent == directory
            ):
                path.unlink()
                removed.append(path.name)
    return removed


class NavigationTuningMetrics:
    """Aggregate metrics that map directly to Nav2 tuning decisions."""

    def __init__(self):
        self.started_at = None
        self.finished_at = None
        self.sample_count = 0
        self.commanded_seconds = 0.0
        self.stalled_seconds = 0.0
        self.pose_jump_count = 0
        self.marker_count = 0
        self._previous_time = None
        self._previous_pose = None
        self._turn_signs = {'raw': 0, 'smoothed': 0, 'final': 0}
        self.turn_reversals = {'raw': 0, 'smoothed': 0, 'final': 0}
        self.series = {
            name: [] for name in (
                'global_cross_track_m',
                'local_cross_track_m',
                'global_heading_error_rad',
                'local_heading_error_rad',
                'raw_smoothed_linear_delta_mps',
                'raw_smoothed_angular_delta_rps',
                'smoothed_final_linear_delta_mps',
                'smoothed_final_angular_delta_rps',
                'final_measured_linear_error_mps',
                'final_measured_angular_error_rps',
                'front_minimum_m',
                'front_p10_m',
            )
        }
        self.stale_counts = {
            name: 0 for name in (
                'raw_command',
                'smoothed_command',
                'final_command',
                'odometry',
                'scan',
                'map_tf',
                'global_path',
                'local_path',
            )
        }

    def _add(self, name, value):
        if value is None or not math.isfinite(float(value)):
            return
        values = self.series[name]
        if len(values) < 20000:
            values.append(float(value))

    def observe(self, sample):
        now = float(sample['elapsed_s'])
        if self.started_at is None:
            self.started_at = now
        self.finished_at = now
        self.sample_count += 1
        elapsed = 0.0
        if self._previous_time is not None:
            elapsed = min(0.5, max(0.0, now - self._previous_time))
        self._previous_time = now

        commands = sample['commands']
        measured = sample.get('measured') or {}
        final = commands.get('final') or {}
        final_linear = abs(float(final.get('linear_x', 0.0)))
        measured_linear = abs(float(measured.get('linear_x', 0.0)))
        if final_linear >= 0.025:
            self.commanded_seconds += elapsed
            if measured_linear < 0.01:
                self.stalled_seconds += elapsed

        for name in ('raw', 'smoothed', 'final'):
            angular = float((commands.get(name) or {}).get('angular_z', 0.0))
            sign = 1 if angular >= 0.05 else -1 if angular <= -0.05 else 0
            previous = self._turn_signs[name]
            if sign and previous and sign != previous:
                self.turn_reversals[name] += 1
            if sign:
                self._turn_signs[name] = sign

        raw = commands.get('raw') or {}
        smoothed = commands.get('smoothed') or {}
        self._add(
            'raw_smoothed_linear_delta_mps',
            abs(float(raw.get('linear_x', 0.0))
                - float(smoothed.get('linear_x', 0.0))),
        )
        self._add(
            'raw_smoothed_angular_delta_rps',
            abs(float(raw.get('angular_z', 0.0))
                - float(smoothed.get('angular_z', 0.0))),
        )
        self._add(
            'smoothed_final_linear_delta_mps',
            abs(float(smoothed.get('linear_x', 0.0))
                - float(final.get('linear_x', 0.0))),
        )
        self._add(
            'smoothed_final_angular_delta_rps',
            abs(float(smoothed.get('angular_z', 0.0))
                - float(final.get('angular_z', 0.0))),
        )
        self._add(
            'final_measured_linear_error_mps',
            abs(float(final.get('linear_x', 0.0))
                - float(measured.get('linear_x', 0.0))),
        )
        self._add(
            'final_measured_angular_error_rps',
            abs(float(final.get('angular_z', 0.0))
                - float(measured.get('angular_z', 0.0))),
        )

        for path_name in ('global', 'local'):
            tracking = sample['tracking'].get(path_name)
            if tracking:
                self._add(
                    f'{path_name}_cross_track_m',
                    tracking['cross_track_m'],
                )
                self._add(
                    f'{path_name}_heading_error_rad',
                    tracking['heading_error_rad'],
                )

        front = ((sample.get('lidar') or {}).get('sectors') or {}).get('front')
        if front:
            self._add('front_minimum_m', front.get('minimum_m'))
            self._add('front_p10_m', front.get('p10_m'))

        pose = sample.get('pose_map')
        if pose and self._previous_pose and elapsed > 0.01:
            distance = math.hypot(
                pose['x'] - self._previous_pose['x'],
                pose['y'] - self._previous_pose['y'],
            )
            yaw_change = abs(normalize_angle(
                pose['yaw'] - self._previous_pose['yaw']
            ))
            if distance / elapsed > 0.45 or yaw_change / elapsed > 1.0:
                self.pose_jump_count += 1
        if pose:
            self._previous_pose = pose

        for name, age in sample['ages_s'].items():
            if age is None or float(age) > 0.6:
                self.stale_counts[name] += 1

    def marker(self):
        self.marker_count += 1

    def summary(self, outcome, goal_id, metadata):
        duration = max(
            0.0,
            float(self.finished_at or 0.0) - float(self.started_at or 0.0),
        )
        series_summary = {}
        for name, values in self.series.items():
            series_summary[name] = {
                'samples': len(values),
                'mean': finite_round(statistics.fmean(values)) if values else None,
                'p50': percentile(values, 0.50),
                'p95': percentile(values, 0.95),
                'maximum': finite_round(max(values)) if values else None,
                'minimum': finite_round(min(values)) if values else None,
            }
        return {
            'schema_version': SCHEMA_VERSION,
            'kind': 'navigation-tuning-summary',
            'created_at': utc_now(),
            'goal_id': goal_id,
            'outcome': outcome,
            'duration_s': round(duration, 3),
            'sample_count': self.sample_count,
            'sample_rate_hz': round(self.sample_count / max(0.001, duration), 3),
            'commanded_seconds': round(self.commanded_seconds, 3),
            'stalled_seconds': round(self.stalled_seconds, 3),
            'stalled_fraction': round(
                self.stalled_seconds / max(0.001, self.commanded_seconds),
                5,
            ),
            'turn_reversals': dict(self.turn_reversals),
            'localization_pose_jumps': self.pose_jump_count,
            'operator_markers': self.marker_count,
            'stale_sample_counts': dict(self.stale_counts),
            'metrics': series_summary,
            'configuration': metadata,
            'control_action': 'none',
        }


class NavigationTuningRecorder(Node):
    """Create a bounded evidence file for each NavigateToPose attempt."""

    def __init__(self):
        super().__init__('dogzilla_navigation_tuning_recorder')
        package_share = Path(__file__).resolve().parents[1]
        default_parameters = package_share / 'config' / 'nav2_test1.yaml'
        log_directory = Path(os.environ.get('ROS_LOG_DIR', '/tmp/dogzilla-ros'))

        self.declare_parameter(
            'output_directory',
            str(log_directory / 'navigation-tuning'),
        )
        self.declare_parameter('nav2_params_file', str(default_parameters))
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('pre_roll_seconds', 3.0)
        self.declare_parameter('maximum_trial_bytes', 8 * 1024 * 1024)
        self.declare_parameter('retained_trials', 12)

        self._output_directory = Path(
            self.get_parameter('output_directory').value
        ).resolve()
        self._output_directory.mkdir(parents=True, exist_ok=True)
        sample_rate = float(self.get_parameter('sample_rate_hz').value)
        pre_roll = float(self.get_parameter('pre_roll_seconds').value)
        self._maximum_bytes = int(
            self.get_parameter('maximum_trial_bytes').value
        )
        self._retained_trials = int(
            self.get_parameter('retained_trials').value
        )
        if not 2.0 <= sample_rate <= 20.0:
            raise ValueError('sample_rate_hz must be between 2 and 20')
        if not 0.0 <= pre_roll <= 10.0:
            raise ValueError('pre_roll_seconds must be between 0 and 10')
        if not 1 <= self._retained_trials <= 50:
            raise ValueError('retained_trials must be between 1 and 50')

        parameter_file = self.get_parameter('nav2_params_file').value
        self._configuration = load_tuning_profile(parameter_file)
        self._parameter_overrides = {}
        self._latest = {}
        self._paths = {'global': [], 'local': []}
        self._path_signatures = {'global': None, 'local': None}
        self._started_monotonic = self._now()
        self._pre_roll = deque(maxlen=max(1, math.ceil(sample_rate * pre_roll)))
        self._writer = None
        self._metrics = None
        self._goal_id = None
        self._status_state = 'idle'
        self._status_detail = 'Waiting for an autonomous navigation goal'
        self._last_status_published = 0.0
        self._last_summary = None
        self._last_marker_count = 0

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            String,
            '/navigation/tuning/status',
            status_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._on_goal_status,
            10,
        )
        for topic, name in (
            ('/cmd_vel_nav_raw', 'raw'),
            ('/cmd_vel_nav', 'smoothed'),
            ('/cmd_vel', 'final'),
        ):
            self.create_subscription(
                Twist,
                topic,
                lambda message, key=name: self._on_twist(key, message),
                10,
            )
        self.create_subscription(
            Odometry,
            '/odom',
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PathMessage,
            '/plan',
            lambda message: self._on_path('global', message),
            10,
        )
        self.create_subscription(
            PathMessage,
            '/local_plan',
            lambda message: self._on_path('local', message),
            10,
        )
        self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self._on_parameter_event,
            100,
        )
        self.create_subscription(
            String,
            '/navigation/diagnostics',
            self._on_diagnostics,
            status_qos,
        )
        self.create_subscription(
            String,
            '/navigation/tuning/marker',
            self._on_marker,
            10,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0 / sample_rate, self._sample)
        prune_artifacts(self._output_directory, self._retained_trials)
        self._publish_status('Waiting for an autonomous navigation goal')
        self.get_logger().info(
            'Targeted Nav2 recorder active; control_action=none'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _twist_value(message):
        return {
            'linear_x': finite_round(message.linear.x),
            'linear_y': finite_round(message.linear.y),
            'angular_z': finite_round(message.angular.z),
        }

    def _on_twist(self, name, message):
        self._latest[f'{name}_command'] = {
            'received': self._now(),
            'value': self._twist_value(message),
        }

    def _on_odometry(self, message):
        self._latest['odometry'] = {
            'received': self._now(),
            'timestamp_age': self._timestamp_age(message.header.stamp),
            'value': {
                'pose': {
                    'x': finite_round(message.pose.pose.position.x),
                    'y': finite_round(message.pose.pose.position.y),
                    'yaw': finite_round(quaternion_yaw(
                        message.pose.pose.orientation
                    )),
                },
                'twist': {
                    'linear_x': finite_round(message.twist.twist.linear.x),
                    'linear_y': finite_round(message.twist.twist.linear.y),
                    'angular_z': finite_round(message.twist.twist.angular.z),
                },
            },
        }

    def _on_scan(self, message):
        self._latest['scan'] = {
            'received': self._now(),
            'timestamp_age': self._timestamp_age(message.header.stamp),
            'value': laser_summary(
                message.angle_min,
                message.angle_increment,
                message.ranges,
                message.range_min,
                message.range_max,
            ),
        }

    def _on_path(self, name, message):
        points = _path_points(message)
        signature = hashlib.sha256(json.dumps(points).encode()).hexdigest()[:16]
        self._paths[name] = points
        self._latest[f'{name}_path'] = {
            'received': self._now(),
            'timestamp_age': self._timestamp_age(message.header.stamp),
            'value': {'points': len(points), 'signature': signature},
        }
        if signature == self._path_signatures[name]:
            return
        self._path_signatures[name] = signature
        if self._writer is not None:
            self._writer.write({
                'schema_version': SCHEMA_VERSION,
                'kind': 'path',
                'path': name,
                'elapsed_s': self._elapsed(),
                'frame_id': message.header.frame_id,
                'signature': signature,
                'points': [
                    {'x': round(x, 5), 'y': round(y, 5)} for x, y in points
                ],
            })

    def _on_diagnostics(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if (
            isinstance(value, dict)
            and value.get('kind') == 'navigation-diagnostics'
            and value.get('warning_only') is True
        ):
            self._latest['diagnostics'] = {
                'received': self._now(),
                'value': {
                    'state': value.get('state'),
                    'warning_codes': [
                        str(item.get('code')) for item in value.get('warnings', [])
                        if isinstance(item, dict) and item.get('code')
                    ][:10],
                },
            }

    def _on_parameter_event(self, message):
        node_name = str(message.node)
        selected = {}
        for parameter in (*message.new_parameters, *message.changed_parameters):
            if is_tuning_parameter(node_name, parameter.name):
                selected[parameter.name] = parameter_value_to_python(
                    parameter.value
                )
        for parameter in message.deleted_parameters:
            if is_tuning_parameter(node_name, parameter.name):
                selected[parameter.name] = None
        if not selected:
            return
        self._parameter_overrides.setdefault(node_name, {}).update(selected)
        if self._writer is not None:
            self._writer.write({
                'schema_version': SCHEMA_VERSION,
                'kind': 'parameter-change',
                'elapsed_s': self._elapsed(),
                'node': node_name,
                'parameters': selected,
            })

    def _on_marker(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        note = str(value.get('note', '')).strip() if isinstance(value, dict) else ''
        if (
            not isinstance(value, dict)
            or value.get('kind') != 'navigation-tuning-marker'
            or not note
            or len(note) > 120
        ):
            return
        if self._writer is None:
            self._publish_status('No active navigation trial to mark')
            return
        record = {
            'schema_version': SCHEMA_VERSION,
            'kind': 'operator-marker',
            'elapsed_s': self._elapsed(),
            'note': note,
            'received_at': utc_now(),
        }
        self._writer.write(record)
        self._metrics.marker()
        self._publish_status('Unstable movement marker recorded')

    def _on_goal_status(self, message):
        statuses = list(message.status_list)
        if self._goal_id is not None:
            current = next(
                (
                    status for status in statuses
                    if _goal_identifier(status) == self._goal_id
                ),
                None,
            )
            if current is not None and current.status in TERMINAL_GOAL_STATES:
                self._finish_trial(TERMINAL_GOAL_STATES[current.status])
        if self._goal_id is None:
            active = [
                status for status in statuses
                if status.status in ACTIVE_GOAL_STATES
            ]
            if active:
                active.sort(
                    key=lambda status: (
                        status.goal_info.stamp.sec,
                        status.goal_info.stamp.nanosec,
                    )
                )
                self._start_trial(_goal_identifier(active[-1]))

    def _configuration_snapshot(self):
        return {
            'nav2_profile': self._configuration,
            'runtime_parameter_overrides': json.loads(json.dumps(
                self._parameter_overrides,
                allow_nan=False,
            )),
            'recorded_signals': [
                'cmd_vel_nav_raw',
                'cmd_vel_nav',
                'cmd_vel',
                'odom',
                'map_to_base_tf',
                'global_plan',
                'local_plan',
                'lidar_sector_clearance',
                'navigation_diagnostics',
                'navigate_to_pose_status',
                'tuning_parameter_events',
                'operator_markers',
            ],
            'excluded_high_volume_signals': [
                'camera_images',
                'full_costmaps',
                'joint_states',
                'raw_ros_graph',
            ],
        }

    def _start_trial(self, goal_id):
        if self._writer is not None:
            return
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        name = f'{timestamp}_{goal_id[:12].upper()}'
        try:
            self._writer = BoundedTrialWriter(
                self._output_directory,
                name,
                self._maximum_bytes,
            )
        except (OSError, ValueError) as exc:
            self._status_state = 'error'
            self._publish_status(f'Cannot create tuning artifact: {exc}')
            self.get_logger().error(str(exc))
            return
        self._goal_id = goal_id
        self._metrics = NavigationTuningMetrics()
        self._status_state = 'recording'
        self._writer.write({
            'schema_version': SCHEMA_VERSION,
            'kind': 'trial-start',
            'goal_id': goal_id,
            'created_at': utc_now(),
            'configuration': self._configuration_snapshot(),
            'control_action': 'none',
        })
        for sample in self._pre_roll:
            self._writer.write({**sample, 'pre_roll': True})
        self._publish_status('Recording synchronized Nav2 tuning evidence')
        self.get_logger().info(f'Navigation tuning trial started: {name}')

    def _finish_trial(self, outcome):
        if self._writer is None or self._metrics is None:
            self._goal_id = None
            return
        writer = self._writer
        metrics = self._metrics
        goal_id = self._goal_id
        writer.write({
            'schema_version': SCHEMA_VERSION,
            'kind': 'trial-finish',
            'goal_id': goal_id,
            'outcome': outcome,
            'elapsed_s': self._elapsed(),
            'created_at': utc_now(),
        })
        summary = metrics.summary(
            outcome,
            goal_id,
            self._configuration_snapshot(),
        )
        try:
            self._last_summary = writer.finish(summary)
        except OSError as exc:
            writer.close()
            self._writer = None
            self._metrics = None
            self._goal_id = None
            self._status_state = 'error'
            self._publish_status(f'Could not finish tuning artifact: {exc}')
            self.get_logger().error(
                f'Could not finish navigation tuning artifact: {exc}'
            )
            return
        self._last_marker_count = metrics.marker_count
        self._writer = None
        self._metrics = None
        self._goal_id = None
        self._status_state = 'complete'
        prune_artifacts(self._output_directory, self._retained_trials)
        self._publish_status(f'Trial finished: {outcome}')
        self.get_logger().info(
            f'Navigation tuning trial finished: {outcome}; '
            f'{writer.summary_path}'
        )

    def _elapsed(self):
        return round(self._now() - self._started_monotonic, 5)

    def _timestamp_age(self, stamp):
        timestamp = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if timestamp <= 0.0:
            return None
        return max(0.0, self._now() - timestamp)

    def _age(self, name, now):
        item = self._latest.get(name)
        if item is None:
            return None
        receive_age = max(0.0, now - item['received'])
        timestamp_age = item.get('timestamp_age')
        if timestamp_age is not None:
            return finite_round(timestamp_age + receive_age)
        return finite_round(receive_age)

    def _value(self, name):
        item = self._latest.get(name)
        return item['value'] if item else None

    def _map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return None
        self._latest['map_tf'] = {
            'received': self._now(),
            'timestamp_age': self._timestamp_age(transform.header.stamp),
            'value': True,
        }
        return {
            'x': finite_round(transform.transform.translation.x),
            'y': finite_round(transform.transform.translation.y),
            'yaw': finite_round(quaternion_yaw(transform.transform.rotation)),
        }

    def _sample(self):
        now = self._now()
        pose_map = self._map_pose()
        odometry = self._value('odometry') or {}
        pose_odom = odometry.get('pose')
        tracking = {'global': None, 'local': None}
        if pose_map:
            tracking['global'] = closest_path_error(
                pose_map['x'],
                pose_map['y'],
                pose_map['yaw'],
                self._paths['global'],
            )
        if pose_odom:
            tracking['local'] = closest_path_error(
                pose_odom['x'],
                pose_odom['y'],
                pose_odom['yaw'],
                self._paths['local'],
            )
        sample = {
            'schema_version': SCHEMA_VERSION,
            'kind': 'sample',
            'created_at': utc_now(),
            'elapsed_s': round(now - self._started_monotonic, 5),
            'pose_map': pose_map,
            'pose_odom': pose_odom,
            'commands': {
                name: self._value(f'{name}_command')
                for name in ('raw', 'smoothed', 'final')
            },
            'measured': odometry.get('twist'),
            'tracking': tracking,
            'lidar': self._value('scan'),
            'diagnostics': self._value('diagnostics'),
            'ages_s': {
                name: self._age(name, now)
                for name in (
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
            'control_action': 'none',
        }
        self._pre_roll.append(sample)
        if self._writer is not None:
            self._writer.write(sample)
            self._metrics.observe(sample)
        if now - self._last_status_published >= 1.0:
            self._publish_status(self._status_detail)

    def _publish_status(self, detail):
        self._status_detail = str(detail)[:200]
        self._last_status_published = self._now()
        payload = {
            'schema_version': SCHEMA_VERSION,
            'kind': 'navigation-tuning-recorder',
            'state': self._status_state,
            'detail': self._status_detail,
            'goal_id': self._goal_id,
            'operator_markers': (
                self._metrics.marker_count
                if self._metrics is not None
                else self._last_marker_count
            ),
            'artifact': (
                self._last_summary.get('artifact')
                if self._last_summary is not None
                and self._status_state == 'complete'
                else None
            ),
            'control_action': 'none',
            'updated_at': utc_now(),
        }
        message = String()
        message.data = json.dumps(
            payload,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._status_publisher.publish(message)

    def destroy_node(self):
        if self._writer is not None:
            self._finish_trial('interrupted')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTuningRecorder()
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
