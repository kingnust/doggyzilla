"""Warning-only navigation health telemetry and bounded diagnostic journal."""

from collections import deque
import json
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(value):
    """Wrap one angle to [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(quaternion):
    """Return planar yaw from a geometry quaternion."""
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class BoundedJsonlRecorder:
    """Append JSON records while retaining at most two bounded files."""

    def __init__(self, path, maximum_bytes=4 * 1024 * 1024):
        self.path = Path(path)
        self.maximum_bytes = int(maximum_bytes)
        if self.maximum_bytes < 4096:
            raise ValueError('maximum_bytes must be at least 4096')
        self.path.parent.mkdir(parents=True, exist_ok=True)

    @property
    def previous_path(self):
        return self.path.with_name(self.path.name + '.1')

    def write(self, payload):
        encoded = (
            json.dumps(
                payload,
                sort_keys=True,
                separators=(',', ':'),
                allow_nan=False,
            )
            + '\n'
        ).encode('utf-8')
        if len(encoded) > self.maximum_bytes:
            raise ValueError('diagnostic record exceeds bounded file size')
        current_size = self.path.stat().st_size if self.path.exists() else 0
        if current_size + len(encoded) > self.maximum_bytes:
            if self.previous_path.exists():
                self.previous_path.unlink()
            if self.path.exists():
                self.path.replace(self.previous_path)
        with self.path.open('ab') as stream:
            stream.write(encoded)


class NavigationWarningTracker:
    """Apply persistence and hysteresis to passive navigation observations."""

    def __init__(
        self,
        *,
        started_at=None,
        startup_grace_seconds=5.0,
        warning_persistence_seconds=1.5,
        recovery_seconds=3.0,
        scan_timeout_seconds=0.6,
        odom_timeout_seconds=0.6,
        tf_timeout_seconds=0.8,
        pose_jump_speed_mps=0.45,
        yaw_jump_speed_rps=1.0,
        jump_window_seconds=3.0,
        jump_count=2,
        oscillation_window_seconds=4.0,
        oscillation_count=5,
        oscillation_minimum_rps=0.08,
        stall_window_seconds=2.0,
        stall_command_max_age_seconds=0.35,
        stall_evidence_fraction=0.8,
        stall_linear_command_mps=0.035,
        stall_linear_measured_mps=0.012,
        stall_displacement_m=0.015,
        stall_turn_command_rps=0.10,
        stall_turn_measured_rps=0.035,
        stall_yaw_change_rad=0.06,
    ):
        self.started_at = (
            time.monotonic() if started_at is None else float(started_at)
        )
        self.startup_grace_seconds = float(startup_grace_seconds)
        self.warning_persistence_seconds = float(
            warning_persistence_seconds
        )
        self.recovery_seconds = float(recovery_seconds)
        self.timeouts = {
            'scan': float(scan_timeout_seconds),
            'odom': float(odom_timeout_seconds),
            'tf': float(tf_timeout_seconds),
        }
        self.pose_jump_speed_mps = float(pose_jump_speed_mps)
        self.yaw_jump_speed_rps = float(yaw_jump_speed_rps)
        self.jump_window_seconds = float(jump_window_seconds)
        self.jump_count = int(jump_count)
        self.oscillation_window_seconds = float(oscillation_window_seconds)
        self.oscillation_count = int(oscillation_count)
        self.oscillation_minimum_rps = float(oscillation_minimum_rps)
        self.stall_window_seconds = float(stall_window_seconds)
        self.stall_command_max_age_seconds = float(
            stall_command_max_age_seconds
        )
        self.stall_evidence_fraction = float(stall_evidence_fraction)
        self.stall_thresholds = {
            'linear_command_mps': float(stall_linear_command_mps),
            'linear_measured_mps': float(stall_linear_measured_mps),
            'displacement_m': float(stall_displacement_m),
            'turn_command_rps': float(stall_turn_command_rps),
            'turn_measured_rps': float(stall_turn_measured_rps),
            'yaw_change_rad': float(stall_yaw_change_rad),
        }
        if min(
            self.startup_grace_seconds,
            self.warning_persistence_seconds,
            self.recovery_seconds,
            *self.timeouts.values(),
        ) <= 0.0:
            raise ValueError('diagnostic timing values must be positive')
        if self.jump_count < 2 or self.oscillation_count < 3:
            raise ValueError('diagnostic event counts are too small')
        if min(
            self.stall_window_seconds,
            self.stall_command_max_age_seconds,
            *self.stall_thresholds.values(),
        ) <= 0.0:
            raise ValueError('stall evidence thresholds must be positive')
        if not 0.5 <= self.stall_evidence_fraction <= 1.0:
            raise ValueError('stall_evidence_fraction must be between 0.5 and 1')

        self._last_received = {'scan': None, 'odom': None, 'tf': None}
        self._stamp_age_at_receive = {
            'scan': None,
            'odom': None,
            'tf': None,
        }
        self._previous_pose = None
        self._pose_jumps = deque()
        self._angular_flips = deque()
        self._previous_angular_command = None
        self._command = {'linear_mps': 0.0, 'angular_rps': 0.0}
        self._command_received = None
        self._command_history = deque()
        self._motion_history = deque()
        self._stall_evidence = {
            'ready': False,
            'window_seconds': self.stall_window_seconds,
            'linear_candidate': False,
            'turn_candidate': False,
        }
        self._pending = {}
        self._active = {}
        self._clear_since = {}

    @staticmethod
    def _round_age(value):
        return None if value is None else round(max(0.0, value), 3)

    def _observe_source(self, name, now, stamp_age=None):
        self._last_received[name] = float(now)
        self._stamp_age_at_receive[name] = (
            None if stamp_age is None else float(stamp_age)
        )

    def observe_scan(self, now, stamp_age=None):
        self._observe_source('scan', now, stamp_age)

    def observe_tf(self, now, stamp_age=None):
        self._observe_source('tf', now, stamp_age)

    def observe_odometry(
        self,
        now,
        x,
        y,
        yaw,
        stamp_age=None,
        linear_mps=None,
        angular_rps=None,
    ):
        now = float(now)
        pose = (now, float(x), float(y), float(yaw))
        if self._previous_pose is not None:
            previous_time, previous_x, previous_y, previous_yaw = (
                self._previous_pose
            )
            elapsed = now - previous_time
            if 0.02 <= elapsed <= 1.0:
                distance = math.hypot(
                    pose[1] - previous_x,
                    pose[2] - previous_y,
                )
                yaw_delta = abs(normalize_angle(pose[3] - previous_yaw))
                if (
                    distance / elapsed > self.pose_jump_speed_mps
                    or yaw_delta / elapsed > self.yaw_jump_speed_rps
                ):
                    self._pose_jumps.append(now)
        self._previous_pose = pose
        self._motion_history.append((
            now,
            pose[1],
            pose[2],
            pose[3],
            None if linear_mps is None else abs(float(linear_mps)),
            None if angular_rps is None else abs(float(angular_rps)),
        ))
        self._observe_source('odom', now, stamp_age)

    def observe_command(self, now, linear_mps, angular_rps):
        now = float(now)
        angular = float(angular_rps)
        previous = self._previous_angular_command
        if (
            previous is not None
            and abs(previous) >= self.oscillation_minimum_rps
            and abs(angular) >= self.oscillation_minimum_rps
            and previous * angular < 0.0
        ):
            self._angular_flips.append(now)
        if abs(angular) >= self.oscillation_minimum_rps:
            self._previous_angular_command = angular
        self._command = {
            'linear_mps': round(abs(float(linear_mps)), 4),
            'angular_rps': round(angular, 4),
        }
        self._command_received = now
        self._command_history.append((
            now,
            abs(float(linear_mps)),
            angular,
        ))

    def _source_age(self, name, now):
        received = self._last_received[name]
        return None if received is None else max(0.0, now - received)

    def _stamp_age(self, name, now):
        received = self._last_received[name]
        age_at_receive = self._stamp_age_at_receive[name]
        if received is None or age_at_receive is None:
            return None
        return max(0.0, age_at_receive + now - received)

    @staticmethod
    def _warning(code, message):
        return {'code': code, 'severity': 'warning', 'message': message}

    @staticmethod
    def _median(values):
        ordered = sorted(float(value) for value in values)
        if not ordered:
            return None
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0

    def _trim_stall_history(self, now):
        keep_after = now - self.stall_window_seconds - 0.5
        for history in (self._command_history, self._motion_history):
            while history and history[0][0] < keep_after:
                history.popleft()

    def _stall_candidates(self, now):
        """Infer sustained command-without-motion, never physical contact."""
        self._trim_stall_history(now)
        cutoff = now - self.stall_window_seconds
        commands = [item for item in self._command_history if item[0] >= cutoff]
        motions = [item for item in self._motion_history if item[0] >= cutoff]
        evidence = {
            'ready': False,
            'window_seconds': self.stall_window_seconds,
            'linear_candidate': False,
            'turn_candidate': False,
            'command_samples': len(commands),
            'odometry_samples': len(motions),
        }
        candidates = {}
        if (
            self._command_received is None
            or now - self._command_received
            > self.stall_command_max_age_seconds
            or self._source_age('odom', now) is None
            or self._source_age('odom', now) > self.timeouts['odom']
            or len(commands) < 4
            or len(motions) < 4
        ):
            self._stall_evidence = evidence
            return candidates

        command_span = commands[-1][0] - commands[0][0]
        motion_span = motions[-1][0] - motions[0][0]
        required_span = self.stall_window_seconds * 0.75
        if command_span < required_span or motion_span < required_span:
            self._stall_evidence = evidence
            return candidates

        linear_threshold = self.stall_thresholds['linear_command_mps']
        turn_threshold = self.stall_thresholds['turn_command_rps']
        linear_fraction = sum(
            command[1] >= linear_threshold for command in commands
        ) / len(commands)
        positive_turn_fraction = sum(
            command[2] >= turn_threshold for command in commands
        ) / len(commands)
        negative_turn_fraction = sum(
            command[2] <= -turn_threshold for command in commands
        ) / len(commands)
        turn_fraction = max(positive_turn_fraction, negative_turn_fraction)
        turn_only_fraction = sum(
            command[1] < linear_threshold for command in commands
        ) / len(commands)

        first_motion = motions[0]
        last_motion = motions[-1]
        displacement = math.hypot(
            last_motion[1] - first_motion[1],
            last_motion[2] - first_motion[2],
        )
        yaw_change = abs(normalize_angle(last_motion[3] - first_motion[3]))
        measured_linear = self._median([
            motion[4] for motion in motions if motion[4] is not None
        ])
        measured_angular = self._median([
            motion[5] for motion in motions if motion[5] is not None
        ])
        median_linear_command = self._median([
            command[1] for command in commands
        ])
        median_turn_command = self._median([
            abs(command[2]) for command in commands
        ])

        linear_candidate = (
            linear_fraction >= self.stall_evidence_fraction
            and displacement <= self.stall_thresholds['displacement_m']
            and (
                measured_linear is None
                or measured_linear
                <= self.stall_thresholds['linear_measured_mps']
            )
        )
        turn_candidate = (
            turn_fraction >= self.stall_evidence_fraction
            and turn_only_fraction >= self.stall_evidence_fraction
            and yaw_change <= self.stall_thresholds['yaw_change_rad']
            and (
                measured_angular is None
                or measured_angular
                <= self.stall_thresholds['turn_measured_rps']
            )
        )
        evidence.update({
            'ready': True,
            'command_span_s': round(command_span, 3),
            'odometry_span_s': round(motion_span, 3),
            'linear_command_fraction': round(linear_fraction, 3),
            'consistent_turn_fraction': round(turn_fraction, 3),
            'turn_only_fraction': round(turn_only_fraction, 3),
            'median_linear_command_mps': (
                None
                if median_linear_command is None
                else round(median_linear_command, 4)
            ),
            'median_turn_command_rps': (
                None
                if median_turn_command is None
                else round(median_turn_command, 4)
            ),
            'measured_displacement_m': round(displacement, 4),
            'measured_yaw_change_rad': round(yaw_change, 4),
            'median_measured_linear_mps': (
                None
                if measured_linear is None
                else round(measured_linear, 4)
            ),
            'median_measured_angular_rps': (
                None
                if measured_angular is None
                else round(measured_angular, 4)
            ),
            'linear_candidate': linear_candidate,
            'turn_candidate': turn_candidate,
        })
        self._stall_evidence = evidence
        if linear_candidate:
            candidates['linear_stall_suspected'] = self._warning(
                'linear_stall_suspected',
                'movement was commanded but scan odometry showed only '
                f'{displacement:.3f}m progress; possible obstruction or '
                'motion inhibition',
            )
        if turn_candidate:
            candidates['turn_stall_suspected'] = self._warning(
                'turn_stall_suspected',
                'turning was commanded but scan odometry showed only '
                f'{yaw_change:.3f}rad rotation; possible obstruction or '
                'motion inhibition',
            )
        return candidates

    def _candidates(self, now):
        values = {}
        for source, label in (
            ('scan', 'LiDAR scan'),
            ('odom', 'scan-matched odometry'),
            ('tf', 'map-to-base transform'),
        ):
            age = self._source_age(source, now)
            if age is None:
                values[f'{source}_missing'] = self._warning(
                    f'{source}_missing',
                    f'{label} has not been received',
                )
            elif age > self.timeouts[source]:
                values[f'{source}_stale'] = self._warning(
                    f'{source}_stale',
                    f'{label} is {age:.2f}s old',
                )
            stamp_age = self._stamp_age(source, now)
            if stamp_age is not None and stamp_age > self.timeouts[source]:
                values[f'{source}_timestamp_stale'] = self._warning(
                    f'{source}_timestamp_stale',
                    f'{label} timestamp is {stamp_age:.2f}s old',
                )

        while (
            self._pose_jumps
            and now - self._pose_jumps[0] > self.jump_window_seconds
        ):
            self._pose_jumps.popleft()
        if len(self._pose_jumps) >= self.jump_count:
            values['pose_jumps'] = self._warning(
                'pose_jumps',
                f'{len(self._pose_jumps)} localization jumps were observed',
            )

        while (
            self._angular_flips
            and now - self._angular_flips[0]
            > self.oscillation_window_seconds
        ):
            self._angular_flips.popleft()
        if len(self._angular_flips) >= self.oscillation_count:
            values['angular_oscillation'] = self._warning(
                'angular_oscillation',
                f'{len(self._angular_flips)} rapid turn reversals were commanded',
            )
        values.update(self._stall_candidates(now))
        return values

    def evaluate(self, now):
        now = float(now)
        in_startup_grace = (
            now - self.started_at < self.startup_grace_seconds
        )
        candidates = {} if in_startup_grace else self._candidates(now)

        for code, warning in candidates.items():
            first_seen = self._pending.setdefault(code, now)
            self._clear_since.pop(code, None)
            if now - first_seen >= self.warning_persistence_seconds:
                active = self._active.setdefault(
                    code,
                    {**warning, 'first_seen': first_seen},
                )
                active.update(warning)

        for code in set(self._pending) - set(candidates):
            self._pending.pop(code, None)
        for code in list(self._active):
            if code in candidates:
                continue
            clear_since = self._clear_since.setdefault(code, now)
            if now - clear_since >= self.recovery_seconds:
                self._active.pop(code, None)
                self._clear_since.pop(code, None)

        warnings = [
            {
                key: value
                for key, value in warning.items()
                if key != 'first_seen'
            }
            | {'observed_seconds': round(now - warning['first_seen'], 2)}
            for warning in sorted(
                self._active.values(),
                key=lambda item: item['code'],
            )
        ]
        all_sources_seen = all(
            value is not None for value in self._last_received.values()
        )
        state = (
            'warning'
            if warnings
            else 'starting'
            if in_startup_grace and not all_sources_seen
            else 'healthy'
        )
        command_age = (
            None
            if self._command_received is None
            else now - self._command_received
        )
        return {
            'state': state,
            'warnings': warnings,
            'metrics': {
                'scan_receive_age_s': self._round_age(
                    self._source_age('scan', now)
                ),
                'scan_timestamp_age_s': self._round_age(
                    self._stamp_age('scan', now)
                ),
                'odom_receive_age_s': self._round_age(
                    self._source_age('odom', now)
                ),
                'odom_timestamp_age_s': self._round_age(
                    self._stamp_age('odom', now)
                ),
                'tf_receive_age_s': self._round_age(
                    self._source_age('tf', now)
                ),
                'tf_timestamp_age_s': self._round_age(
                    self._stamp_age('tf', now)
                ),
                'pose_jump_count': len(self._pose_jumps),
                'angular_flip_count': len(self._angular_flips),
                'command_age_s': self._round_age(command_age),
                'command': dict(self._command),
                'stall_evidence': dict(self._stall_evidence),
            },
        }


class NavigationDiagnostics(Node):
    """Observe and report navigation health without controlling the robot."""

    def __init__(self):
        super().__init__('dogzilla_navigation_diagnostics')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('startup_grace_seconds', 5.0)
        self.declare_parameter('warning_persistence_seconds', 1.5)
        self.declare_parameter('recovery_seconds', 3.0)
        self.declare_parameter('scan_timeout_seconds', 0.6)
        self.declare_parameter('odom_timeout_seconds', 0.6)
        self.declare_parameter('tf_timeout_seconds', 0.8)
        self.declare_parameter('stall_window_seconds', 2.0)
        self.declare_parameter('stall_command_max_age_seconds', 0.35)
        self.declare_parameter('stall_evidence_fraction', 0.8)
        self.declare_parameter('stall_linear_command_mps', 0.035)
        self.declare_parameter('stall_linear_measured_mps', 0.012)
        self.declare_parameter('stall_displacement_m', 0.015)
        self.declare_parameter('stall_turn_command_rps', 0.10)
        self.declare_parameter('stall_turn_measured_rps', 0.035)
        self.declare_parameter('stall_yaw_change_rad', 0.06)
        self.declare_parameter('log_path', '')
        self.declare_parameter('maximum_log_bytes', 4 * 1024 * 1024)

        self._tracker = NavigationWarningTracker(
            started_at=time.monotonic(),
            startup_grace_seconds=self.get_parameter(
                'startup_grace_seconds'
            ).value,
            warning_persistence_seconds=self.get_parameter(
                'warning_persistence_seconds'
            ).value,
            recovery_seconds=self.get_parameter('recovery_seconds').value,
            scan_timeout_seconds=self.get_parameter(
                'scan_timeout_seconds'
            ).value,
            odom_timeout_seconds=self.get_parameter(
                'odom_timeout_seconds'
            ).value,
            tf_timeout_seconds=self.get_parameter('tf_timeout_seconds').value,
            stall_window_seconds=self.get_parameter(
                'stall_window_seconds'
            ).value,
            stall_command_max_age_seconds=self.get_parameter(
                'stall_command_max_age_seconds'
            ).value,
            stall_evidence_fraction=self.get_parameter(
                'stall_evidence_fraction'
            ).value,
            stall_linear_command_mps=self.get_parameter(
                'stall_linear_command_mps'
            ).value,
            stall_linear_measured_mps=self.get_parameter(
                'stall_linear_measured_mps'
            ).value,
            stall_displacement_m=self.get_parameter(
                'stall_displacement_m'
            ).value,
            stall_turn_command_rps=self.get_parameter(
                'stall_turn_command_rps'
            ).value,
            stall_turn_measured_rps=self.get_parameter(
                'stall_turn_measured_rps'
            ).value,
            stall_yaw_change_rad=self.get_parameter(
                'stall_yaw_change_rad'
            ).value,
        )
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._last_signature = None
        self._sequence = 0
        self._recording_error = None

        log_path = str(self.get_parameter('log_path').value).strip()
        if not log_path:
            log_directory = os.environ.get('ROS_LOG_DIR', '/logs/navigation')
            log_path = str(Path(log_directory) / 'navigation-diagnostics.jsonl')
        try:
            self._recorder = BoundedJsonlRecorder(
                log_path,
                self.get_parameter('maximum_log_bytes').value,
            )
        except (OSError, ValueError) as exc:
            self._recorder = None
            self._recording_error = str(exc)
            self.get_logger().warn(
                f'Navigation diagnostic journal is unavailable: {exc}'
            )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            String,
            '/navigation/diagnostics',
            status_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('command_topic').value,
            self._on_command,
            10,
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if not 0.5 <= publish_rate <= 5.0:
            raise ValueError('publish_rate_hz must be between 0.5 and 5')
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Warning-only navigation diagnostics active; movement_action=none'
        )

    def _stamp_age(self, stamp):
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return max(
            0.0,
            (self.get_clock().now().nanoseconds - stamp_ns) / 1e9,
        )

    def _on_scan(self, message):
        self._tracker.observe_scan(
            time.monotonic(),
            self._stamp_age(message.header.stamp),
        )

    def _on_odometry(self, message):
        pose = message.pose.pose
        self._tracker.observe_odometry(
            time.monotonic(),
            pose.position.x,
            pose.position.y,
            quaternion_yaw(pose.orientation),
            self._stamp_age(message.header.stamp),
            math.hypot(
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
            ),
            message.twist.twist.angular.z,
        )

    def _on_command(self, message):
        self._tracker.observe_command(
            time.monotonic(),
            math.hypot(message.linear.x, message.linear.y),
            message.angular.z,
        )

    def _observe_tf(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return
        self._tracker.observe_tf(
            time.monotonic(),
            self._stamp_age(transform.header.stamp),
        )

    def _publish(self):
        self._observe_tf()
        self._sequence += 1
        result = self._tracker.evaluate(time.monotonic())
        stamp = self.get_clock().now().to_msg()
        payload = {
            'schema_version': 1,
            'kind': 'navigation-diagnostics',
            'sequence': self._sequence,
            'warning_only': True,
            'movement_action': 'none',
            'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            **result,
            'recording': {
                'enabled': self._recorder is not None,
                'path': str(self._recorder.path) if self._recorder else None,
                'maximum_bytes_per_file': (
                    self._recorder.maximum_bytes if self._recorder else None
                ),
                'retained_files': 2,
                'error': self._recording_error,
            },
        }
        message = String()
        message.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._publisher.publish(message)

        if self._recorder is not None:
            try:
                self._recorder.write(payload)
            except (OSError, ValueError) as exc:
                self._recording_error = str(exc)
                self._recorder = None
                self.get_logger().warn(
                    f'Navigation diagnostic journal stopped: {exc}'
                )

        signature = (
            payload['state'],
            tuple(item['code'] for item in payload['warnings']),
        )
        if signature != self._last_signature:
            if payload['state'] == 'warning':
                detail = '; '.join(
                    item['message'] for item in payload['warnings']
                )
                self.get_logger().warn(
                    f'Navigation warning (monitoring only): {detail}'
                )
            elif self._last_signature is not None:
                self.get_logger().info(
                    f'Navigation diagnostics state: {payload["state"]}'
                )
            self._last_signature = signature


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDiagnostics()
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
