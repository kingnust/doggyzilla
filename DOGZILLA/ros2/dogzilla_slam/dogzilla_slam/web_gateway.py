"""ROS-aware monitoring and autonomous-task gateway for DOGZILLA."""

from action_msgs.msg import GoalStatus
from datetime import datetime, timezone
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import json
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import os
from pathlib import Path
from rcl_interfaces.srv import SetParametersAtomically
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CompressedImage, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .web_core import build_location_payload
from .web_core import build_keepout_zone_payload
from .web_core import build_delivery_payload
from .web_core import build_patrol_area_payload
from .web_core import build_patrol_payload
from .web_core import build_route_payload
from .web_core import classify_robot_mode
from .web_core import ConflictError
from .web_core import EventBus
from .web_core import MAP_NAME_PATTERN
from .web_core import OccupancyMap
from .web_core import patrol_vision_readiness
from .web_core import TaskStore
from .web_core import TelemetryCache
from .web_core import utc_now
from .web_core import ValidationError
from .web_http import GatewayHTTPServer
from .object_detector import validate_detection_payload
from .speed_control import AUTONOMY_ANGULAR_LIMITS
from .speed_control import AUTONOMY_LINEAR_LIMITS
from .speed_control import normalize_speed_level
from .speed_control import SPEED_LEVELS
from .speed_control import TURN_LEVELS
from .vision_core import validate_request
from .vision_core import validate_danger_confirmation
from .vision_core import validate_patrol_detection_payload
from .vision_core import DangerConfirmationTracker
from .vision_core import VisionConfigurationError


NAV2_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'cancelling',
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'cancelled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class DogzillaWebGateway(Node):
    """Aggregate telemetry and execute validated Nav2 waypoint tasks."""

    def __init__(self):
        super().__init__('dogzilla_web_gateway')
        self._lock = threading.RLock()
        self.events = EventBus()
        self.telemetry = TelemetryCache()

        self.web_host = os.environ.get('DOGZILLA_WEB_HOST', '127.0.0.1')
        self.web_port = self._integer_environment(
            'DOGZILLA_WEB_PORT',
            8080,
            1,
            65535,
        )
        self.web_password = os.environ.get(
            'DOGZILLA_WEB_PASSWORD',
            'yahboom',
        ).strip()
        if not 6 <= len(self.web_password) <= 128:
            raise RuntimeError(
                'DOGZILLA_WEB_PASSWORD must contain 6 to 128 characters'
            )
        self.web_legacy_token = os.environ.get(
            'DOGZILLA_WEB_TOKEN',
            '',
        ).strip()
        self.manual_drive_enabled = self._boolean_environment(
            'DOGZILLA_WEB_MANUAL_DRIVE_ENABLED',
            False,
        )
        self.require_initial_pose = self._boolean_environment(
            'DOGZILLA_WEB_REQUIRE_INITIAL_POSE',
            True,
        )
        self.map_name = os.environ.get(
            'DOGZILLA_WEB_MAP_NAME',
            'test1',
        ).strip()
        if not MAP_NAME_PATTERN.fullmatch(self.map_name):
            raise RuntimeError(
                'DOGZILLA_WEB_MAP_NAME contains unsupported characters'
            )
        self.minimum_battery = self._float_environment(
            'DOGZILLA_WEB_MIN_BATTERY',
            28.0,
            26.0,
            100.0,
        )
        database_path = os.environ.get(
            'DOGZILLA_WEB_DATABASE',
            '/data/tasks.sqlite3',
        )
        self.store = TaskStore(database_path)
        default_alert_directory = Path(database_path).parent / 'alerts'
        self.alert_directory = Path(os.environ.get(
            'DOGZILLA_WEB_ALERT_DIRECTORY',
            str(default_alert_directory),
        )).resolve()
        self.alert_directory.mkdir(parents=True, exist_ok=True)
        if not self.alert_directory.is_dir():
            raise RuntimeError('DOGZILLA web alert path is not a directory')
        self._alert_cooldown = self._float_environment(
            'DOGZILLA_WEB_ALERT_COOLDOWN',
            30.0,
            5.0,
            300.0,
        )
        self._person_tracker = DangerConfirmationTracker(
            minimum_confidence=0.65,
            minimum_observations=3,
            minimum_duration_seconds=0.8,
            minimum_iou=0.35,
            maximum_gap_seconds=1.5,
            cooldown_seconds=8.0,
            required_label='person',
            require_dangerous=False,
        )
        self._recent_alerts = {}
        self._prune_alert_photos()
        self._restore_alert_deduplication()
        self.occupancy_map = OccupancyMap(
            self.map_name,
            occupied_threshold=self._integer_environment(
                'DOGZILLA_WEB_OCCUPIED_THRESHOLD',
                50,
                1,
                100,
            ),
            minimum_clearance_m=self._float_environment(
                'DOGZILLA_WEB_GOAL_CLEARANCE',
                0.18,
                0.0,
                2.0,
            ),
            keepout_clearance_m=self._float_environment(
                'DOGZILLA_WEB_KEEPOUT_CLEARANCE',
                0.32,
                0.0,
                2.0,
            ),
        )

        self._estop_latched = False
        self._active = None
        self._cancel_requests = set()
        self._cancel_reasons = {}
        self._stop_until = 0.0
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._drive_speed_level = 4
        self._drive_turn_level = 4
        self._autonomy_update_pending = False
        self._manual_command_until = 0.0
        self._map_switch_pending = False
        self._map_switch_target = None
        self._map_switch_received_map = False
        self._map_switch_started_ns = 0
        self._waiting_for_map_pose = False
        self._localization_state = (
            'awaiting-initial-pose'
            if self.require_initial_pose
            else 'matching'
        )
        self._localization_requested_pose = None
        self._localization_started_ns = 0
        self._localization_pose_samples = 0
        self._localization_last_pose = None
        self._localization_last_stamp_ns = 0
        self._keepout_map_signature = None
        self._vision_frame = None
        self._vision_frame_received = 0.0
        self._vision_frame_sequence = 0
        self._vision_frame_condition = threading.Condition(self._lock)
        self._vision_status_signature = None
        self._vision_action_status_signature = None
        self._navigation_diagnostics_signature = None
        self._navigation_tuning_signature = None
        self._graph = {
            'mode': 'stopped',
            'nodes': [],
            'nav_available': False,
            'updated_at': utc_now(),
        }

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._keepout_mask_publisher = self.create_publisher(
            OccupancyGrid,
            '/keepout_filter_mask',
            map_qos,
        )
        self._keepout_info_publisher = self.create_publisher(
            CostmapFilterInfo,
            '/keepout_filter_info',
            map_qos,
        )
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self._on_battery,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            '/vision/detections',
            self._on_vision_detections,
            10,
        )
        self.create_subscription(
            String,
            '/vision/danger_confirmed',
            self._on_danger_confirmed,
            10,
        )
        vision_status_qos = QoSProfile(depth=1)
        vision_status_qos.reliability = ReliabilityPolicy.RELIABLE
        vision_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            '/vision/status',
            self._on_vision_status,
            vision_status_qos,
        )
        self.create_subscription(
            String,
            '/vision/action_status',
            self._on_vision_action_status,
            vision_status_qos,
        )
        self.create_subscription(
            String,
            '/navigation/diagnostics',
            self._on_navigation_diagnostics,
            vision_status_qos,
        )
        self.create_subscription(
            String,
            '/navigation/tuning/status',
            self._on_navigation_tuning_status,
            vision_status_qos,
        )
        self.create_subscription(
            CompressedImage,
            '/vision/annotated/compressed',
            self._on_vision_frame,
            qos_profile_sensor_data,
        )
        self._vision_command_publisher = self.create_publisher(
            String,
            '/vision/mode_command',
            10,
        )
        self._navigation_tuning_marker_publisher = self.create_publisher(
            String,
            '/navigation/tuning/marker',
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            map_qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._direct_stop_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )
        self._priority_stop_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_teleop',
            10,
        )
        self._estop_publisher = self.create_publisher(
            Bool,
            '/safety/estop',
            10,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self._safe_base_parameters = self.create_client(
            SetParametersAtomically,
            '/dogzilla_safe_base/set_parameters_atomically',
        )
        self._controller_parameters = self.create_client(
            SetParametersAtomically,
            '/controller_server/set_parameters_atomically',
        )
        self._velocity_smoother_parameters = self.create_client(
            SetParametersAtomically,
            '/velocity_smoother/set_parameters_atomically',
        )
        self._navigate = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._planner = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose',
        )
        self._task_timer = self.create_timer(0.10, self._tick)
        self._pose_timer = self.create_timer(0.10, self._update_map_pose)
        self._graph_timer = self.create_timer(1.0, self._refresh_graph)
        self.events.publish(
            'gateway.started',
            {
                'map': self.map_name,
                'minimum_battery': self.minimum_battery,
                'initial_pose_required': self.require_initial_pose,
            },
        )

    def _active_keepout_zones(self):
        return self.store.list_keepout_zones(self.map_name)

    def _publish_keepout_filter(self):
        """Publish the saved polygons as a transient-local Nav2 mask."""
        mask = self.occupancy_map.keepout_mask(
            self._active_keepout_zones()
        )
        timestamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp = timestamp
        message.header.frame_id = mask['frame']
        message.info.map_load_time = timestamp
        message.info.resolution = mask['resolution']
        message.info.width = mask['width']
        message.info.height = mask['height']
        origin = mask['origin']
        message.info.origin.position.x = origin['x']
        message.info.origin.position.y = origin['y']
        message.info.origin.orientation.z = math.sin(origin['yaw'] / 2.0)
        message.info.origin.orientation.w = math.cos(origin['yaw'] / 2.0)
        message.data = mask['data']
        self._keepout_mask_publisher.publish(message)

        information = CostmapFilterInfo()
        information.header.stamp = timestamp
        information.header.frame_id = mask['frame']
        information.type = 0
        information.filter_mask_topic = '/keepout_filter_mask'
        information.base = 0.0
        information.multiplier = 1.0
        self._keepout_info_publisher.publish(information)

    @staticmethod
    def _integer_environment(name, default, lower, upper):
        try:
            value = int(os.environ.get(name, str(default)))
        except ValueError as exc:
            raise RuntimeError(f'{name} must be an integer') from exc
        if not lower <= value <= upper:
            raise RuntimeError(f'{name} must be between {lower} and {upper}')
        return value

    @staticmethod
    def _float_environment(name, default, lower, upper):
        try:
            value = float(os.environ.get(name, str(default)))
        except ValueError as exc:
            raise RuntimeError(f'{name} must be a number') from exc
        if not math.isfinite(value) or not lower <= value <= upper:
            raise RuntimeError(f'{name} must be between {lower} and {upper}')
        return value

    @staticmethod
    def _boolean_environment(name, default):
        value = os.environ.get(
            name,
            'true' if default else 'false',
        ).strip().lower()
        if value in {'1', 'true', 'yes', 'on'}:
            return True
        if value in {'0', 'false', 'no', 'off'}:
            return False
        raise RuntimeError(f'{name} must be true or false')

    def log_http(self, message):
        self.get_logger().debug(message)

    def log_exception(self, message, exception):
        self.get_logger().error(f'{message}: {exception}')

    @staticmethod
    def _valid_alert_photo_name(name):
        value = str(name)
        if not value.startswith('alert-') or not value.endswith('.jpg'):
            return False
        identifier = value[6:-4]
        return len(identifier) == 32 and all(
            character in '0123456789abcdef' for character in identifier
        )

    def _delete_alert_photo(self, name):
        if not self._valid_alert_photo_name(name):
            return
        path = self.alert_directory / str(name)
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            self.get_logger().warn(
                f'Could not remove expired alert photo {name}: {exc}'
            )

    def _prune_alert_photos(self):
        """Keep at most 25 files created by the alert-photo subsystem."""
        try:
            photos = [
                path for path in self.alert_directory.iterdir()
                if path.is_file() and self._valid_alert_photo_name(path.name)
            ]
            photos.sort(
                key=lambda path: (path.stat().st_mtime_ns, path.name),
                reverse=True,
            )
        except OSError as exc:
            self.get_logger().warn(f'Could not inspect alert photos: {exc}')
            return
        for path in photos[25:]:
            self._delete_alert_photo(path.name)

    def _restore_alert_deduplication(self):
        """Prevent a gateway restart from immediately repeating an alert."""
        wall_now = datetime.now(timezone.utc)
        monotonic_now = time.monotonic()
        for alert in self.store.list_vision_alerts(self.map_name, 25):
            try:
                created = datetime.fromisoformat(
                    alert['created_at'].replace('Z', '+00:00')
                )
                age = max(0.0, (wall_now - created).total_seconds())
            except (AttributeError, TypeError, ValueError):
                continue
            if age >= self._alert_cooldown:
                continue
            key = (alert['category'], alert['label'])
            self._recent_alerts.setdefault(key, []).append({
                'time': monotonic_now - age,
                'box': tuple(alert['box']),
            })

    @staticmethod
    def _alert_box_iou(first, second):
        first_x, first_y, first_width, first_height = (
            float(value) for value in first
        )
        second_x, second_y, second_width, second_height = (
            float(value) for value in second
        )
        overlap_width = max(
            0.0,
            min(first_x + first_width, second_x + second_width)
            - max(first_x, second_x),
        )
        overlap_height = max(
            0.0,
            min(first_y + first_height, second_y + second_height)
            - max(first_y, second_y),
        )
        intersection = overlap_width * overlap_height
        union = (
            first_width * first_height
            + second_width * second_height
            - intersection
        )
        return intersection / union if union > 0.0 else 0.0

    @staticmethod
    def _public_alert(alert):
        value = dict(alert)
        photo_name = value.pop('photo_name', None)
        value['photo_url'] = (
            f"/api/v1/alerts/{value['id']}/photo.jpg"
            if photo_name else None
        )
        return value

    def _record_vision_alert(
        self,
        *,
        category,
        detection,
        confirmation,
        mode,
    ):
        """Store one deduplicated web alert and its latest annotated frame."""
        now = time.monotonic()
        key = (str(category), str(detection['label']))
        box = tuple(detection['box'])
        with self._lock:
            recent = [
                item for item in self._recent_alerts.get(key, [])
                if now - item['time'] < self._alert_cooldown
            ]
            if any(
                self._alert_box_iou(box, item['box']) >= 0.35
                for item in recent
            ):
                self._recent_alerts[key] = recent
                return None
            reservation = {'time': now, 'box': box}
            recent.append(reservation)
            self._recent_alerts[key] = recent
            frame = self._vision_frame
            frame_age = now - self._vision_frame_received
            active_task_id = self._active['task_id'] if self._active else None
        pose = self.telemetry.get('pose', stale_after=3.0)
        robot_pose = None if pose is None or pose['stale'] else pose['value']

        photo_name = None
        photo_path = None
        if frame is not None and frame_age <= 3.0:
            candidate = f'alert-{uuid.uuid4().hex}.jpg'
            if (
                len(frame) <= 4 * 1024 * 1024
                and frame.startswith(b'\xff\xd8')
                and frame.endswith(b'\xff\xd9')
            ):
                photo_path = self.alert_directory / candidate
                temporary = self.alert_directory / f'.{candidate}.tmp'
                try:
                    with temporary.open('xb') as stream:
                        stream.write(frame)
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(temporary, photo_path)
                    photo_name = candidate
                except OSError as exc:
                    temporary.unlink(missing_ok=True)
                    self.get_logger().warn(
                        f'Could not save alert photo: {exc}'
                    )

        try:
            alert, removed_photos = self.store.record_vision_alert({
                'task_id': active_task_id,
                'map': self.map_name,
                'category': category,
                'label': detection['label'],
                'confidence': detection['confidence'],
                'box': detection['box'],
                'robot_pose': robot_pose,
                'confirmation': {
                    'mode': mode,
                    **confirmation,
                },
                'photo_name': photo_name,
            }, limit=25)
        except Exception as exc:
            if photo_path is not None:
                self._delete_alert_photo(photo_path.name)
            with self._lock:
                recent = self._recent_alerts.get(key, [])
                self._recent_alerts[key] = [
                    item for item in recent if item is not reservation
                ]
                if not self._recent_alerts[key]:
                    self._recent_alerts.pop(key, None)
            self.get_logger().error(f'Could not store vision alert: {exc}')
            return None
        for expired_name in removed_photos:
            self._delete_alert_photo(expired_name)
        self._prune_alert_photos()
        return self._public_alert(alert)

    def _on_battery(self, message):
        percentage = None
        if math.isfinite(message.percentage):
            percentage = round(float(message.percentage) * 100.0, 1)
        value = {
            'percentage': percentage,
            'present': bool(message.present),
        }
        self.telemetry.update('battery', value)

    def _on_joints(self, message):
        positions = {
            name: round(float(position), 5)
            for name, position in zip(message.name, message.position)
            if math.isfinite(float(position))
        }
        self.telemetry.update(
            'joints',
            {'count': len(positions), 'positions': positions},
        )

    def _on_navigation_diagnostics(self, message):
        """Expose passive warnings without changing navigation state."""
        try:
            value = self._json_message(
                message,
                'navigation diagnostics',
            )
            state = value.get('state')
            warnings = value.get('warnings')
            if value.get('kind') != 'navigation-diagnostics':
                raise ValueError('navigation diagnostics kind is invalid')
            if value.get('warning_only') is not True:
                raise ValueError('navigation diagnostics are not warning-only')
            if value.get('movement_action') != 'none':
                raise ValueError('navigation diagnostics requested movement')
            if state not in {'starting', 'healthy', 'warning'}:
                raise ValueError('navigation diagnostics state is invalid')
            if not isinstance(warnings, list) or len(warnings) > 10:
                raise ValueError('navigation diagnostics warnings are invalid')
            normalized = []
            for warning in warnings:
                if not isinstance(warning, dict):
                    raise ValueError(
                        'navigation diagnostics warning is invalid'
                    )
                code = str(warning.get('code', '')).strip()
                detail = str(warning.get('message', '')).strip()
                if (
                    not code
                    or len(code) > 64
                    or not detail
                    or len(detail) > 200
                ):
                    raise ValueError(
                        'navigation diagnostics warning is invalid'
                    )
                normalized.append({**warning, 'code': code, 'message': detail})
            value['warnings'] = normalized
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        self.telemetry.update('navigation_diagnostics', value)
        signature = (state, tuple(item['code'] for item in normalized))
        with self._lock:
            previous = self._navigation_diagnostics_signature
            self._navigation_diagnostics_signature = signature
        if signature == previous:
            return
        if state == 'warning':
            self.events.publish('navigation.warning', value)
        elif previous is not None and previous[0] == 'warning':
            self.events.publish('navigation.warning_cleared', value)

    def _on_navigation_tuning_status(self, message):
        """Expose the read-only tuning recorder and its bounded artifact."""
        try:
            value = self._json_message(message, 'navigation tuning status')
            state = value.get('state')
            detail = str(value.get('detail', '')).strip()
            goal_id = value.get('goal_id')
            artifact = value.get('artifact')
            if value.get('kind') != 'navigation-tuning-recorder':
                raise ValueError('navigation tuning status kind is invalid')
            if value.get('control_action') != 'none':
                raise ValueError('navigation tuning recorder requested control')
            if state not in {'idle', 'recording', 'complete', 'error'}:
                raise ValueError('navigation tuning recorder state is invalid')
            if not detail or len(detail) > 200:
                raise ValueError('navigation tuning recorder detail is invalid')
            if goal_id is not None and (
                not isinstance(goal_id, str)
                or len(goal_id) != 32
                or any(
                    character not in '0123456789abcdef'
                    for character in goal_id
                )
            ):
                raise ValueError('navigation tuning goal identifier is invalid')
            if artifact is not None:
                if not isinstance(artifact, dict):
                    raise ValueError('navigation tuning artifact is invalid')
                for name in ('data', 'summary'):
                    path = artifact.get(name)
                    if not isinstance(path, str) or not 1 <= len(path) <= 1024:
                        raise ValueError('navigation tuning artifact is invalid')
                artifact = {
                    'data': artifact['data'],
                    'summary': artifact['summary'],
                    'bytes': int(artifact.get('bytes', 0)),
                    'records': int(artifact.get('records', 0)),
                    'truncated': bool(artifact.get('truncated', False)),
                    'maximum_bytes': int(artifact.get('maximum_bytes', 0)),
                }
                if min(
                    artifact['bytes'],
                    artifact['records'],
                    artifact['maximum_bytes'],
                ) < 0:
                    raise ValueError('navigation tuning artifact is invalid')
                value['artifact'] = artifact
            markers = int(value.get('operator_markers', 0))
            if not 0 <= markers <= 10000:
                raise ValueError('navigation tuning marker count is invalid')
            value['operator_markers'] = markers
            value['detail'] = detail
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(str(exc))
            return

        self.telemetry.update('navigation_tuning', value)
        signature = (state, goal_id, detail)
        with self._lock:
            previous = self._navigation_tuning_signature
            self._navigation_tuning_signature = signature
        if signature == previous:
            return
        if state == 'recording':
            self.events.publish('navigation.tuning_started', value)
        elif state == 'complete':
            self.events.publish('navigation.tuning_complete', value)
        elif state == 'error':
            self.events.publish('navigation.tuning_error', value)

    def _on_odometry(self, message):
        with self._lock:
            self._linear_speed = math.hypot(
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
            )
            self._angular_speed = float(message.twist.twist.angular.z)

    @staticmethod
    def _json_message(message, label):
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as exc:
            raise ValueError(f'invalid {label} JSON: {exc}') from exc
        if not isinstance(value, dict):
            raise ValueError(f'{label} must be a JSON object')
        return value

    def _on_vision_detections(self, message):
        try:
            value = self._json_message(message, 'vision detection')
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        detections = value.get('detections', [])
        if not isinstance(detections, list) or len(detections) > 100:
            self.get_logger().warn('Invalid vision detection list ignored')
            return
        if value.get('mode') in {
            'objects', 'dangerous-objects', 'floor-hazards', 'patrol'
        }:
            try:
                validator = (
                    validate_patrol_detection_payload
                    if value.get('mode') == 'patrol'
                    else validate_detection_payload
                )
                detections = [validator(item) for item in detections]
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(f'Invalid object detection ignored: {exc}')
                return
            value['detections'] = detections
        self.telemetry.update('vision', value)
        if value.get('mode') != 'patrol':
            with self._lock:
                self._person_tracker.reset()
            return
        now = time.monotonic()
        with self._lock:
            confirmations = self._person_tracker.observe(
                [
                    detection for detection in detections
                    if detection.get('fresh', True)
                ],
                now=now,
            )
        for confirmation in confirmations:
            confirmed = {
                'schema_version': 1,
                'kind': 'person-confirmation',
                'mode': 'patrol',
                'source_frame': value.get('source_frame', ''),
                'stamp': value.get('stamp'),
                **confirmation,
            }
            self.telemetry.update('person_confirmation', confirmed)
            self._handle_confirmed_person(confirmed)

    def _on_danger_confirmed(self, message):
        try:
            value = validate_danger_confirmation(
                self._json_message(message, 'danger confirmation')
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'Invalid danger confirmation ignored: {exc}'
            )
            return
        self.telemetry.update('danger_confirmation', value)
        self._handle_confirmed_hazard(value)

    def _handle_confirmed_hazard(self, confirmed):
        detection = confirmed['detection']
        with self._lock:
            active_task_id = self._active['task_id'] if self._active else None
        alert = self._record_vision_alert(
            category='danger',
            detection=detection,
            confirmation=confirmed['confirmation'],
            mode=confirmed['mode'],
        )
        observation = None
        if alert is not None:
            observation = self.store.record_hazard({
                'task_id': active_task_id,
                'map': self.map_name,
                'label': detection['label'],
                'risk': detection.get('risk', 'danger'),
                'confidence': detection['confidence'],
                'box': detection['box'],
                'robot_pose': alert['robot_pose'],
                'confirmation': {
                    'mode': confirmed['mode'],
                    **confirmed['confirmation'],
                },
            })
            observation['position_semantics'] = (
                'robot pose when observed; not the object position'
            )
            observation['photo_url'] = alert['photo_url']
            self.events.publish('hazard.confirmed', observation)

    def _handle_confirmed_person(self, confirmed):
        alert = self._record_vision_alert(
            category='person',
            detection=confirmed['detection'],
            confirmation=confirmed['confirmation'],
            mode='patrol',
        )
        if alert is not None:
            self.events.publish('person.confirmed', alert)

    def _on_vision_status(self, message):
        try:
            value = self._json_message(message, 'vision status')
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        self.telemetry.update('vision_status', value)
        signature = json.dumps(value, sort_keys=True, separators=(',', ':'))
        with self._lock:
            changed = signature != self._vision_status_signature
            self._vision_status_signature = signature
        if changed:
            self.events.publish('vision.status', value)

    def _on_vision_action_status(self, message):
        try:
            value = self._json_message(message, 'vision action status')
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        self.telemetry.update('vision_action_status', value)
        signature = json.dumps(value, sort_keys=True, separators=(',', ':'))
        with self._lock:
            changed = signature != self._vision_action_status_signature
            self._vision_action_status_signature = signature
        if changed:
            self.events.publish('vision.action_status', value)

    def _on_vision_frame(self, message):
        if message.format.lower() not in {'jpeg', 'jpg'}:
            self.get_logger().warn(
                'Ignoring unsupported annotated image format: '
                f'{message.format}'
            )
            return
        frame = bytes(message.data)
        if not frame or len(frame) > 4 * 1024 * 1024:
            self.get_logger().warn('Ignoring empty or oversized vision frame')
            return
        with self._lock:
            self._vision_frame = frame
            self._vision_frame_received = time.monotonic()
            self._vision_frame_sequence += 1
            self._vision_frame_condition.notify_all()

    @staticmethod
    def _quaternion_yaw(quaternion):
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _angle_distance(first, second):
        return abs(math.atan2(
            math.sin(first - second),
            math.cos(first - second),
        ))

    def _reset_localization_progress(self, state):
        self._localization_state = state
        self._localization_started_ns = 0
        self._localization_pose_samples = 0
        self._localization_last_pose = None
        self._localization_last_stamp_ns = 0

    def _update_localization_progress(self, stamp_ns, pose):
        """Require distinct, stable localization samples before autonomy."""
        with self._lock:
            state = self._localization_state
            if state in {'awaiting-initial-pose', 'ready'}:
                return
            if stamp_ns <= self._localization_last_stamp_ns:
                return
            if (
                self._localization_started_ns
                and stamp_ns < self._localization_started_ns
            ):
                return
            self._localization_last_stamp_ns = stamp_ns

            requested = self._localization_requested_pose
            if requested is not None:
                plausible = (
                    math.hypot(
                        pose[0] - requested['x'],
                        pose[1] - requested['y'],
                    ) <= 1.0
                    and self._angle_distance(
                        pose[2], requested['yaw']
                    ) <= 1.05
                )
                if not plausible:
                    self._localization_pose_samples = 0
                    self._localization_last_pose = pose
                    return

            previous = self._localization_last_pose
            stable = previous is None or (
                math.hypot(
                    pose[0] - previous[0],
                    pose[1] - previous[1],
                ) <= 0.15
                and self._angle_distance(pose[2], previous[2]) <= 0.20
            )
            self._localization_pose_samples = (
                self._localization_pose_samples + 1 if stable else 1
            )
            self._localization_last_pose = pose
            if self._localization_pose_samples < 20:
                return

            map_switch_was_pending = self._map_switch_pending
            self._localization_state = 'ready'
            self._map_switch_pending = False
            self._map_switch_target = None
            ready = {
                'map': self.map_name,
                'pose_samples': self._localization_pose_samples,
                'method': (
                    'initial-pose'
                    if requested is not None
                    else 'automatic-matching'
                ),
            }
        self.events.publish('localization.ready', ready)
        if map_switch_was_pending:
            self.events.publish('map.switch_ready', ready)

    def _update_map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            if not self._waiting_for_map_pose:
                self.get_logger().info(
                    f'Waiting for map-to-base localization transform: {exc}'
                )
                self._waiting_for_map_pose = True
            return
        self._waiting_for_map_pose = False
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._quaternion_yaw(rotation)
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        pose = (float(translation.x), float(translation.y), yaw)
        with self._lock:
            linear_speed = self._linear_speed
            angular_speed = self._angular_speed
            if self._map_switch_pending:
                if (
                    not self._map_switch_received_map
                    or stamp_ns < self._map_switch_started_ns
                ):
                    return
        self._update_localization_progress(stamp_ns, pose)
        self.telemetry.update(
            'pose',
            {
                'frame': transform.header.frame_id or 'map',
                'x': round(float(translation.x), 4),
                'y': round(float(translation.y), 4),
                'yaw': round(yaw, 4),
                'linear_speed': round(linear_speed, 4),
                'angular_speed': round(angular_speed, 4),
            },
        )

    def _on_map(self, message):
        origin = message.info.origin
        try:
            self.occupancy_map.update(
                frame=message.header.frame_id,
                width=message.info.width,
                height=message.info.height,
                resolution=message.info.resolution,
                origin_x=origin.position.x,
                origin_y=origin.position.y,
                origin_yaw=self._quaternion_yaw(origin.orientation),
                data=message.data,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid occupancy map ignored: {exc}')
            return
        summary = self.occupancy_map.summary()
        map_signature = (
            self.map_name,
            summary['frame'],
            summary['width'],
            summary['height'],
            summary['resolution'],
            summary['origin']['x'],
            summary['origin']['y'],
            summary['origin']['yaw'],
        )
        with self._lock:
            if self._map_switch_pending:
                self._map_switch_received_map = True
            publish_keepout = map_signature != self._keepout_map_signature
            if publish_keepout:
                self._keepout_map_signature = map_signature
        self.telemetry.update('map', summary)
        if publish_keepout:
            self._publish_keepout_filter()
        self.events.publish(
            'map.updated',
            {'name': self.map_name, 'revision': summary['revision']},
        )

    def _refresh_graph(self):
        node_names = sorted(set(self.get_node_names()))
        nav_available = self._navigate.server_is_ready()
        mode = classify_robot_mode(node_names, nav_available)
        graph = {
            'mode': mode,
            'nodes': node_names,
            'nav_available': nav_available,
            'updated_at': utc_now(),
        }
        with self._lock:
            changed = (
                graph['mode'] != self._graph['mode']
                or graph['nav_available'] != self._graph['nav_available']
            )
            self._graph = graph
        if changed:
            self.events.publish('robot.mode', graph)

    def _battery_gate(self):
        battery = self.telemetry.get('battery', stale_after=12.0)
        if battery is None or battery['stale']:
            return False, 'fresh battery telemetry is unavailable'
        battery_value = battery['value']
        if not battery_value['present'] or battery_value['percentage'] is None:
            return False, 'battery telemetry is invalid'
        if battery_value['percentage'] < self.minimum_battery:
            return False, (
                f"battery {battery_value['percentage']:.1f}% is below the "
                f'{self.minimum_battery:.1f}% task minimum'
            )
        return True, 'ready'

    def _task_gate(self, task=None):
        with self._lock:
            if self._estop_latched:
                return False, 'emergency stop is latched'
            if self._autonomy_update_pending:
                return False, 'autonomous speed update is in progress'
            if self._map_switch_pending:
                return False, (
                    'waiting for the new map and stable localization'
                )
            if self._localization_state == 'awaiting-initial-pose':
                return False, 'set and confirm the initial pose on the map'
            if self._localization_state != 'ready':
                return False, 'waiting for stable LiDAR scan matching'
            nav_available = bool(self._graph['nav_available'])
        if not nav_available:
            return False, 'Nav2 navigate_to_pose action is unavailable'
        battery_ready, battery_reason = self._battery_gate()
        if not battery_ready:
            return False, battery_reason
        pose = self.telemetry.get('pose', stale_after=3.0)
        if pose is None or pose['stale']:
            return False, 'fresh localization odometry is unavailable'
        if not self.occupancy_map.available():
            return False, 'map telemetry is unavailable'
        if task is not None and task['payload']['map'] != self.map_name:
            return False, (
                f"task belongs to map '{task['payload']['map']}', not "
                f"'{self.map_name}'"
            )
        if task is not None and task['kind'] == 'patrol':
            return self._patrol_vision_gate()
        return True, 'ready'

    def _patrol_vision_gate(self):
        vision = self.telemetry.get('vision_status', stale_after=5.0)
        if vision is None or vision['stale']:
            return False, 'fresh patrol vision status is unavailable'
        return patrol_vision_readiness(vision['value'])

    def get_state(self):
        ready, gate_reason = self._task_gate()
        with self._lock:
            active_task_id = self._active['task_id'] if self._active else None
            graph = dict(self._graph)
            estop_latched = self._estop_latched
            localization = {
                'required': self.require_initial_pose,
                'state': self._localization_state,
                'method': (
                    'initial-pose'
                    if self._localization_requested_pose is not None
                    else 'automatic-matching'
                ),
                'requested_pose': self._localization_requested_pose,
                'stable_samples': self._localization_pose_samples,
                'required_samples': 20,
            }
        active_task = (
            self.store.get(active_task_id)
            if active_task_id
            else None
        )
        return {
            'time': utc_now(),
            'configuration': {
                'map': self.map_name,
                'map_switch_pending': self._map_switch_pending,
                'map_switch_target': self._map_switch_target,
                'localization': localization,
            },
            'robot': graph,
            'telemetry': self.telemetry.snapshot(stale_after=10.0),
            'safety': {
                'estop_latched': estop_latched,
                'minimum_task_battery': self.minimum_battery,
                'task_ready': ready,
                'task_gate_reason': gate_reason,
            },
            'autonomy': self.get_autonomy_settings(),
            'active_task': active_task,
        }

    def list_tasks(self, limit=100):
        return self.store.list(limit)

    def get_task(self, task_id):
        return self.store.get(task_id)

    def get_map(self):
        return self.occupancy_map.payload()

    def set_initial_pose(self, value):
        """Publish one validated map pose to start/restart localization."""
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        map_name = str(value.get('map', '')).strip()
        if map_name != self.map_name:
            raise ValidationError(
                f"initial pose must belong to active map '{self.map_name}'"
            )
        try:
            x = float(value.get('x'))
            y = float(value.get('y'))
            yaw = float(value.get('yaw'))
        except (TypeError, ValueError) as exc:
            raise ValidationError(
                'initial pose x, y, and yaw must be numbers'
            ) from exc
        if not all(math.isfinite(item) for item in (x, y, yaw)):
            raise ValidationError('initial pose values must be finite')
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        pose = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}
        self.occupancy_map.validate_waypoints([{
            'label': 'Initial pose',
            **pose,
        }])
        if self._initial_pose_publisher.get_subscription_count() < 1:
            raise ConflictError(
                'localization manager is not ready to receive the pose yet'
            )

        with self._lock:
            if self._active is not None:
                raise ConflictError(
                    'cancel or finish the active task before resetting pose'
                )
            self._manual_command_until = 0.0
            self._localization_requested_pose = pose
            self._reset_localization_progress('matching')
            self._localization_started_ns = (
                self.get_clock().now().nanoseconds
            )
            if self._map_switch_pending:
                self._map_switch_started_ns = self._localization_started_ns

        self.telemetry.discard('pose')
        try:
            self._tf_buffer.clear()
        except AttributeError:
            pass
        self._publish_stop()

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = pose['x']
        message.pose.pose.position.y = pose['y']
        message.pose.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
        message.pose.covariance[0] = 0.0625
        message.pose.covariance[7] = 0.0625
        message.pose.covariance[35] = 0.0685
        self._initial_pose_publisher.publish(message)

        result = {
            'map': self.map_name,
            'pose': pose,
            'state': 'matching',
            'movement_action': 'stop-only',
        }
        self.events.publish('localization.initial_pose_requested', result)
        return result

    def prepare_map_switch(self, value):
        """Freeze motion and task dispatch before localization is stopped."""
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        map_name = str(value.get('map', '')).strip()
        if not MAP_NAME_PATTERN.fullmatch(map_name):
            raise ValidationError(
                'map may contain only letters, numbers, dot, underscore, '
                'and dash'
            )
        with self._lock:
            if self._active is not None:
                raise ConflictError(
                    'cancel or finish the active task before switching maps'
                )
            self._map_switch_pending = True
            self._map_switch_target = map_name
            self._map_switch_received_map = False
            self._map_switch_started_ns = 0
            self._manual_command_until = 0.0
        self.telemetry.discard('pose')
        self._publish_stop()
        result = {
            'map': self.map_name,
            'target_map': map_name,
            'state': 'prepared',
        }
        self.events.publish('map.switch_prepared', result)
        return result

    def switch_map(self, value):
        """Commit a prepared map after the old localization process stops."""
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        map_name = str(value.get('map', '')).strip()
        if not MAP_NAME_PATTERN.fullmatch(map_name):
            raise ValidationError(
                'map may contain only letters, numbers, dot, underscore, '
                'and dash'
            )
        with self._lock:
            if self._active is not None:
                raise ConflictError(
                    'cancel or finish the active task before switching maps'
                )
            if (
                not self._map_switch_pending
                or self._map_switch_target != map_name
            ):
                raise ConflictError(
                    'map switch must be prepared before it is committed'
                )
        replacement = OccupancyMap(
            map_name,
            occupied_threshold=self.occupancy_map.occupied_threshold,
            minimum_clearance_m=self.occupancy_map.minimum_clearance_m,
            keepout_clearance_m=self.occupancy_map.keepout_clearance_m,
        )
        with self._lock:
            previous_map = self.map_name
            self.map_name = map_name
            self.occupancy_map = replacement
            self._keepout_map_signature = None
            self._manual_command_until = 0.0
            self._drive_speed_level = 4
            self._drive_turn_level = 4
            self._map_switch_pending = True
            self._map_switch_target = map_name
            self._map_switch_received_map = False
            self._map_switch_started_ns = self.get_clock().now().nanoseconds
            self._localization_requested_pose = None
            self._reset_localization_progress(
                'awaiting-initial-pose'
                if self.require_initial_pose
                else 'matching'
            )
            self._person_tracker.reset()
            self._recent_alerts.clear()
            self._restore_alert_deduplication()
        self.telemetry.discard('map', 'pose')
        try:
            self._tf_buffer.clear()
        except AttributeError:
            pass
        result = {
            'map': map_name,
            'previous_map': previous_map,
            'state': 'waiting-for-localization',
            'keepout_zone_count': len(self._active_keepout_zones()),
        }
        self.events.publish('map.switch_started', result)
        return result

    def get_vision_frame(self):
        """Return the newest annotated JPEG while it is still live."""
        with self._lock:
            frame = self._vision_frame
            age = time.monotonic() - self._vision_frame_received
        if frame is None:
            raise ConflictError('vision frame is unavailable')
        if age > 3.0:
            raise ConflictError(f'vision frame is stale ({age:.1f}s old)')
        return frame

    def wait_for_vision_frame(self, after_sequence, timeout=1.0):
        """Return only a frame newer than the browser's last sequence."""
        try:
            after_sequence = int(after_sequence)
        except (TypeError, ValueError) as exc:
            raise ValidationError('vision frame sequence must be an integer') from exc
        if after_sequence < 0:
            raise ValidationError('vision frame sequence cannot be negative')
        with self._vision_frame_condition:
            self._vision_frame_condition.wait_for(
                lambda: self._vision_frame_sequence > after_sequence,
                timeout=float(timeout),
            )
            sequence = self._vision_frame_sequence
            frame = self._vision_frame
            age = time.monotonic() - self._vision_frame_received
        if sequence <= after_sequence:
            return None, sequence
        if frame is None or age > 3.0:
            raise ConflictError('vision frame is unavailable or stale')
        return frame, sequence

    def set_vision_mode(self, value):
        """Publish one validated detection-only configuration request."""
        try:
            requested = validate_request(value)
        except VisionConfigurationError as exc:
            raise ValidationError(str(exc)) from exc
        with self._lock:
            patrol_active = (
                self._active is not None
                and self._active['payload']['kind'] == 'patrol'
            )
        if patrol_active and requested['mode'] != 'patrol':
            raise ConflictError(
                'cannot leave patrol vision mode during an active patrol'
            )
        message = String()
        message.data = json.dumps(
            requested,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._vision_command_publisher.publish(message)
        response = {
            **requested,
            'state': 'requested',
            'action_output': 'disabled',
        }
        self.events.publish('vision.requested', response)
        return response

    def mark_navigation_tuning(self, value):
        """Mark unstable motion without stopping or changing navigation."""
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        note = str(
            value.get('note', 'Operator marked unstable movement')
        ).strip()
        if not note or len(note) > 120:
            raise ValidationError('marker note must contain 1 to 120 characters')
        status = self.telemetry.get('navigation_tuning', stale_after=3.0)
        if (
            status is None
            or status['stale']
            or status['value'].get('state') != 'recording'
        ):
            raise ConflictError('no active navigation tuning trial is recording')
        payload = {
            'schema_version': 1,
            'kind': 'navigation-tuning-marker',
            'note': note,
            'requested_at': utc_now(),
            'control_action': 'none',
        }
        message = String()
        message.data = json.dumps(
            payload,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._navigation_tuning_marker_publisher.publish(message)
        self.events.publish('navigation.tuning_marker', payload)
        return {'state': 'requested', **payload}

    def get_autonomy_settings(self):
        with self._lock:
            speed_level = self._drive_speed_level
            turn_level = self._drive_turn_level
        return {
            'speed_level': speed_level,
            'turn_level': turn_level,
            'max_linear_mps': AUTONOMY_LINEAR_LIMITS[speed_level],
            'max_angular_rps': AUTONOMY_ANGULAR_LIMITS[turn_level],
            'purpose': 'autonomous-navigation',
            'integer_range': [1, 9],
        }

    def _set_parameters_checked(self, client, parameters, description):
        if not client.wait_for_service(timeout_sec=1.0):
            raise ConflictError(f'{description} is unavailable')
        request = SetParametersAtomically.Request()
        request.parameters = [
            parameter.to_parameter_msg() for parameter in parameters
        ]
        response = self._wait_for_future(
            client.call_async(request),
            2.0,
            description,
        )
        if not response.result.successful:
            reason = (
                response.result.reason
                or 'node rejected the requested parameters'
            )
            raise ConflictError(f'{description} failed: {reason}')

    @staticmethod
    def _autonomy_parameter_updates(speed_level, turn_level):
        linear = AUTONOMY_LINEAR_LIMITS[speed_level]
        angular = AUTONOMY_ANGULAR_LIMITS[turn_level]
        return {
            'controller': [
                Parameter(
                    'FollowPath.desired_linear_vel',
                    Parameter.Type.DOUBLE,
                    linear,
                ),
                Parameter(
                    'FollowPath.rotate_to_heading_angular_vel',
                    Parameter.Type.DOUBLE,
                    angular,
                ),
            ],
            'smoother': [
                Parameter(
                    'max_velocity',
                    Parameter.Type.DOUBLE_ARRAY,
                    [linear, 0.0, angular],
                ),
                Parameter(
                    'min_velocity',
                    Parameter.Type.DOUBLE_ARRAY,
                    [0.0, 0.0, -angular],
                ),
            ],
            'safe_base': [
                Parameter(
                    'speed_level',
                    Parameter.Type.INTEGER,
                    speed_level,
                ),
                Parameter(
                    'turn_level',
                    Parameter.Type.INTEGER,
                    turn_level,
                ),
            ],
        }

    def set_autonomy_settings(self, value):
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        try:
            speed_level = normalize_speed_level(value.get('speed_level'))
            turn_level = normalize_speed_level(value.get('turn_level'))
        except ValueError as exc:
            raise ValidationError(str(exc)) from exc
        with self._lock:
            if self._active is not None:
                raise ConflictError(
                    'cannot change autonomous speed during an active task'
                )
            if self._estop_latched:
                raise ConflictError('emergency stop is latched')
            if self._map_switch_pending:
                raise ConflictError('map switching is still in progress')
            self._autonomy_update_pending = True
            previous_speed_level = self._drive_speed_level
            previous_turn_level = self._drive_turn_level
        self._publish_stop()
        updates = self._autonomy_parameter_updates(
            speed_level,
            turn_level,
        )
        previous = self._autonomy_parameter_updates(
            previous_speed_level,
            previous_turn_level,
        )
        nodes = (
            (
                'controller',
                self._controller_parameters,
                'Nav2 path-controller speed update',
            ),
            (
                'smoother',
                self._velocity_smoother_parameters,
                'Nav2 velocity-smoother update',
            ),
            (
                'safe_base',
                self._safe_base_parameters,
                'DOGZILLA safe-base speed update',
            ),
        )
        applied = []
        try:
            for name, client, description in nodes:
                self._set_parameters_checked(
                    client,
                    updates[name],
                    description,
                )
                applied.append((name, client, description))
            with self._lock:
                self._drive_speed_level = speed_level
                self._drive_turn_level = turn_level
        except ConflictError:
            for name, client, description in reversed(applied):
                try:
                    self._set_parameters_checked(
                        client,
                        previous[name],
                        f'{description} rollback',
                    )
                except ConflictError as exc:
                    self.get_logger().error(str(exc))
            raise
        finally:
            with self._lock:
                self._autonomy_update_pending = False
        settings = self.get_autonomy_settings()
        self.events.publish('autonomy.speed', settings)
        return settings

    def set_manual_drive(self, value):
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        direction = str(value.get('direction', '')).strip().lower()
        commands = {
            'stop': (0.0, 0.0, 0.0),
            'forward': (1.0, 0.0, 0.0),
            'backward': (-1.0, 0.0, 0.0),
            'left': (0.0, 1.0, 0.0),
            'right': (0.0, -1.0, 0.0),
            'turn-left': (0.0, 0.0, 1.0),
            'turn-right': (0.0, 0.0, -1.0),
        }
        if direction not in commands:
            raise ValidationError(
                'direction must be forward, backward, left, right, '
                'turn-left, turn-right, or stop'
            )
        if direction == 'stop':
            with self._lock:
                self._manual_command_until = 0.0
            self._priority_stop_publisher.publish(Twist())
            return {
                'direction': direction,
                'enabled': self.manual_drive_enabled,
            }
        if not self.manual_drive_enabled:
            raise ConflictError(
                'manual web driving is stored for future use but disabled'
            )
        with self._lock:
            if self._active is not None:
                raise ConflictError(
                    'manual driving is disabled during an active task'
                )
            if self._estop_latched:
                raise ConflictError('emergency stop is latched')
            if self._map_switch_pending:
                raise ConflictError('map switching is still in progress')
        battery_ready, battery_reason = self._battery_gate()
        if not battery_ready:
            raise ConflictError(battery_reason)
        linear_x, linear_y, angular_z = commands[direction]
        message = Twist()
        message.linear.x = linear_x * SPEED_LEVELS[1].max_linear
        message.linear.y = linear_y * SPEED_LEVELS[1].max_linear
        message.angular.z = angular_z * TURN_LEVELS[1].max_angular
        self._priority_stop_publisher.publish(message)
        with self._lock:
            self._manual_command_until = time.monotonic() + 0.35
        return {
            'direction': direction,
            'enabled': True,
            'speed_profile': 'fixed-conservative',
        }

    def list_locations(self):
        return self.store.list_locations(self.map_name)

    def save_location(self, value):
        payload = build_location_payload(value, default_map=self.map_name)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints([
            {
                'label': payload['name'],
                'x': payload['x'],
                'y': payload['y'],
            },
        ], self._active_keepout_zones())
        location = self.store.save_location(payload)
        self.events.publish('location.saved', location)
        return location

    def delete_location(self, location_id):
        self.store.delete_location(location_id, self.map_name)
        self.events.publish('location.deleted', {'id': location_id})

    def list_patrol_areas(self):
        return self.store.list_patrol_areas(self.map_name)

    def save_patrol_area(self, value):
        payload = build_patrol_area_payload(value, default_map=self.map_name)
        self._validate_active_map(payload)
        waypoints = self.occupancy_map.generate_patrol_waypoints(
            payload['polygon'],
            payload['spacing_m'],
            keepout_zones=self._active_keepout_zones(),
        )
        area = self.store.save_patrol_area(payload)
        area['waypoint_count'] = len(waypoints)
        self.events.publish('patrol_area.saved', area)
        return area

    def delete_patrol_area(self, area_id):
        self.store.delete_patrol_area(area_id, self.map_name)
        self.events.publish('patrol_area.deleted', {'id': area_id})

    def list_keepout_zones(self):
        return self._active_keepout_zones()

    def save_keepout_zone(self, value):
        payload = build_keepout_zone_payload(
            value, default_map=self.map_name
        )
        self._validate_active_map(payload)
        self.occupancy_map.validate_polygon_bounds(
            payload['polygon'], 'Keepout zone'
        )
        zone = self.store.save_keepout_zone(payload)
        if self.occupancy_map.available():
            self._publish_keepout_filter()
        self.events.publish('keepout_zone.saved', zone)
        return zone

    def delete_keepout_zone(self, zone_id):
        self.store.delete_keepout_zone(zone_id, self.map_name)
        if self.occupancy_map.available():
            self._publish_keepout_filter()
        self.events.publish('keepout_zone.deleted', {'id': zone_id})

    def _patrol_area_and_waypoints(self, value):
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        area_id = str(value.get('patrol_area_id', '')).strip()
        if not area_id:
            raise ValidationError('patrol_area_id is required')
        area = self.store.get_patrol_area(area_id, self.map_name)
        if area is None:
            raise KeyError(area_id)
        waypoints = self.occupancy_map.generate_patrol_waypoints(
            area['polygon'],
            area['spacing_m'],
            keepout_zones=self._active_keepout_zones(),
        )
        return area, waypoints

    def preview_patrol(self, value):
        """Return deterministic coverage points without moving the robot."""
        if isinstance(value, dict) and value.get('patrol_area_id'):
            area, waypoints = self._patrol_area_and_waypoints(value)
        else:
            payload = build_patrol_area_payload(
                value, default_map=self.map_name
            )
            self._validate_active_map(payload)
            area = {**payload, 'id': None}
            waypoints = self.occupancy_map.generate_patrol_waypoints(
                area['polygon'],
                area['spacing_m'],
                keepout_zones=self._active_keepout_zones(),
            )
        distance = sum(
            math.hypot(
                current['x'] - previous['x'],
                current['y'] - previous['y'],
            )
            for previous, current in zip(waypoints, waypoints[1:])
        )
        return {
            'map': self.map_name,
            'area': area,
            'waypoints': waypoints,
            'waypoint_count': len(waypoints),
            'coverage_distance_m': round(distance, 3),
            'generated_at': utc_now(),
        }

    def create_patrol(self, value):
        area, waypoints = self._patrol_area_and_waypoints(value)
        payload = build_patrol_payload(value, area, waypoints)
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

    def list_hazards(self, limit=100):
        return self.store.list_hazards(self.map_name, limit)

    def list_alerts(self, limit=25):
        alerts = self.store.list_vision_alerts(self.map_name, limit)
        return [self._public_alert(alert) for alert in alerts]

    def get_alert_photo(self, alert_id):
        alert = self.store.get_vision_alert(alert_id)
        if alert is None or alert['map'] != self.map_name:
            raise KeyError(alert_id)
        photo_name = alert.get('photo_name')
        if not photo_name or not self._valid_alert_photo_name(photo_name):
            raise KeyError(alert_id)
        path = self.alert_directory / photo_name
        try:
            body = path.read_bytes()
        except FileNotFoundError as exc:
            raise KeyError(alert_id) from exc
        if (
            not body
            or len(body) > 4 * 1024 * 1024
            or not body.startswith(b'\xff\xd8')
            or not body.endswith(b'\xff\xd9')
        ):
            raise KeyError(alert_id)
        return body

    def _validate_active_map(self, payload):
        if payload['map'] != self.map_name:
            raise ValidationError(
                f"gateway is configured for map '{self.map_name}', not "
                f"'{payload['map']}'"
            )

    def create_delivery(self, value):
        payload = build_delivery_payload(value)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints(
            payload['waypoints'], self._active_keepout_zones()
        )
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

    def create_route(self, value):
        payload = build_route_payload(value)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints(
            payload['waypoints'], self._active_keepout_zones()
        )
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

    @staticmethod
    def _wait_for_future(future, timeout, description):
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(float(timeout)):
            raise ConflictError(f'{description} timed out')
        try:
            return future.result()
        except Exception as exc:
            raise ConflictError(f'{description} failed: {exc}') from exc

    def preview_route(self, value):
        """Ask Nav2 for a non-executing path through validated waypoints."""
        payload = build_route_payload(value)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints(
            payload['waypoints'], self._active_keepout_zones()
        )
        if not self._planner.server_is_ready():
            raise ConflictError('Nav2 path planner is unavailable')
        pose = self.telemetry.get('pose', stale_after=3.0)
        if pose is None or pose['stale']:
            raise ConflictError('fresh map-frame localization is unavailable')

        start = pose['value']
        path_points = []
        total_distance = 0.0
        for waypoint in payload['waypoints']:
            goal = ComputePathToPose.Goal()
            goal.use_start = True
            goal.start.header.frame_id = 'map'
            goal.start.header.stamp = self.get_clock().now().to_msg()
            goal.start.pose.position.x = start['x']
            goal.start.pose.position.y = start['y']
            goal.start.pose.orientation.z = math.sin(start['yaw'] / 2.0)
            goal.start.pose.orientation.w = math.cos(start['yaw'] / 2.0)
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = goal.start.header.stamp
            goal.goal.pose.position.x = waypoint['x']
            goal.goal.pose.position.y = waypoint['y']
            goal.goal.pose.orientation.z = math.sin(waypoint['yaw'] / 2.0)
            goal.goal.pose.orientation.w = math.cos(waypoint['yaw'] / 2.0)

            goal_handle = self._wait_for_future(
                self._planner.send_goal_async(goal),
                3.0,
                'Nav2 path request',
            )
            if goal_handle is None or not goal_handle.accepted:
                raise ConflictError('Nav2 rejected the path preview request')
            result = self._wait_for_future(
                goal_handle.get_result_async(),
                5.0,
                'Nav2 path calculation',
            )
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise ConflictError(
                    f'Nav2 path preview failed with status {result.status}'
                )
            poses = result.result.path.poses
            segment = [
                {
                    'x': round(float(item.pose.position.x), 4),
                    'y': round(float(item.pose.position.y), 4),
                }
                for item in poses
            ]
            for previous, current in zip(segment, segment[1:]):
                total_distance += math.hypot(
                    current['x'] - previous['x'],
                    current['y'] - previous['y'],
                )
            if path_points and segment:
                segment = segment[1:]
            path_points.extend(segment)
            start = waypoint

        if not path_points:
            raise ConflictError('Nav2 returned an empty path preview')
        if len(path_points) > 3000:
            stride = math.ceil(len(path_points) / 3000)
            path_points = path_points[::stride]
        return {
            'map': self.map_name,
            'generated_at': utc_now(),
            'distance_m': round(total_distance, 3),
            'path': path_points,
            'waypoints': payload['waypoints'],
        }

    def cancel_task(self, task_id):
        task = self.store.get(task_id)
        if task is None:
            raise KeyError(task_id)
        if task['state'] in {'completed', 'failed', 'cancelled'}:
            raise ConflictError(f"task is already {task['state']}")
        with self._lock:
            self._cancel_requests.add(task_id)
            self._cancel_reasons[task_id] = 'Cancelled by operator'
        if task['state'] == 'queued':
            task = self.store.update(
                task_id,
                state='cancelled',
                error='Cancelled by operator before execution',
            )
            with self._lock:
                self._cancel_requests.discard(task_id)
                self._cancel_reasons.pop(task_id, None)
            self.events.publish('task.cancelled', task)
            return task
        task = self.store.update(task_id, state='cancelling')
        self.events.publish('task.cancelling', task)
        return task

    def emergency_stop(self):
        with self._lock:
            self._estop_latched = True
            if self._active:
                task_id = self._active['task_id']
                self._cancel_requests.add(task_id)
                self._cancel_reasons[task_id] = 'Emergency stop activated'
        self.events.publish('safety.estop', {'latched': True})
        self._publish_estop(True)
        self._publish_stop()
        return self.get_state()['safety']

    def reset_estop(self):
        battery = self.telemetry.get('battery', stale_after=12.0)
        if battery is None or battery['stale']:
            raise ConflictError(
                'cannot reset emergency stop without fresh battery telemetry'
            )
        value = battery['value']
        if not value['present'] or value['percentage'] is None:
            raise ConflictError(
                'cannot reset emergency stop with invalid battery telemetry'
            )
        if value['percentage'] < self.minimum_battery:
            raise ConflictError(
                'cannot reset emergency stop below the task battery minimum'
            )
        with self._lock:
            self._estop_latched = False
        self.events.publish('safety.estop', {'latched': False})
        self._publish_estop(False)
        return self.get_state()['safety']

    def _publish_estop(self, latched):
        self._estop_publisher.publish(Bool(data=bool(latched)))

    def _publish_stop(self):
        zero = Twist()
        self._priority_stop_publisher.publish(zero)
        self._direct_stop_publisher.publish(zero)

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            estop_latched = self._estop_latched
            active = self._active
            manual_expired = (
                self._manual_command_until > 0.0
                and now >= self._manual_command_until
            )
            if manual_expired:
                self._manual_command_until = 0.0
        if manual_expired:
            self._priority_stop_publisher.publish(Twist())
        if estop_latched or now < self._stop_until:
            self._publish_stop()
        if estop_latched:
            self._publish_estop(True)

        if active is not None:
            task_id = active['task_id']
            battery_ready, battery_reason = self._battery_gate()
            patrol_vision_ready = True
            patrol_vision_reason = 'ready'
            if active['payload']['kind'] == 'patrol':
                patrol_vision_ready, patrol_vision_reason = (
                    self._patrol_vision_gate()
                )
            with self._lock:
                safety_stop = (
                    (not battery_ready or not patrol_vision_ready)
                    and not self._estop_latched
                )
                safety_reason = (
                    battery_reason if not battery_ready
                    else patrol_vision_reason
                )
                if safety_stop:
                    self._estop_latched = True
                    self._cancel_requests.add(task_id)
                    self._cancel_reasons[task_id] = safety_reason
                cancel_requested = task_id in self._cancel_requests
            if safety_stop:
                self._publish_estop(True)
                self.events.publish(
                    'safety.estop',
                    {
                        'latched': True,
                        'reason': safety_reason,
                    },
                )
            if cancel_requested:
                self._request_active_cancel()
                return
            dwell_until = active.get('dwell_until')
            if dwell_until is not None and now >= dwell_until:
                with self._lock:
                    if self._active:
                        self._active['dwell_until'] = None
                self._send_current_waypoint()
            return

        if estop_latched:
            return
        task = self.store.next_queued()
        if task is None:
            return
        ready, _ = self._task_gate(task)
        if not ready:
            return
        self._begin_task(task)

    def _begin_task(self, task):
        active = {
            'task_id': task['id'],
            'payload': task['payload'],
            'step': 0,
            'cycle': 0,
            'goal_handle': None,
            'sending': False,
            'cancel_sent': False,
            'dwell_until': None,
        }
        with self._lock:
            if self._active is not None or self._estop_latched:
                return
            self._active = active
        task = self.store.update(
            task['id'],
            state='running',
            current_step=0,
        )
        self.events.publish('task.started', task)
        self._send_current_waypoint()

    def _send_current_waypoint(self):
        with self._lock:
            if self._active is None or self._active['sending']:
                return
            active = dict(self._active)
            if active['goal_handle'] is not None:
                return
            waypoints = active['payload']['waypoints']
            step = active['step']
            if step >= len(waypoints):
                self._finish_active('completed')
                return
            self._active['sending'] = True
        waypoint = waypoints[step]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['x']
        goal.pose.pose.position.y = waypoint['y']
        goal.pose.pose.orientation.z = math.sin(waypoint['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint['yaw'] / 2.0)
        try:
            future = self._navigate.send_goal_async(goal)
        except Exception as exc:
            self._finish_active('failed', f'Nav2 goal send failed: {exc}')
            return
        future.add_done_callback(self._on_goal_response)
        self.events.publish(
            'task.waypoint_sent',
            {
                'task_id': active['task_id'],
                'step': step,
                'waypoint': waypoint,
            },
        )

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_active('failed', f'Nav2 goal send failed: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_active('failed', 'Nav2 rejected the waypoint goal')
            return
        with self._lock:
            if self._active is None:
                goal_handle.cancel_goal_async()
                return
            self._active['goal_handle'] = goal_handle
            self._active['sending'] = False
            cancel_requested = (
                self._active['task_id'] in self._cancel_requests
                or self._estop_latched
            )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)
        if cancel_requested:
            self._request_active_cancel()

    def _on_goal_result(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._finish_active('failed', f'Nav2 result failed: {exc}')
            return
        with self._lock:
            if self._active is None:
                return
            task_id = self._active['task_id']
            cancel_requested = (
                task_id in self._cancel_requests or self._estop_latched
            )
            cancel_reason = self._cancel_reasons.get(
                task_id,
                'Cancelled by operator',
            )
            waypoint = self._active['payload']['waypoints'][
                self._active['step']
            ]
            waypoint_count = len(self._active['payload']['waypoints'])
            repeats = int(self._active['payload'].get('repeats', 1))
            cycle = self._active['cycle']
            self._active['goal_handle'] = None
            self._active['sending'] = False
        if cancel_requested or status == GoalStatus.STATUS_CANCELED:
            self._finish_active('cancelled', cancel_reason)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            status_name = NAV2_STATUS_NAMES.get(status, 'unrecognized')
            self._finish_active(
                'failed',
                f'Nav2 waypoint {status_name} (status {status})',
            )
            return
        self.events.publish(
            'task.waypoint_reached',
            {'task_id': task_id, 'waypoint': waypoint},
        )
        dwell = waypoint['dwell_seconds']
        with self._lock:
            if self._active is None:
                return
            self._active['step'] += 1
            next_step = self._active['step']
            next_cycle = self._active['cycle']
            if next_step >= waypoint_count and next_cycle + 1 < repeats:
                next_cycle += 1
                self._active['cycle'] = next_cycle
                self._active['step'] = 0
                next_step = 0
            self._active['dwell_until'] = (
                time.monotonic() + dwell if dwell > 0 else None
            )
        completed_steps = next_cycle * waypoint_count + next_step
        self.store.update(task_id, current_step=completed_steps)
        if next_step >= waypoint_count:
            self._finish_active('completed')
        elif next_cycle > cycle:
            self.events.publish(
                'task.patrol_cycle_completed',
                {
                    'task_id': task_id,
                    'completed_cycle': cycle + 1,
                    'total_cycles': repeats,
                },
            )
            if dwell <= 0:
                self._send_current_waypoint()
        elif dwell <= 0:
            self._send_current_waypoint()

    def _request_active_cancel(self):
        with self._lock:
            if self._active is None:
                return
            goal_handle = self._active['goal_handle']
            sending = self._active['sending']
            cancel_reason = self._cancel_reasons.get(
                self._active['task_id'],
                'Cancelled by operator',
            )
            should_cancel_goal = (
                goal_handle is not None
                and not self._active['cancel_sent']
            )
            if should_cancel_goal:
                self._active['cancel_sent'] = True
        self._stop_until = time.monotonic() + 1.0
        self._publish_stop()
        if should_cancel_goal:
            try:
                cancel_future = goal_handle.cancel_goal_async()
            except Exception as exc:
                self._cancel_failed(f'Nav2 cancellation request failed: {exc}')
                return
            cancel_future.add_done_callback(self._on_cancel_response)
        elif not sending:
            if goal_handle is None:
                self._finish_active('cancelled', cancel_reason)

    def _on_cancel_response(self, future):
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as exc:
            self._cancel_failed(f'Nav2 cancellation response failed: {exc}')
            return
        if not accepted:
            self._cancel_failed('Nav2 did not accept the cancellation request')

    def _cancel_failed(self, message):
        with self._lock:
            if self._active is None:
                return
            self._estop_latched = True
        self.events.publish(
            'safety.estop',
            {'latched': True, 'reason': message},
        )
        self._publish_estop(True)
        self._finish_active('failed', message)

    def _finish_active(self, state, error=None):
        with self._lock:
            if self._active is None:
                return
            task_id = self._active['task_id']
            self._active = None
            self._cancel_requests.discard(task_id)
            self._cancel_reasons.pop(task_id, None)
        if state in {'cancelled', 'failed'}:
            self._stop_until = time.monotonic() + 1.0
            self._publish_stop()
        task = self.store.update(task_id, state=state, error=error or '')
        self.events.publish(f'task.{state}', task)

    def close(self):
        with self._lock:
            self._estop_latched = True
        if rclpy.ok():
            for _ in range(3):
                self._publish_estop(True)
                self._publish_stop()
        self.store.close()


def main(args=None):
    rclpy.init(args=args)
    node = DogzillaWebGateway()
    static_directory = Path(__file__).resolve().parent / 'web_static'
    server = GatewayHTTPServer(
        (node.web_host, node.web_port),
        node,
        node.web_password,
        static_directory,
        legacy_token=node.web_legacy_token,
    )
    server_thread = threading.Thread(
        target=server.serve_forever,
        name='dogzilla-web-http',
        daemon=True,
    )
    server_thread.start()
    node.get_logger().info(
        f'DOGZILLA web gateway listening on '
        f'{node.web_host}:{node.web_port}'
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.close()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
