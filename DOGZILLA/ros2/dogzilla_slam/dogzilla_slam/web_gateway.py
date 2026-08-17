"""ROS-aware monitoring and autonomous-task gateway for DOGZILLA."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import json
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, CompressedImage, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .web_core import build_location_payload
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
from .vision_core import validate_request
from .vision_core import VisionConfigurationError


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
        self.web_token = os.environ.get('DOGZILLA_WEB_TOKEN', '').strip()
        if len(self.web_token) < 24:
            raise RuntimeError(
                'DOGZILLA_WEB_TOKEN must contain at least 24 characters'
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
        self.hazard_minimum_confidence = self._float_environment(
            'DOGZILLA_WEB_HAZARD_CONFIDENCE',
            0.55,
            0.30,
            0.99,
        )
        self.hazard_confirmations = self._integer_environment(
            'DOGZILLA_WEB_HAZARD_CONFIRMATIONS',
            3,
            2,
            10,
        )
        database_path = os.environ.get(
            'DOGZILLA_WEB_DATABASE',
            '/data/tasks.sqlite3',
        )
        self.store = TaskStore(database_path)
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
        )

        self._estop_latched = False
        self._active = None
        self._cancel_requests = set()
        self._cancel_reasons = {}
        self._stop_until = 0.0
        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._waiting_for_map_pose = False
        self._vision_frame = None
        self._vision_frame_received = 0.0
        self._vision_status_signature = None
        self._vision_action_status_signature = None
        self._hazard_candidate = None
        self._hazard_candidate_count = 0
        self._hazard_candidate_time = 0.0
        self._last_hazard_signature = None
        self._last_hazard_time = 0.0
        self._graph = {
            'mode': 'stopped',
            'nodes': [],
            'nav_available': False,
            'updated_at': utc_now(),
        }

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
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
        self.create_subscription(Odometry, '/odom', self._on_odometry, 10)
        self.create_subscription(
            String,
            '/vision/detections',
            self._on_vision_detections,
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
            {'map': self.map_name, 'minimum_battery': self.minimum_battery},
        )

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

    def log_http(self, message):
        self.get_logger().debug(message)

    def log_exception(self, message, exception):
        self.get_logger().error(f'{message}: {exception}')

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
        if value.get('mode') in {'objects', 'floor-hazards'}:
            try:
                detections = [
                    validate_detection_payload(item) for item in detections
                ]
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(f'Invalid object detection ignored: {exc}')
                return
            value['detections'] = detections
        self.telemetry.update('vision', value)
        if value.get('mode') == 'floor-hazards':
            self._observe_floor_hazards(detections)

    def _observe_floor_hazards(self, detections):
        """Debounce model results before recording or stopping a patrol."""
        hazards = [
            item for item in detections
            if item.get('floor_hazard') is True
            and item['confidence'] >= self.hazard_minimum_confidence
        ]
        now = time.monotonic()
        if not hazards:
            with self._lock:
                self._hazard_candidate = None
                self._hazard_candidate_count = 0
            return
        strongest = max(hazards, key=lambda item: item['confidence'])
        signature = strongest['label']
        with self._lock:
            continuing = (
                signature == self._hazard_candidate
                and now - self._hazard_candidate_time <= 1.5
            )
            self._hazard_candidate_count = (
                self._hazard_candidate_count + 1 if continuing else 1
            )
            self._hazard_candidate = signature
            self._hazard_candidate_time = now
            confirmed = self._hazard_candidate_count >= self.hazard_confirmations
            duplicate = (
                signature == self._last_hazard_signature
                and now - self._last_hazard_time < 8.0
            )
            if confirmed and not duplicate:
                self._last_hazard_signature = signature
                self._last_hazard_time = now
                self._hazard_candidate_count = 0
        if confirmed and not duplicate:
            self._handle_confirmed_hazard(strongest)

    def _handle_confirmed_hazard(self, detection):
        pose = self.telemetry.get('pose', stale_after=3.0)
        robot_pose = None if pose is None or pose['stale'] else pose['value']
        with self._lock:
            active_task_id = self._active['task_id'] if self._active else None
            active_kind = (
                self._active['payload']['kind'] if self._active else None
            )
        observation = self.store.record_hazard({
            'task_id': active_task_id,
            'map': self.map_name,
            'label': detection['label'],
            'risk': detection.get('risk', 'danger'),
            'confidence': detection['confidence'],
            'box': detection['box'],
            'robot_pose': robot_pose,
        })
        observation['position_semantics'] = (
            'robot pose when observed; not the object position'
        )
        self.events.publish('hazard.confirmed', observation)
        if active_kind != 'patrol':
            return
        reason = (
            f"Confirmed floor hazard: {detection['label']} "
            f"({detection['confidence']:.0%})"
        )
        with self._lock:
            self._estop_latched = True
            self._cancel_requests.add(active_task_id)
            self._cancel_reasons[active_task_id] = reason
        self._publish_estop(True)
        self._publish_stop()
        self.events.publish(
            'safety.estop',
            {'latched': True, 'reason': reason, 'observation': observation},
        )

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
        with self._lock:
            linear_speed = self._linear_speed
            angular_speed = self._angular_speed
        self.telemetry.update(
            'pose',
            {
                'frame': transform.header.frame_id or 'map',
                'x': round(float(translation.x), 4),
                'y': round(float(translation.y), 4),
                'yaw': round(self._quaternion_yaw(rotation), 4),
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
        self.telemetry.update('map', summary)
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
        if task is not None and task['kind'] == 'patrol':
            return self._patrol_vision_gate()
        return True, 'ready'

    def _patrol_vision_gate(self):
        vision = self.telemetry.get('vision_status', stale_after=5.0)
        if vision is None or vision['stale']:
            return False, 'fresh floor-hazard vision status is unavailable'
        return patrol_vision_readiness(vision['value'])

    def get_state(self):
        ready, gate_reason = self._task_gate()
        with self._lock:
            active_task_id = self._active['task_id'] if self._active else None
            graph = dict(self._graph)
            estop_latched = self._estop_latched
        active_task = (
            self.store.get(active_task_id)
            if active_task_id
            else None
        )
        return {
            'time': utc_now(),
            'configuration': {'map': self.map_name},
            'robot': graph,
            'telemetry': self.telemetry.snapshot(stale_after=10.0),
            'safety': {
                'estop_latched': estop_latched,
                'minimum_task_battery': self.minimum_battery,
                'task_ready': ready,
                'task_gate_reason': gate_reason,
                'hazard_confidence': self.hazard_minimum_confidence,
                'hazard_confirmations': self.hazard_confirmations,
            },
            'active_task': active_task,
        }

    def list_tasks(self, limit=100):
        return self.store.list(limit)

    def get_task(self, task_id):
        return self.store.get(task_id)

    def get_map(self):
        return self.occupancy_map.payload()

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
        if patrol_active and requested['mode'] != 'floor-hazards':
            raise ConflictError(
                'cannot leave floor-hazards mode during an active patrol'
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
        ])
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
            payload['polygon'], payload['spacing_m']
        )
        area = self.store.save_patrol_area(payload)
        area['waypoint_count'] = len(waypoints)
        self.events.publish('patrol_area.saved', area)
        return area

    def delete_patrol_area(self, area_id):
        self.store.delete_patrol_area(area_id, self.map_name)
        self.events.publish('patrol_area.deleted', {'id': area_id})

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
            area['polygon'], area['spacing_m']
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
                area['polygon'], area['spacing_m']
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

    def _validate_active_map(self, payload):
        if payload['map'] != self.map_name:
            raise ValidationError(
                f"gateway is configured for map '{self.map_name}', not "
                f"'{payload['map']}'"
            )

    def create_delivery(self, value):
        payload = build_delivery_payload(value)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints(payload['waypoints'])
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

    def create_route(self, value):
        payload = build_route_payload(value)
        self._validate_active_map(payload)
        self.occupancy_map.validate_waypoints(payload['waypoints'])
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
        self.occupancy_map.validate_waypoints(payload['waypoints'])
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
            self._finish_active(
                'failed',
                f'Nav2 waypoint finished with status {status}',
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
        node.web_token,
        static_directory,
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
