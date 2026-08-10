"""ROS-aware monitoring and autonomous-task gateway for DOGZILLA."""

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import os
from pathlib import Path
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState

from .web_core import build_delivery_payload
from .web_core import build_route_payload
from .web_core import ConflictError
from .web_core import EventBus
from .web_core import MAP_NAME_PATTERN
from .web_core import TaskStore
from .web_core import TelemetryCache
from .web_core import utc_now
from .web_core import ValidationError
from .web_http import GatewayHTTPServer


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
        database_path = os.environ.get(
            'DOGZILLA_WEB_DATABASE',
            '/data/tasks.sqlite3',
        )
        self.store = TaskStore(database_path)

        self._estop_latched = False
        self._active = None
        self._cancel_requests = set()
        self._cancel_reasons = {}
        self._stop_until = 0.0
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
            OccupancyGrid,
            '/map',
            self._on_map,
            map_qos,
        )
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
        self._navigate = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._task_timer = self.create_timer(0.10, self._tick)
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
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.telemetry.update(
            'pose',
            {
                'frame': message.header.frame_id,
                'x': round(float(message.pose.pose.position.x), 4),
                'y': round(float(message.pose.pose.position.y), 4),
                'yaw': round(yaw, 4),
                'linear_speed': round(
                    math.hypot(
                        float(message.twist.twist.linear.x),
                        float(message.twist.twist.linear.y),
                    ),
                    4,
                ),
                'angular_speed': round(
                    float(message.twist.twist.angular.z),
                    4,
                ),
            },
        )

    def _on_map(self, message):
        self.telemetry.update(
            'map',
            {
                'name': self.map_name,
                'frame': message.header.frame_id,
                'width': int(message.info.width),
                'height': int(message.info.height),
                'resolution': round(float(message.info.resolution), 5),
            },
        )

    def _refresh_graph(self):
        node_names = sorted(set(self.get_node_names()))
        nav_available = self._navigate.server_is_ready()
        if nav_available or any('bt_navigator' in name for name in node_names):
            mode = 'navigation'
        elif any('cartographer' in name for name in node_names):
            mode = 'mapping_or_localization'
        elif any('dogzilla_safe_base' in name for name in node_names):
            mode = 'drive'
        else:
            mode = 'stopped'
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

    def _task_gate(self):
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
        map_state = self.telemetry.get('map', stale_after=30.0)
        if map_state is None:
            return False, 'map telemetry is unavailable'
        return True, 'ready'

    def get_state(self):
        ready, gate_reason = self._task_gate()
        with self._lock:
            active_task_id = self._active['task_id'] if self._active else None
            graph = dict(self._graph)
            estop_latched = self._estop_latched
        active_task = self.store.get(active_task_id) if active_task_id else None
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
            },
            'active_task': active_task,
        }

    def list_tasks(self, limit=100):
        return self.store.list(limit)

    def get_task(self, task_id):
        return self.store.get(task_id)

    def _validate_active_map(self, payload):
        if payload['map'] != self.map_name:
            raise ValidationError(
                f"gateway is configured for map '{self.map_name}', not "
                f"'{payload['map']}'"
            )

    def create_delivery(self, value):
        payload = build_delivery_payload(value)
        self._validate_active_map(payload)
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

    def create_route(self, value):
        payload = build_route_payload(value)
        self._validate_active_map(payload)
        task = self.store.create(payload)
        self.events.publish('task.created', task)
        return task

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
        return self.get_state()['safety']

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

        if active is not None:
            task_id = active['task_id']
            battery_ready, battery_reason = self._battery_gate()
            with self._lock:
                battery_stop = not battery_ready and not self._estop_latched
                if battery_stop:
                    self._estop_latched = True
                    self._cancel_requests.add(task_id)
                    self._cancel_reasons[task_id] = battery_reason
                cancel_requested = task_id in self._cancel_requests
            if battery_stop:
                self.events.publish(
                    'safety.estop',
                    {
                        'latched': True,
                        'reason': battery_reason,
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
        ready, _ = self._task_gate()
        if not ready:
            return
        self._begin_task(task)

    def _begin_task(self, task):
        active = {
            'task_id': task['id'],
            'payload': task['payload'],
            'step': 0,
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
            self._active['dwell_until'] = (
                time.monotonic() + dwell if dwell > 0 else None
            )
        self.store.update(task_id, current_step=next_step)
        if next_step >= waypoint_count:
            self._finish_active('completed')
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
