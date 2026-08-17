"""Pure-Python state, validation, persistence, and event helpers for the web UI."""

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sqlite3
import threading
import time
import uuid


MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9._-]{1,64}$')
TASK_STATES = {
    'queued',
    'running',
    'cancelling',
    'completed',
    'failed',
    'cancelled',
}


class ValidationError(ValueError):
    """Raised when an API payload is not safe to execute."""


class ConflictError(RuntimeError):
    """Raised when the requested state transition is not currently allowed."""


def classify_robot_mode(node_names, nav_available=False):
    """Return the operator-facing mode represented by the current ROS graph."""
    names = tuple(str(name) for name in node_names)
    if bool(nav_available) or any('bt_navigator' in name for name in names):
        return 'navigation'
    if any('cartographer' in name for name in names):
        return 'mapping_or_localization'
    has_base = any('dogzilla_safe_base' in name for name in names)
    has_vision = any('dogzilla_vision' in name for name in names)
    if has_base and has_vision:
        return 'vision_control'
    if has_base:
        return 'drive'
    if has_vision:
        return 'vision'
    return 'stopped'


def utc_now():
    """Return a compact, sortable UTC timestamp."""
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace(
        '+00:00',
        'Z',
    )


def _finite_number(value, field_name, lower, upper):
    if isinstance(value, bool):
        raise ValidationError(f'{field_name} must be a number')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f'{field_name} must be a number') from exc
    if not math.isfinite(result):
        raise ValidationError(f'{field_name} must be finite')
    if not lower <= result <= upper:
        raise ValidationError(
            f'{field_name} must be between {lower:g} and {upper:g}'
        )
    return result


def _clean_name(value, field_name='name', maximum=80):
    if value is None:
        return ''
    if not isinstance(value, str):
        raise ValidationError(f'{field_name} must be text')
    result = value.strip()
    if len(result) > maximum:
        raise ValidationError(
            f'{field_name} must contain at most {maximum} characters'
        )
    return result


def validate_waypoint(value, index=0):
    """Validate and normalize one map-frame navigation waypoint."""
    if not isinstance(value, dict):
        raise ValidationError(f'waypoints[{index}] must be an object')
    return {
        'label': _clean_name(
            value.get('label', f'Waypoint {index + 1}'),
            f'waypoints[{index}].label',
        ) or f'Waypoint {index + 1}',
        'x': _finite_number(
            value.get('x'),
            f'waypoints[{index}].x',
            -100.0,
            100.0,
        ),
        'y': _finite_number(
            value.get('y'),
            f'waypoints[{index}].y',
            -100.0,
            100.0,
        ),
        'yaw': _finite_number(
            value.get('yaw', 0.0),
            f'waypoints[{index}].yaw',
            -math.pi,
            math.pi,
        ),
        'dwell_seconds': _finite_number(
            value.get('dwell_seconds', 0.0),
            f'waypoints[{index}].dwell_seconds',
            0.0,
            300.0,
        ),
    }


def _validate_common_task(value):
    if not isinstance(value, dict):
        raise ValidationError('request body must be a JSON object')
    map_name = _clean_name(value.get('map', 'test1'), 'map', maximum=64)
    if not MAP_NAME_PATTERN.fullmatch(map_name):
        raise ValidationError(
            'map may contain only letters, numbers, dot, underscore, and dash'
        )
    return {
        'map': map_name,
        'name': _clean_name(value.get('name', ''), 'name') or 'Untitled task',
    }


def build_delivery_payload(value):
    """Normalize a two-stop pickup/drop-off delivery request."""
    common = _validate_common_task(value)
    if 'pickup' not in value or 'dropoff' not in value:
        raise ValidationError('delivery requires pickup and dropoff waypoints')
    pickup = dict(value['pickup']) if isinstance(value['pickup'], dict) else value['pickup']
    dropoff = dict(value['dropoff']) if isinstance(value['dropoff'], dict) else value['dropoff']
    if isinstance(pickup, dict):
        pickup.setdefault('label', 'Pickup')
    if isinstance(dropoff, dict):
        dropoff.setdefault('label', 'Drop-off')
    return {
        **common,
        'kind': 'delivery',
        'waypoints': [
            validate_waypoint(pickup, 0),
            validate_waypoint(dropoff, 1),
        ],
    }


def build_route_payload(value):
    """Normalize a generic ordered waypoint route."""
    common = _validate_common_task(value)
    waypoints = value.get('waypoints')
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 20:
        raise ValidationError('route requires between 1 and 20 waypoints')
    return {
        **common,
        'kind': 'route',
        'waypoints': [
            validate_waypoint(waypoint, index)
            for index, waypoint in enumerate(waypoints)
        ],
    }


def _validate_map_name(value, default='test1'):
    map_name = _clean_name(value if value is not None else default, 'map', 64)
    if not MAP_NAME_PATTERN.fullmatch(map_name):
        raise ValidationError(
            'map may contain only letters, numbers, dot, underscore, and dash'
        )
    return map_name


def _segments_intersect(first, second, third, fourth):
    """Return whether two closed 2-D line segments intersect."""

    def orientation(a, b, c):
        value = (
            (b['x'] - a['x']) * (c['y'] - a['y'])
            - (b['y'] - a['y']) * (c['x'] - a['x'])
        )
        if abs(value) < 1e-9:
            return 0
        return 1 if value > 0 else -1

    def on_segment(a, b, c):
        return (
            min(a['x'], c['x']) - 1e-9 <= b['x']
            <= max(a['x'], c['x']) + 1e-9
            and min(a['y'], c['y']) - 1e-9 <= b['y']
            <= max(a['y'], c['y']) + 1e-9
        )

    values = (
        orientation(first, second, third),
        orientation(first, second, fourth),
        orientation(third, fourth, first),
        orientation(third, fourth, second),
    )
    if values[0] != values[1] and values[2] != values[3]:
        return True
    return (
        (values[0] == 0 and on_segment(first, third, second))
        or (values[1] == 0 and on_segment(first, fourth, second))
        or (values[2] == 0 and on_segment(third, first, fourth))
        or (values[3] == 0 and on_segment(third, second, fourth))
    )


def validate_patrol_polygon(value):
    """Validate a simple map-frame polygon used to generate patrol coverage."""
    if not isinstance(value, list) or not 3 <= len(value) <= 12:
        raise ValidationError('patrol polygon requires between 3 and 12 points')
    points = []
    for index, point in enumerate(value):
        if not isinstance(point, dict):
            raise ValidationError(f'polygon[{index}] must be an object')
        points.append({
            'x': _finite_number(
                point.get('x'), f'polygon[{index}].x', -100.0, 100.0
            ),
            'y': _finite_number(
                point.get('y'), f'polygon[{index}].y', -100.0, 100.0
            ),
        })

    for index, point in enumerate(points):
        for other in points[index + 1:]:
            if math.hypot(point['x'] - other['x'], point['y'] - other['y']) < 0.05:
                raise ValidationError('patrol polygon points must be 0.05 m apart')

    point_count = len(points)
    for index in range(point_count):
        first = points[index]
        second = points[(index + 1) % point_count]
        for other_index in range(index + 1, point_count):
            if other_index in {index, (index + 1) % point_count}:
                continue
            if (other_index + 1) % point_count in {index, (index + 1) % point_count}:
                continue
            third = points[other_index]
            fourth = points[(other_index + 1) % point_count]
            if _segments_intersect(first, second, third, fourth):
                raise ValidationError('patrol polygon must not cross itself')

    signed_double_area = sum(
        point['x'] * points[(index + 1) % point_count]['y']
        - points[(index + 1) % point_count]['x'] * point['y']
        for index, point in enumerate(points)
    )
    area = abs(signed_double_area) / 2.0
    if not 0.25 <= area <= 250.0:
        raise ValidationError('patrol polygon area must be between 0.25 and 250 m^2')
    return points


def build_patrol_area_payload(value, default_map='test1'):
    """Normalize one reusable polygon and its coverage spacing."""
    if not isinstance(value, dict):
        raise ValidationError('request body must be a JSON object')
    name = _clean_name(value.get('name'), 'name')
    if not name:
        raise ValidationError('patrol area name is required')
    return {
        'map': _validate_map_name(value.get('map'), default_map),
        'name': name,
        'polygon': validate_patrol_polygon(value.get('polygon')),
        'spacing_m': _finite_number(
            value.get('spacing_m', 0.6), 'spacing_m', 0.3, 3.0
        ),
    }


def build_patrol_payload(value, area, waypoints):
    """Build an executable patrol task from a saved area and safe route."""
    if not isinstance(value, dict):
        raise ValidationError('request body must be a JSON object')
    if not isinstance(area, dict):
        raise ValidationError('saved patrol area is required')
    repeats = int(_finite_number(value.get('repeats', 1), 'repeats', 1, 20))
    if float(value.get('repeats', 1)) != repeats:
        raise ValidationError('repeats must be a whole number')
    dwell = _finite_number(
        value.get('dwell_seconds', 0.0), 'dwell_seconds', 0.0, 30.0
    )
    normalized_waypoints = []
    for index, waypoint in enumerate(waypoints):
        item = dict(waypoint)
        item['dwell_seconds'] = dwell
        normalized_waypoints.append(validate_waypoint(item, index))
    if not 2 <= len(normalized_waypoints) <= 120:
        raise ValidationError('patrol route requires between 2 and 120 waypoints')
    return {
        'kind': 'patrol',
        'name': _clean_name(value.get('name'), 'name') or area['name'],
        'map': area['map'],
        'patrol_area_id': area['id'],
        'patrol_area_name': area['name'],
        'repeats': repeats,
        'waypoints': normalized_waypoints,
    }


def patrol_vision_readiness(status):
    """Fail closed unless patrol has complete, non-actuating hazard vision."""
    if not isinstance(status, dict):
        return False, 'floor-hazard vision status is invalid'
    if status.get('state') != 'ready':
        return False, 'floor-hazard vision is not ready'
    if status.get('action_output') != 'disabled':
        return False, 'vision action output must be disabled for patrol'
    if status.get('mode') != 'floor-hazards':
        return False, 'vision must be in floor-hazards mode for patrol'
    object_status = status.get('object_detection')
    if not isinstance(object_status, dict) or not object_status.get(
        'ready', False
    ):
        return False, 'object detector is unavailable for patrol'
    missing = object_status.get('missing_dangerous_classes')
    if not isinstance(missing, list) or any(
        not isinstance(label, str) or not label for label in missing
    ):
        return False, 'dangerous-object coverage metadata is invalid'
    if (
        object_status.get('dangerous_coverage_complete') is not True
        or missing
    ):
        detail = ', '.join(missing[:8])
        if len(missing) > 8:
            detail += f' and {len(missing) - 8} more'
        suffix = f': {detail}' if detail else ''
        return False, f'dangerous-object coverage is incomplete{suffix}'
    models = object_status.get('models')
    if not isinstance(models, list) or not models or any(
        not isinstance(model, str) or not model for model in models
    ):
        return False, 'object detector model metadata is invalid'
    return True, 'ready'


def build_location_payload(value, default_map='test1'):
    """Normalize one reusable named map location."""
    if not isinstance(value, dict):
        raise ValidationError('request body must be a JSON object')
    map_name = _clean_name(
        value.get('map', default_map),
        'map',
        maximum=64,
    )
    if not MAP_NAME_PATTERN.fullmatch(map_name):
        raise ValidationError(
            'map may contain only letters, numbers, dot, underscore, and dash'
        )
    name = _clean_name(value.get('name'), 'name')
    if not name:
        raise ValidationError('location name is required')
    waypoint = validate_waypoint(
        {
            'label': name,
            'x': value.get('x'),
            'y': value.get('y'),
            'yaw': value.get('yaw', 0.0),
        },
    )
    return {
        'map': map_name,
        'name': name,
        'x': waypoint['x'],
        'y': waypoint['y'],
        'yaw': waypoint['yaw'],
    }


def _encode_occupancy_runs(data):
    """Return a compact flat value/count run-length encoding."""
    if not data:
        return []
    encoded = []
    previous = int(data[0])
    count = 1
    for raw_value in data[1:]:
        value = int(raw_value)
        if value == previous:
            count += 1
            continue
        encoded.extend((previous, count))
        previous = value
        count = 1
    encoded.extend((previous, count))
    return encoded


class OccupancyMap:
    """Thread-safe map snapshot and authoritative goal-cell validator."""

    def __init__(
        self,
        map_name,
        *,
        occupied_threshold=50,
        minimum_clearance_m=0.18,
    ):
        if not MAP_NAME_PATTERN.fullmatch(str(map_name)):
            raise ValueError('unsupported map name')
        threshold = int(occupied_threshold)
        clearance = float(minimum_clearance_m)
        if not 1 <= threshold <= 100:
            raise ValueError('occupied_threshold must be between 1 and 100')
        if not 0.0 <= clearance <= 2.0:
            raise ValueError('minimum_clearance_m must be between 0 and 2')
        self.map_name = str(map_name)
        self.occupied_threshold = threshold
        self.minimum_clearance_m = clearance
        self._lock = threading.RLock()
        self._snapshot = None
        self._revision = 0

    def update(
        self,
        *,
        frame,
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        origin_yaw,
        data,
    ):
        width = int(width)
        height = int(height)
        resolution = float(resolution)
        origin = (float(origin_x), float(origin_y), float(origin_yaw))
        if not 1 <= width <= 8192 or not 1 <= height <= 8192:
            raise ValueError('map dimensions must be between 1 and 8192')
        if width * height > 16_777_216:
            raise ValueError('map contains too many cells')
        if not 0.001 <= resolution <= 2.0:
            raise ValueError('map resolution must be between 0.001 and 2 m')
        if not all(math.isfinite(value) for value in origin):
            raise ValueError('map origin must be finite')
        cells = tuple(int(value) for value in data)
        if len(cells) != width * height:
            raise ValueError('map data length does not match its dimensions')
        if any(value < -1 or value > 100 for value in cells):
            raise ValueError('map cells must be between -1 and 100')

        with self._lock:
            self._revision += 1
            self._snapshot = {
                'name': self.map_name,
                'frame': str(frame) or 'map',
                'revision': self._revision,
                'updated_at': utc_now(),
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin': {
                    'x': origin[0],
                    'y': origin[1],
                    'yaw': origin[2],
                },
                'occupied_threshold': self.occupied_threshold,
                'minimum_clearance_m': self.minimum_clearance_m,
                'encoding': 'rle-value-count',
                'data': cells,
                'runs': _encode_occupancy_runs(cells),
            }

    def available(self):
        with self._lock:
            return self._snapshot is not None

    def summary(self):
        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                return None
            return {
                key: deepcopy(value)
                for key, value in snapshot.items()
                if key not in {'data', 'runs'}
            }

    def payload(self):
        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                raise ConflictError('occupancy map is not available yet')
            return {
                key: deepcopy(value)
                for key, value in snapshot.items()
                if key != 'data'
            }

    @staticmethod
    def _world_to_cell(snapshot, x, y):
        dx = float(x) - snapshot['origin']['x']
        dy = float(y) - snapshot['origin']['y']
        yaw = snapshot['origin']['yaw']
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return (
            math.floor(local_x / snapshot['resolution']),
            math.floor(local_y / snapshot['resolution']),
        )

    def validate_waypoints(self, waypoints):
        """Reject goals outside free map cells or without safe clearance."""
        with self._lock:
            snapshot = self._snapshot
            if snapshot is None:
                raise ConflictError('occupancy map is not available yet')
            # Tuples and immutable scalars remain safe after releasing the lock.
            snapshot = dict(snapshot)
            snapshot['origin'] = dict(snapshot['origin'])

        for index, waypoint in enumerate(waypoints):
            column, row = self._world_to_cell(
                snapshot,
                waypoint['x'],
                waypoint['y'],
            )
            label = waypoint.get('label') or f'Waypoint {index + 1}'
            if not (
                0 <= column < snapshot['width']
                and 0 <= row < snapshot['height']
            ):
                raise ValidationError(f'{label} is outside the active map')

            radius = math.ceil(
                snapshot['minimum_clearance_m'] / snapshot['resolution']
            )
            for offset_y in range(-radius, radius + 1):
                for offset_x in range(-radius, radius + 1):
                    if (
                        math.hypot(offset_x, offset_y)
                        * snapshot['resolution']
                        > snapshot['minimum_clearance_m']
                    ):
                        continue
                    test_column = column + offset_x
                    test_row = row + offset_y
                    if not (
                        0 <= test_column < snapshot['width']
                        and 0 <= test_row < snapshot['height']
                    ):
                        raise ValidationError(
                            f'{label} is too close to the map boundary'
                        )
                    value = snapshot['data'][
                        test_row * snapshot['width'] + test_column
                    ]
                    if value < 0:
                        raise ValidationError(
                            f'{label} is in or too close to unknown space'
                        )
                    if value >= snapshot['occupied_threshold']:
                        raise ValidationError(
                            f'{label} is in or too close to an obstacle'
                        )
        return True

    @staticmethod
    def _point_error(snapshot, waypoint):
        column, row = OccupancyMap._world_to_cell(
            snapshot, waypoint['x'], waypoint['y']
        )
        if not (0 <= column < snapshot['width'] and 0 <= row < snapshot['height']):
            return 'outside the active map'
        radius = math.ceil(
            snapshot['minimum_clearance_m'] / snapshot['resolution']
        )
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if (
                    math.hypot(offset_x, offset_y) * snapshot['resolution']
                    > snapshot['minimum_clearance_m']
                ):
                    continue
                test_column = column + offset_x
                test_row = row + offset_y
                if not (
                    0 <= test_column < snapshot['width']
                    and 0 <= test_row < snapshot['height']
                ):
                    return 'too close to the map boundary'
                value = snapshot['data'][
                    test_row * snapshot['width'] + test_column
                ]
                if value < 0:
                    return 'in or too close to unknown space'
                if value >= snapshot['occupied_threshold']:
                    return 'in or too close to an obstacle'
        return None

    def generate_patrol_waypoints(self, polygon, spacing_m, maximum=120):
        """Generate a deterministic serpentine route inside a safe polygon."""
        points = validate_patrol_polygon(polygon)
        spacing = _finite_number(spacing_m, 'spacing_m', 0.3, 3.0)
        maximum = max(2, min(int(maximum), 500))
        with self._lock:
            if self._snapshot is None:
                raise ConflictError('occupancy map is not available yet')
            snapshot = dict(self._snapshot)
            snapshot['origin'] = dict(self._snapshot['origin'])

        edges = [
            (
                points[index],
                points[(index + 1) % len(points)],
                math.hypot(
                    points[(index + 1) % len(points)]['x'] - point['x'],
                    points[(index + 1) % len(points)]['y'] - point['y'],
                ),
            )
            for index, point in enumerate(points)
        ]
        edge_start, edge_end, _ = max(edges, key=lambda item: item[2])
        heading = math.atan2(
            edge_end['y'] - edge_start['y'],
            edge_end['x'] - edge_start['x'],
        )
        cosine = math.cos(heading)
        sine = math.sin(heading)

        def to_local(point):
            return (
                cosine * point['x'] + sine * point['y'],
                -sine * point['x'] + cosine * point['y'],
            )

        local = [to_local(point) for point in points]
        low_v = min(point[1] for point in local)
        high_v = max(point[1] for point in local)
        if high_v - low_v < spacing:
            scan_values = [(low_v + high_v) / 2.0]
        else:
            count = max(1, math.floor((high_v - low_v) / spacing))
            margin = ((high_v - low_v) - count * spacing) / 2.0
            scan_values = [low_v + margin + (index + 0.5) * spacing for index in range(count)]

        route = []
        reverse = False
        for scan_v in scan_values:
            intersections = []
            for index, first in enumerate(local):
                second = local[(index + 1) % len(local)]
                if (first[1] <= scan_v < second[1]) or (
                    second[1] <= scan_v < first[1]
                ):
                    ratio = (scan_v - first[1]) / (second[1] - first[1])
                    intersections.append(first[0] + ratio * (second[0] - first[0]))
            intersections.sort()
            scan_points = []
            for low_u, high_u in zip(intersections[::2], intersections[1::2]):
                width = high_u - low_u
                if width <= 0:
                    continue
                sample_count = max(1, math.floor(width / spacing))
                sample_margin = (width - (sample_count - 1) * spacing) / 2.0
                for sample in range(sample_count):
                    local_u = low_u + sample_margin + sample * spacing
                    waypoint = {
                        'x': cosine * local_u - sine * scan_v,
                        'y': sine * local_u + cosine * scan_v,
                    }
                    if self._point_error(snapshot, waypoint) is None:
                        scan_points.append(waypoint)
            if reverse:
                scan_points.reverse()
            reverse = not reverse
            route.extend(scan_points)
            if len(route) > maximum:
                raise ValidationError(
                    f'patrol route exceeds {maximum} safe waypoints; '
                    'increase spacing'
                )

        if len(route) < 2:
            raise ValidationError(
                'patrol area contains fewer than two safe coverage points'
            )
        result = []
        for index, point in enumerate(route):
            other = route[index + 1] if index + 1 < len(route) else route[index - 1]
            yaw = math.atan2(other['y'] - point['y'], other['x'] - point['x'])
            result.append({
                'label': f'Patrol {index + 1}',
                'x': round(point['x'], 4),
                'y': round(point['y'], 4),
                'yaw': yaw,
                'dwell_seconds': 0.0,
            })
        self.validate_waypoints(result)
        return result


class TaskStore:
    """Thread-safe SQLite persistence for autonomous task state."""

    def __init__(self, database_path):
        self.path = Path(database_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            str(self.path),
            check_same_thread=False,
        )
        self._connection.row_factory = sqlite3.Row
        with self._lock, self._connection:
            self._connection.execute(
                '''
                CREATE TABLE IF NOT EXISTS tasks (
                    id TEXT PRIMARY KEY,
                    kind TEXT NOT NULL,
                    name TEXT NOT NULL,
                    state TEXT NOT NULL,
                    payload TEXT NOT NULL,
                    current_step INTEGER NOT NULL DEFAULT 0,
                    error TEXT,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                '''
            )
            self._connection.execute(
                'CREATE INDEX IF NOT EXISTS task_state_created_idx '
                'ON tasks(state, created_at)'
            )
            self._connection.execute(
                '''
                CREATE TABLE IF NOT EXISTS locations (
                    id TEXT PRIMARY KEY,
                    map_name TEXT NOT NULL,
                    name TEXT COLLATE NOCASE NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    yaw REAL NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    UNIQUE(map_name, name)
                )
                '''
            )
            self._connection.execute(
                '''
                CREATE TABLE IF NOT EXISTS patrol_areas (
                    id TEXT PRIMARY KEY,
                    map_name TEXT NOT NULL,
                    name TEXT COLLATE NOCASE NOT NULL,
                    polygon TEXT NOT NULL,
                    spacing_m REAL NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    UNIQUE(map_name, name)
                )
                '''
            )
            self._connection.execute(
                '''
                CREATE TABLE IF NOT EXISTS hazard_observations (
                    id TEXT PRIMARY KEY,
                    task_id TEXT,
                    map_name TEXT NOT NULL,
                    label TEXT NOT NULL,
                    risk TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    box TEXT NOT NULL,
                    robot_pose TEXT,
                    created_at TEXT NOT NULL
                )
                '''
            )
            self._connection.execute(
                'CREATE INDEX IF NOT EXISTS hazard_map_created_idx '
                'ON hazard_observations(map_name, created_at)'
            )
            self._connection.execute(
                '''
                UPDATE tasks
                SET state = 'failed',
                    error = 'Web gateway restarted while task was active',
                    updated_at = ?
                WHERE state IN ('running', 'cancelling')
                ''',
                (utc_now(),),
            )

    @staticmethod
    def _row_to_task(row):
        if row is None:
            return None
        return {
            'id': row['id'],
            'kind': row['kind'],
            'name': row['name'],
            'state': row['state'],
            'payload': json.loads(row['payload']),
            'current_step': int(row['current_step']),
            'error': row['error'],
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def create(self, payload):
        task_id = str(uuid.uuid4())
        timestamp = utc_now()
        with self._lock, self._connection:
            self._connection.execute(
                '''
                INSERT INTO tasks (
                    id, kind, name, state, payload, current_step,
                    error, created_at, updated_at
                ) VALUES (?, ?, ?, 'queued', ?, 0, NULL, ?, ?)
                ''',
                (
                    task_id,
                    payload['kind'],
                    payload['name'],
                    json.dumps(payload, sort_keys=True, separators=(',', ':')),
                    timestamp,
                    timestamp,
                ),
            )
        return self.get(task_id)

    def get(self, task_id):
        with self._lock:
            row = self._connection.execute(
                'SELECT * FROM tasks WHERE id = ?',
                (task_id,),
            ).fetchone()
        return self._row_to_task(row)

    def list(self, limit=100):
        limit = max(1, min(int(limit), 500))
        with self._lock:
            rows = self._connection.execute(
                'SELECT * FROM tasks '
                'ORDER BY created_at DESC, rowid DESC LIMIT ?',
                (limit,),
            ).fetchall()
        return [self._row_to_task(row) for row in rows]

    def next_queued(self):
        with self._lock:
            row = self._connection.execute(
                "SELECT * FROM tasks WHERE state = 'queued' "
                'ORDER BY created_at ASC, rowid ASC LIMIT 1'
            ).fetchone()
        return self._row_to_task(row)

    def update(self, task_id, *, state=None, current_step=None, error=None):
        fields = []
        values = []
        if state is not None:
            if state not in TASK_STATES:
                raise ValueError(f'unknown task state: {state}')
            fields.append('state = ?')
            values.append(state)
        if current_step is not None:
            fields.append('current_step = ?')
            values.append(max(0, int(current_step)))
        if error is not None:
            fields.append('error = ?')
            values.append(str(error)[:1000] if error else None)
        if not fields:
            return self.get(task_id)
        fields.append('updated_at = ?')
        values.append(utc_now())
        values.append(task_id)
        with self._lock, self._connection:
            cursor = self._connection.execute(
                f"UPDATE tasks SET {', '.join(fields)} WHERE id = ?",
                values,
            )
            if cursor.rowcount != 1:
                raise KeyError(task_id)
        return self.get(task_id)

    @staticmethod
    def _row_to_location(row):
        if row is None:
            return None
        return {
            'id': row['id'],
            'map': row['map_name'],
            'name': row['name'],
            'x': float(row['x']),
            'y': float(row['y']),
            'yaw': float(row['yaw']),
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def save_location(self, payload):
        """Create or update a named location within one map."""
        timestamp = utc_now()
        location_id = str(uuid.uuid4())
        with self._lock, self._connection:
            self._connection.execute(
                '''
                INSERT INTO locations (
                    id, map_name, name, x, y, yaw, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(map_name, name) DO UPDATE SET
                    x = excluded.x,
                    y = excluded.y,
                    yaw = excluded.yaw,
                    updated_at = excluded.updated_at
                ''',
                (
                    location_id,
                    payload['map'],
                    payload['name'],
                    payload['x'],
                    payload['y'],
                    payload['yaw'],
                    timestamp,
                    timestamp,
                ),
            )
            row = self._connection.execute(
                'SELECT * FROM locations WHERE map_name = ? AND name = ?',
                (payload['map'], payload['name']),
            ).fetchone()
        return self._row_to_location(row)

    def list_locations(self, map_name):
        with self._lock:
            rows = self._connection.execute(
                'SELECT * FROM locations WHERE map_name = ? '
                'ORDER BY name COLLATE NOCASE ASC',
                (str(map_name),),
            ).fetchall()
        return [self._row_to_location(row) for row in rows]

    def delete_location(self, location_id, map_name):
        with self._lock, self._connection:
            cursor = self._connection.execute(
                'DELETE FROM locations WHERE id = ? AND map_name = ?',
                (str(location_id), str(map_name)),
            )
            if cursor.rowcount != 1:
                raise KeyError(location_id)

    @staticmethod
    def _row_to_patrol_area(row):
        if row is None:
            return None
        return {
            'id': row['id'],
            'map': row['map_name'],
            'name': row['name'],
            'polygon': json.loads(row['polygon']),
            'spacing_m': float(row['spacing_m']),
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def save_patrol_area(self, payload):
        """Create or update a named patrol area within one map."""
        timestamp = utc_now()
        area_id = str(uuid.uuid4())
        polygon_json = json.dumps(
            payload['polygon'], sort_keys=True, separators=(',', ':')
        )
        with self._lock, self._connection:
            self._connection.execute(
                '''
                INSERT INTO patrol_areas (
                    id, map_name, name, polygon, spacing_m,
                    created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(map_name, name) DO UPDATE SET
                    polygon = excluded.polygon,
                    spacing_m = excluded.spacing_m,
                    updated_at = excluded.updated_at
                ''',
                (
                    area_id,
                    payload['map'],
                    payload['name'],
                    polygon_json,
                    payload['spacing_m'],
                    timestamp,
                    timestamp,
                ),
            )
            row = self._connection.execute(
                'SELECT * FROM patrol_areas WHERE map_name = ? AND name = ?',
                (payload['map'], payload['name']),
            ).fetchone()
        return self._row_to_patrol_area(row)

    def get_patrol_area(self, area_id, map_name):
        with self._lock:
            row = self._connection.execute(
                'SELECT * FROM patrol_areas WHERE id = ? AND map_name = ?',
                (str(area_id), str(map_name)),
            ).fetchone()
        return self._row_to_patrol_area(row)

    def list_patrol_areas(self, map_name):
        with self._lock:
            rows = self._connection.execute(
                'SELECT * FROM patrol_areas WHERE map_name = ? '
                'ORDER BY name COLLATE NOCASE ASC',
                (str(map_name),),
            ).fetchall()
        return [self._row_to_patrol_area(row) for row in rows]

    def delete_patrol_area(self, area_id, map_name):
        with self._lock, self._connection:
            cursor = self._connection.execute(
                'DELETE FROM patrol_areas WHERE id = ? AND map_name = ?',
                (str(area_id), str(map_name)),
            )
            if cursor.rowcount != 1:
                raise KeyError(area_id)

    @staticmethod
    def _row_to_hazard(row):
        if row is None:
            return None
        return {
            'id': row['id'],
            'task_id': row['task_id'],
            'map': row['map_name'],
            'label': row['label'],
            'risk': row['risk'],
            'confidence': float(row['confidence']),
            'box': json.loads(row['box']),
            'robot_pose': (
                json.loads(row['robot_pose']) if row['robot_pose'] else None
            ),
            'created_at': row['created_at'],
        }

    def record_hazard(self, payload):
        """Persist one confirmed image observation at the robot pose."""
        observation_id = str(uuid.uuid4())
        timestamp = utc_now()
        pose = payload.get('robot_pose')
        with self._lock, self._connection:
            self._connection.execute(
                '''
                INSERT INTO hazard_observations (
                    id, task_id, map_name, label, risk, confidence,
                    box, robot_pose, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ''',
                (
                    observation_id,
                    payload.get('task_id'),
                    payload['map'],
                    payload['label'],
                    payload['risk'],
                    payload['confidence'],
                    json.dumps(payload['box'], separators=(',', ':')),
                    json.dumps(pose, separators=(',', ':')) if pose else None,
                    timestamp,
                ),
            )
            row = self._connection.execute(
                'SELECT * FROM hazard_observations WHERE id = ?',
                (observation_id,),
            ).fetchone()
        return self._row_to_hazard(row)

    def list_hazards(self, map_name, limit=100):
        limit = max(1, min(int(limit), 500))
        with self._lock:
            rows = self._connection.execute(
                'SELECT * FROM hazard_observations WHERE map_name = ? '
                'ORDER BY created_at DESC, rowid DESC LIMIT ?',
                (str(map_name), limit),
            ).fetchall()
        return [self._row_to_hazard(row) for row in rows]

    def close(self):
        with self._lock:
            self._connection.close()


class EventBus:
    """Small in-memory event journal used by the authenticated SSE endpoint."""

    def __init__(self, capacity=256):
        self._events = deque(maxlen=max(16, int(capacity)))
        self._condition = threading.Condition()
        self._sequence = 0

    def publish(self, event_type, data):
        with self._condition:
            self._sequence += 1
            event = {
                'id': self._sequence,
                'type': str(event_type),
                'time': utc_now(),
                'data': deepcopy(data),
            }
            self._events.append(event)
            self._condition.notify_all()
            return deepcopy(event)

    def after(self, sequence, timeout=15.0):
        sequence = max(0, int(sequence))
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._condition:
            while self._sequence <= sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return []
                self._condition.wait(remaining)
            return [
                deepcopy(event)
                for event in self._events
                if event['id'] > sequence
            ]


class TelemetryCache:
    """Thread-safe latest-value cache with explicit freshness metadata."""

    def __init__(self):
        self._lock = threading.RLock()
        self._values = {}

    def update(self, key, value):
        with self._lock:
            self._values[str(key)] = {
                'value': deepcopy(value),
                'updated_at': utc_now(),
                'monotonic': time.monotonic(),
            }

    def get(self, key, stale_after=10.0):
        with self._lock:
            item = deepcopy(self._values.get(str(key)))
        if item is None:
            return None
        age = max(0.0, time.monotonic() - item.pop('monotonic'))
        item['age_seconds'] = round(age, 3)
        item['stale'] = age > float(stale_after)
        return item

    def snapshot(self, stale_after=10.0):
        with self._lock:
            keys = list(self._values)
        return {key: self.get(key, stale_after) for key in keys}
