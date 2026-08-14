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
