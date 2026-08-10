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
