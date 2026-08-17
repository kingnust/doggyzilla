from email.parser import BytesParser
from email.message import Message
from io import BytesIO
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from dogzilla_slam.web_core import EventBus
from dogzilla_slam.web_core import ValidationError
from dogzilla_slam.web_http import GatewayRequestHandler


TOKEN = 'a-secure-test-token-that-is-long-enough'


class FakeGateway:
    def __init__(self):
        self.events = EventBus()
        self.tasks = {}
        self.estop = False
        self.exceptions = []
        self.locations = {}
        self.patrol_areas = {}
        self.hazards = []

    def log_http(self, _message):
        pass

    def log_exception(self, message, exception):
        self.exceptions.append((message, exception))

    def get_state(self):
        return {
            'configuration': {'map': 'test1'},
            'robot': {
                'mode': 'navigation',
                'nodes': ['planner_server', 'dogzilla_safe_base'],
                'nav_available': True,
                'updated_at': '2026-08-10T00:00:00.000Z',
            },
            'telemetry': {
                'battery': {
                    'value': {'percentage': 84, 'present': True},
                    'age_seconds': 0.1,
                    'stale': False,
                },
                'pose': {
                    'value': {
                        'x': 0.2,
                        'y': 0.2,
                        'yaw': 0.0,
                        'linear_speed': 0.0,
                        'angular_speed': 0.0,
                    },
                    'age_seconds': 0.1,
                    'stale': False,
                },
                'map': {
                    'value': {
                        'name': 'test1',
                        'width': 20,
                        'height': 20,
                        'resolution': 0.05,
                        'revision': 1,
                    },
                    'age_seconds': 0.1,
                    'stale': False,
                },
                'joints': {
                    'value': {'count': 12},
                    'age_seconds': 0.1,
                    'stale': False,
                },
            },
            'safety': {
                'estop_latched': self.estop,
                'task_ready': True,
                'task_gate_reason': 'ready',
            },
            'active_task': None,
        }

    def list_tasks(self, limit):
        return list(self.tasks.values())[:limit]

    def get_map(self):
        return {
            'name': 'test1',
            'frame': 'map',
            'revision': 1,
            'updated_at': '2026-08-10T00:00:00.000Z',
            'width': 20,
            'height': 20,
            'resolution': 0.05,
            'origin': {'x': -0.5, 'y': -0.5, 'yaw': 0.0},
            'occupied_threshold': 50,
            'minimum_clearance_m': 0.1,
            'encoding': 'rle-value-count',
            'runs': [0, 400],
        }

    def get_vision_frame(self):
        return b'\xff\xd8fake-jpeg\xff\xd9'

    def set_vision_mode(self, body):
        if body.get('mode') == 'teach':
            raise ValidationError('unsupported action mode')
        return {
            'mode': body.get('mode', 'raw'),
            'color': body.get('color', 'red'),
            'state': 'requested',
            'action_output': 'disabled',
        }

    def list_locations(self):
        return list(self.locations.values())

    def save_location(self, body):
        if not body.get('name'):
            raise ValidationError('location name is required')
        location = {'id': 'location-1', **body}
        self.locations[location['id']] = location
        return location

    def delete_location(self, location_id):
        del self.locations[location_id]

    def list_patrol_areas(self):
        return list(self.patrol_areas.values())

    def save_patrol_area(self, body):
        area = {'id': 'area-1', **body, 'waypoint_count': 8}
        self.patrol_areas[area['id']] = area
        return area

    def delete_patrol_area(self, area_id):
        del self.patrol_areas[area_id]

    def preview_patrol(self, body):
        return {
            'map': body.get('map', 'test1'),
            'waypoint_count': 8,
            'coverage_distance_m': 4.2,
            'waypoints': [{'x': 0, 'y': 0}, {'x': 1, 'y': 0}],
        }

    def create_patrol(self, body):
        task = {'id': 'patrol-1', 'state': 'queued', 'payload': body}
        self.tasks[task['id']] = task
        return task

    def list_hazards(self, limit):
        return self.hazards[:limit]

    def get_task(self, task_id):
        return self.tasks.get(task_id)

    def create_delivery(self, body):
        if 'pickup' not in body:
            raise ValidationError('pickup required')
        task = {'id': 'delivery-1', 'state': 'queued', 'payload': body}
        self.tasks[task['id']] = task
        return task

    def create_route(self, body):
        task = {'id': 'route-1', 'state': 'queued', 'payload': body}
        self.tasks[task['id']] = task
        return task

    def preview_route(self, body):
        return {
            'map': body.get('map', 'test1'),
            'distance_m': 1.25,
            'path': [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}],
        }

    def cancel_task(self, task_id):
        task = self.tasks[task_id]
        task['state'] = 'cancelled'
        return task

    def emergency_stop(self):
        self.estop = True
        return {'estop_latched': True}

    def reset_estop(self):
        self.estop = False
        return {'estop_latched': False}


def request(server, method, path, body=None, authorized=False):
    encoded = b'' if body is None else json.dumps(body).encode()
    headers = Message()
    if authorized:
        headers['Authorization'] = f'Bearer {TOKEN}'
    if body is not None:
        headers['Content-Type'] = 'application/json'
        headers['Content-Length'] = str(len(encoded))

    handler = GatewayRequestHandler.__new__(GatewayRequestHandler)
    handler.server = server
    handler.command = method
    handler.path = path
    handler.request_version = 'HTTP/1.1'
    handler.requestline = f'{method} {path} HTTP/1.1'
    handler.client_address = ('127.0.0.1', 12345)
    handler.close_connection = True
    handler.headers = headers
    handler.rfile = BytesIO(encoded)
    handler.wfile = BytesIO()

    if method == 'GET':
        handler.do_GET()
    elif method == 'POST':
        handler.do_POST()
    else:
        handler.do_DELETE()

    head, raw_payload = handler.wfile.getvalue().split(b'\r\n\r\n', 1)
    status_line, raw_headers = head.split(b'\r\n', 1)
    status = int(status_line.split()[1])
    response_headers = dict(
        BytesParser().parsebytes(raw_headers + b'\r\n\r\n').items()
    )
    if response_headers.get('Content-Type', '').startswith('application/json'):
        payload = json.loads(raw_payload)
    else:
        payload = raw_payload
    return status, response_headers, payload


class WebHTTPTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        static = Path(self.directory.name)
        (static / 'index.html').write_text(
            '<!doctype html><title>DOGZILLA</title>'
        )
        (static / 'app.js').write_text('')
        (static / 'styles.css').write_text('')
        self.gateway = FakeGateway()
        self.server = SimpleNamespace(
            service=self.gateway,
            token=TOKEN,
            static_directory=static.resolve(),
        )

    def tearDown(self):
        self.directory.cleanup()

    def test_public_routes_and_api_authentication(self):
        status, headers, payload = request(self.server, 'GET', '/')
        self.assertEqual(status, 200)
        self.assertIn(b'DOGZILLA', payload)
        self.assertIn(
            "default-src 'self'",
            headers['Content-Security-Policy'],
        )

        status, _, payload = request(self.server, 'GET', '/healthz')
        self.assertEqual(status, 200)
        self.assertEqual(payload, {'status': 'ok'})

        status, headers, payload = request(
            self.server,
            'GET',
            '/api/v1/state',
        )
        self.assertEqual(status, 401)
        self.assertEqual(headers['WWW-Authenticate'], 'Bearer')
        self.assertIn('token', payload['error'])

    def test_authenticated_delivery_lifecycle_and_estop(self):
        status, _, task = request(
            self.server,
            'POST',
            '/api/v1/tasks/delivery',
            {
                'pickup': {'x': 0, 'y': 0},
                'dropoff': {'x': 1, 'y': 1},
            },
            authorized=True,
        )
        self.assertEqual(status, 201)
        self.assertEqual(task['state'], 'queued')

        status, _, payload = request(
            self.server,
            'GET',
            '/api/v1/tasks?limit=10',
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload['tasks'][0]['id'], task['id'])

        status, _, cancelled = request(
            self.server,
            'POST',
            f"/api/v1/tasks/{task['id']}/cancel",
            {},
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(cancelled['state'], 'cancelled')

        status, _, safety = request(
            self.server,
            'POST',
            '/api/v1/estop',
            {},
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertTrue(safety['estop_latched'])

    def test_authenticated_vision_frame_and_safe_mode_request(self):
        status, headers, payload = request(
            self.server,
            'GET',
            '/api/v1/vision/frame.jpg',
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(headers['Content-Type'], 'image/jpeg')
        self.assertTrue(payload.startswith(b'\xff\xd8'))

        status, _, value = request(
            self.server,
            'POST',
            '/api/v1/vision/mode',
            {'mode': 'color-track', 'color': 'blue'},
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(value['mode'], 'color-track')
        self.assertEqual(value['action_output'], 'disabled')

        status, _, value = request(
            self.server,
            'POST',
            '/api/v1/vision/mode',
            {'mode': 'qr-action', 'color': 'red'},
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(value['mode'], 'qr-action')
        self.assertEqual(value['action_output'], 'disabled')

        status, _, value = request(
            self.server,
            'POST',
            '/api/v1/vision/mode',
            {'mode': 'teach', 'color': 'red'},
            authorized=True,
        )
        self.assertEqual(status, 400)
        self.assertIn('action', value['error'])

    def test_map_preview_and_named_location_lifecycle(self):
        status, _, map_payload = request(
            self.server,
            'GET',
            '/api/v1/map',
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(map_payload['revision'], 1)

        status, _, preview = request(
            self.server,
            'POST',
            '/api/v1/routes/preview',
            {'map': 'test1', 'waypoints': [{'x': 1, 'y': 1}]},
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(preview['distance_m'], 1.25)

        status, _, location = request(
            self.server,
            'POST',
            '/api/v1/locations',
            {'map': 'test1', 'name': 'Lab', 'x': 1, 'y': 1, 'yaw': 0},
            authorized=True,
        )
        self.assertEqual(status, 200)
        status, _, payload = request(
            self.server,
            'GET',
            '/api/v1/locations',
            authorized=True,
        )
        self.assertEqual(payload['locations'][0]['name'], 'Lab')
        status, _, payload = request(
            self.server,
            'DELETE',
            f"/api/v1/locations/{location['id']}",
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload['deleted'], location['id'])

    def test_validation_errors_do_not_leak_internal_details(self):
        cases = [{}, {'dropoff': {'x': 1, 'y': 1}}]
        for body in cases:
            with self.subTest(body=body):
                status, _, payload = request(
                    self.server,
                    'POST',
                    '/api/v1/tasks/delivery',
                    body,
                    authorized=True,
                )
                self.assertEqual(status, 400)
                self.assertEqual(payload, {'error': 'pickup required'})
                self.assertEqual(self.gateway.exceptions, [])

    def test_patrol_area_preview_queue_and_delete(self):
        area_body = {
            'map': 'test1',
            'name': 'Workshop',
            'spacing_m': 0.6,
            'polygon': [
                {'x': 0, 'y': 0},
                {'x': 2, 'y': 0},
                {'x': 2, 'y': 1},
                {'x': 0, 'y': 1},
            ],
        }
        status, _, preview = request(
            self.server,
            'POST',
            '/api/v1/patrol-areas/preview',
            area_body,
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(preview['waypoint_count'], 8)

        status, _, area = request(
            self.server,
            'POST',
            '/api/v1/patrol-areas',
            area_body,
            authorized=True,
        )
        self.assertEqual(status, 200)
        status, _, areas = request(
            self.server,
            'GET',
            '/api/v1/patrol-areas',
            authorized=True,
        )
        self.assertEqual(areas['patrol_areas'][0]['id'], area['id'])

        status, _, task = request(
            self.server,
            'POST',
            '/api/v1/tasks/patrol',
            {'patrol_area_id': area['id'], 'repeats': 2},
            authorized=True,
        )
        self.assertEqual(status, 201)
        self.assertEqual(task['state'], 'queued')

        status, _, deleted = request(
            self.server,
            'DELETE',
            f"/api/v1/patrol-areas/{area['id']}",
            authorized=True,
        )
        self.assertEqual(status, 200)
        self.assertEqual(deleted['kind'], 'patrol area')


if __name__ == '__main__':
    unittest.main()
