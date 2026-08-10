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

    def log_http(self, _message):
        pass

    def log_exception(self, message, exception):
        self.exceptions.append((message, exception))

    def get_state(self):
        return {
            'robot': {'mode': 'navigation'},
            'safety': {'estop_latched': self.estop},
        }

    def list_tasks(self, limit):
        return list(self.tasks.values())[:limit]

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
    else:
        handler.do_POST()

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


if __name__ == '__main__':
    unittest.main()
