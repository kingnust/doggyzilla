"""Authenticated standard-library HTTP API and static dashboard server."""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
from pathlib import Path
import socket
from urllib.parse import parse_qs, urlsplit

from .web_core import ConflictError, ValidationError


MAX_REQUEST_BYTES = 64 * 1024
STATIC_FILES = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/assets/app.js': ('app.js', 'text/javascript; charset=utf-8'),
    '/assets/styles.css': ('styles.css', 'text/css; charset=utf-8'),
}


class GatewayHTTPServer(ThreadingHTTPServer):
    """Threading HTTP server carrying the gateway service dependency."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address,
        service,
        password,
        static_directory,
        legacy_token='',
    ):
        self.service = service
        self.password = str(password)
        self.legacy_token = str(legacy_token)
        self.static_directory = Path(static_directory).resolve()
        super().__init__(address, GatewayRequestHandler)


class GatewayRequestHandler(BaseHTTPRequestHandler):
    """Serve static assets and the versioned DOGZILLA API."""

    server_version = 'DogzillaWeb/0.1'
    protocol_version = 'HTTP/1.1'

    def setup(self):
        super().setup()
        self.connection.settimeout(10.0)

    def log_message(self, format_string, *args):
        self.server.service.log_http(format_string % args)

    def _security_headers(self, api=False):
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'self'; img-src 'self' data: blob:; "
            "script-src 'self'; style-src 'self'; connect-src 'self'",
        )
        if api:
            self.send_header('Cache-Control', 'no-store')

    def _json(self, status, payload):
        body = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()
        self.send_response(int(status))
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self._security_headers(api=True)
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status, message):
        self._json(status, {'error': str(message)})

    def _jpeg(self, body, sequence=None):
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(body)))
        if sequence is not None:
            self.send_header('X-Dogzilla-Frame-Sequence', str(sequence))
        self._security_headers(api=True)
        self.end_headers()
        self.wfile.write(body)

    def _no_content(self, sequence=None):
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header('Content-Length', '0')
        if sequence is not None:
            self.send_header('X-Dogzilla-Frame-Sequence', str(sequence))
        self._security_headers(api=True)
        self.end_headers()

    def _authorized(self):
        supplied_password = self.headers.get('X-Dogzilla-Password', '')
        if supplied_password and hmac.compare_digest(
            supplied_password,
            self.server.password,
        ):
            return True
        header = self.headers.get('Authorization', '')
        scheme, separator, supplied = header.partition(' ')
        if separator != ' ' or scheme.lower() != 'bearer':
            return False
        accepted = [self.server.password]
        if self.server.legacy_token:
            accepted.append(self.server.legacy_token)
        return any(hmac.compare_digest(supplied, value) for value in accepted)

    def _require_authorized(self):
        if self._authorized():
            return True
        self.send_response(HTTPStatus.UNAUTHORIZED)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        body = b'{"error":"valid DOGZILLA password required"}'
        self.send_header('Content-Length', str(len(body)))
        self._security_headers(api=True)
        self.end_headers()
        self.wfile.write(body)
        return False

    def _read_json(self):
        try:
            content_length = int(self.headers.get('Content-Length', '0'))
        except ValueError as exc:
            raise ValidationError('invalid Content-Length') from exc
        if content_length <= 0:
            return {}
        if content_length > MAX_REQUEST_BYTES:
            raise ValidationError('request body exceeds 64 KiB')
        raw = self.rfile.read(content_length)
        try:
            value = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValidationError('request body must be valid UTF-8 JSON') from exc
        if not isinstance(value, dict):
            raise ValidationError('request body must be a JSON object')
        return value

    def _serve_static(self, path):
        static = STATIC_FILES.get(path)
        if static is None:
            self._error(HTTPStatus.NOT_FOUND, 'not found')
            return
        relative_path, content_type = static
        file_path = (self.server.static_directory / relative_path).resolve()
        if self.server.static_directory not in file_path.parents:
            self._error(HTTPStatus.NOT_FOUND, 'not found')
            return
        try:
            body = file_path.read_bytes()
        except FileNotFoundError:
            self._error(HTTPStatus.NOT_FOUND, 'static asset missing')
            return
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self._security_headers()
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlsplit(self.path)
        path = parsed.path
        if path in STATIC_FILES:
            self._serve_static(path)
            return
        if path == '/healthz':
            self._json(HTTPStatus.OK, {'status': 'ok'})
            return
        if not path.startswith('/api/v1/'):
            self._error(HTTPStatus.NOT_FOUND, 'not found')
            return
        if not self._require_authorized():
            return
        try:
            if path == '/api/v1/state':
                self._json(HTTPStatus.OK, self.server.service.get_state())
            elif path == '/api/v1/vision/frame.jpg':
                query = parse_qs(parsed.query)
                if 'after' not in query:
                    self._jpeg(self.server.service.get_vision_frame())
                else:
                    after = int(query['after'][0])
                    frame, sequence = self.server.service.wait_for_vision_frame(
                        after,
                        timeout=1.0,
                    )
                    if frame is None:
                        self._no_content(sequence)
                    else:
                        self._jpeg(frame, sequence)
            elif path == '/api/v1/map':
                self._json(HTTPStatus.OK, self.server.service.get_map())
            elif path == '/api/v1/locations':
                self._json(
                    HTTPStatus.OK,
                    {'locations': self.server.service.list_locations()},
                )
            elif path == '/api/v1/patrol-areas':
                self._json(
                    HTTPStatus.OK,
                    {'patrol_areas': self.server.service.list_patrol_areas()},
                )
            elif path == '/api/v1/keepout-zones':
                self._json(
                    HTTPStatus.OK,
                    {'keepout_zones': self.server.service.list_keepout_zones()},
                )
            elif path == '/api/v1/hazards':
                query = parse_qs(parsed.query)
                limit = int(query.get('limit', ['100'])[0])
                self._json(
                    HTTPStatus.OK,
                    {'hazards': self.server.service.list_hazards(limit)},
                )
            elif path == '/api/v1/alerts':
                query = parse_qs(parsed.query)
                limit = int(query.get('limit', ['25'])[0])
                self._json(
                    HTTPStatus.OK,
                    {'alerts': self.server.service.list_alerts(limit)},
                )
            elif path.startswith('/api/v1/alerts/') and path.endswith(
                '/photo.jpg'
            ):
                alert_id = path.removeprefix('/api/v1/alerts/').removesuffix(
                    '/photo.jpg'
                )
                self._jpeg(self.server.service.get_alert_photo(alert_id))
            elif path == '/api/v1/tasks':
                query = parse_qs(parsed.query)
                limit = int(query.get('limit', ['100'])[0])
                self._json(
                    HTTPStatus.OK,
                    {'tasks': self.server.service.list_tasks(limit)},
                )
            elif path.startswith('/api/v1/tasks/'):
                task_id = path.removeprefix('/api/v1/tasks/')
                task = self.server.service.get_task(task_id)
                if task is None:
                    self._error(HTTPStatus.NOT_FOUND, 'task not found')
                else:
                    self._json(HTTPStatus.OK, task)
            elif path == '/api/v1/events':
                self._stream_events()
            else:
                self._error(HTTPStatus.NOT_FOUND, 'not found')
        except (TypeError, ValueError, ValidationError) as exc:
            self._error(HTTPStatus.BAD_REQUEST, exc)
        except ConflictError as exc:
            self._error(HTTPStatus.CONFLICT, exc)
        except KeyError:
            self._error(HTTPStatus.NOT_FOUND, 'alert photo not found')
        except Exception as exc:  # Keep internal details out of API responses.
            self.server.service.log_exception('GET request failed', exc)
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, 'internal server error')

    def do_POST(self):
        parsed = urlsplit(self.path)
        path = parsed.path
        if not path.startswith('/api/v1/'):
            self._error(HTTPStatus.NOT_FOUND, 'not found')
            return
        if not self._require_authorized():
            return
        try:
            body = self._read_json()
            if path == '/api/v1/tasks/delivery':
                task = self.server.service.create_delivery(body)
                self._json(HTTPStatus.CREATED, task)
            elif path == '/api/v1/tasks/route':
                task = self.server.service.create_route(body)
                self._json(HTTPStatus.CREATED, task)
            elif path == '/api/v1/tasks/patrol':
                task = self.server.service.create_patrol(body)
                self._json(HTTPStatus.CREATED, task)
            elif path == '/api/v1/routes/preview':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.preview_route(body),
                )
            elif path == '/api/v1/patrol-areas/preview':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.preview_patrol(body),
                )
            elif path == '/api/v1/patrol-areas':
                area = self.server.service.save_patrol_area(body)
                self._json(HTTPStatus.OK, area)
            elif path == '/api/v1/keepout-zones':
                zone = self.server.service.save_keepout_zone(body)
                self._json(HTTPStatus.OK, zone)
            elif path == '/api/v1/locations':
                location = self.server.service.save_location(body)
                self._json(HTTPStatus.OK, location)
            elif path.startswith('/api/v1/tasks/') and path.endswith('/cancel'):
                task_id = path.removeprefix('/api/v1/tasks/').removesuffix(
                    '/cancel'
                )
                self._json(
                    HTTPStatus.OK,
                    self.server.service.cancel_task(task_id),
                )
            elif path == '/api/v1/estop':
                self._json(HTTPStatus.OK, self.server.service.emergency_stop())
            elif path == '/api/v1/estop/reset':
                self._json(HTTPStatus.OK, self.server.service.reset_estop())
            elif path == '/api/v1/vision/mode':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.set_vision_mode(body),
                )
            elif path == '/api/v1/map/switch':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.switch_map(body),
                )
            elif path == '/api/v1/map/switch/prepare':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.prepare_map_switch(body),
                )
            elif path == '/api/v1/autonomy/speed':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.set_autonomy_settings(body),
                )
            elif path == '/api/v1/drive':
                self._json(
                    HTTPStatus.OK,
                    self.server.service.set_manual_drive(body),
                )
            else:
                self._error(HTTPStatus.NOT_FOUND, 'not found')
        except ValidationError as exc:
            self._error(HTTPStatus.BAD_REQUEST, exc)
        except KeyError:
            self._error(HTTPStatus.NOT_FOUND, 'requested item not found')
        except ConflictError as exc:
            self._error(HTTPStatus.CONFLICT, exc)
        except Exception as exc:  # Keep internal details out of API responses.
            self.server.service.log_exception('POST request failed', exc)
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, 'internal server error')

    def do_DELETE(self):
        path = urlsplit(self.path).path
        if not path.startswith('/api/v1/'):
            self._error(HTTPStatus.NOT_FOUND, 'not found')
            return
        if not self._require_authorized():
            return
        try:
            location_prefix = '/api/v1/locations/'
            patrol_prefix = '/api/v1/patrol-areas/'
            keepout_prefix = '/api/v1/keepout-zones/'
            if path.startswith(location_prefix) and len(path) > len(location_prefix):
                item_id = path.removeprefix(location_prefix)
                self.server.service.delete_location(item_id)
                kind = 'location'
            elif path.startswith(patrol_prefix) and len(path) > len(patrol_prefix):
                item_id = path.removeprefix(patrol_prefix)
                self.server.service.delete_patrol_area(item_id)
                kind = 'patrol area'
            elif path.startswith(keepout_prefix) and len(path) > len(keepout_prefix):
                item_id = path.removeprefix(keepout_prefix)
                self.server.service.delete_keepout_zone(item_id)
                kind = 'keepout zone'
            else:
                self._error(HTTPStatus.NOT_FOUND, 'not found')
                return
            self._json(HTTPStatus.OK, {'deleted': item_id, 'kind': kind})
        except KeyError:
            self._error(HTTPStatus.NOT_FOUND, 'requested item not found')
        except Exception as exc:  # Keep internal details out of API responses.
            self.server.service.log_exception('DELETE request failed', exc)
            self._error(HTTPStatus.INTERNAL_SERVER_ERROR, 'internal server error')

    def _stream_events(self):
        try:
            sequence = int(self.headers.get('Last-Event-ID', '0') or '0')
        except ValueError:
            sequence = 0
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self.send_header('Connection', 'keep-alive')
        self._security_headers(api=True)
        self.end_headers()
        self.connection.settimeout(20.0)
        try:
            while True:
                events = self.server.service.events.after(sequence, timeout=15.0)
                if not events:
                    self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
                    continue
                for event in events:
                    sequence = event['id']
                    payload = json.dumps(
                        event,
                        separators=(',', ':'),
                        allow_nan=False,
                    ).encode()
                    self.wfile.write(f'id: {sequence}\n'.encode())
                    self.wfile.write(
                        f"event: {event['type']}\n".encode()
                    )
                    self.wfile.write(b'data: ' + payload + b'\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            return
