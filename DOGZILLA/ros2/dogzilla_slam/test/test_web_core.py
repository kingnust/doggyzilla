import math
from pathlib import Path
import tempfile
import unittest

from dogzilla_slam.web_core import build_delivery_payload
from dogzilla_slam.web_core import build_route_payload
from dogzilla_slam.web_core import EventBus
from dogzilla_slam.web_core import TaskStore
from dogzilla_slam.web_core import TelemetryCache
from dogzilla_slam.web_core import ValidationError


def delivery_request():
    return {
        'name': 'Lab delivery',
        'map': 'room1',
        'pickup': {'x': 1, 'y': 2, 'yaw': 0.5, 'dwell_seconds': 4},
        'dropoff': {'x': -1, 'y': 3, 'yaw': -0.5},
    }


class WebCoreTest(unittest.TestCase):
    def test_delivery_is_normalized_to_two_labeled_waypoints(self):
        payload = build_delivery_payload(delivery_request())

        self.assertEqual(payload['kind'], 'delivery')
        self.assertEqual(payload['name'], 'Lab delivery')
        self.assertEqual(payload['map'], 'room1')
        self.assertEqual(
            payload['waypoints'][0],
            {
                'label': 'Pickup',
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.5,
                'dwell_seconds': 4.0,
            },
        )
        self.assertEqual(payload['waypoints'][1]['label'], 'Drop-off')

    def test_unsafe_waypoint_values_are_rejected(self):
        unsafe_values = [
            ('x', math.nan),
            ('x', math.inf),
            ('x', 100.01),
            ('yaw', math.pi + 0.001),
            ('dwell_seconds', 301),
            ('x', True),
        ]
        for field, value in unsafe_values:
            with self.subTest(field=field, value=value):
                request = delivery_request()
                request['pickup'][field] = value
                with self.assertRaises(ValidationError):
                    build_delivery_payload(request)

    def test_route_size_and_map_name_are_bounded(self):
        with self.assertRaisesRegex(ValidationError, 'between 1 and 20'):
            build_route_payload({'map': 'room1', 'waypoints': []})
        with self.assertRaisesRegex(ValidationError, 'map may contain'):
            build_route_payload(
                {'map': '../../room1', 'waypoints': [{'x': 0, 'y': 0}]}
            )

    def test_task_store_persists_and_recovers_interrupted_tasks(self):
        with tempfile.TemporaryDirectory() as directory:
            database = Path(directory) / 'tasks.sqlite3'
            store = TaskStore(database)
            task = store.create(build_delivery_payload(delivery_request()))
            store.update(task['id'], state='running', current_step=1)
            store.close()

            reopened = TaskStore(database)
            recovered = reopened.get(task['id'])
            self.assertEqual(recovered['state'], 'failed')
            self.assertEqual(recovered['current_step'], 1)
            self.assertIn('restarted', recovered['error'])
            self.assertIsNone(reopened.next_queued())
            reopened.close()

    def test_task_store_returns_oldest_queued_task(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            first = store.create(build_delivery_payload(delivery_request()))
            second_request = delivery_request()
            second_request['name'] = 'Second'
            store.create(build_delivery_payload(second_request))

            self.assertEqual(store.next_queued()['id'], first['id'])
            self.assertEqual(len(store.list()), 2)
            store.close()

    def test_event_bus_and_telemetry_cache_return_copies(self):
        events = EventBus()
        source = {'value': [1]}
        published = events.publish('robot.test', source)
        source['value'].append(2)
        published['data']['value'].append(3)

        self.assertEqual(
            events.after(0, timeout=0)[0]['data'],
            {'value': [1]},
        )

        telemetry = TelemetryCache()
        telemetry.update('battery', {'percentage': 90})
        reading = telemetry.get('battery')
        reading['value']['percentage'] = 0
        self.assertEqual(
            telemetry.get('battery')['value']['percentage'],
            90,
        )


if __name__ == '__main__':
    unittest.main()
