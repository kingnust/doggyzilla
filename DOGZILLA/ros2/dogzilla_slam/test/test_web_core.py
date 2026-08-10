import math
from pathlib import Path
import tempfile
import unittest

from dogzilla_slam.web_core import build_delivery_payload
from dogzilla_slam.web_core import build_location_payload
from dogzilla_slam.web_core import build_route_payload
from dogzilla_slam.web_core import ConflictError
from dogzilla_slam.web_core import EventBus
from dogzilla_slam.web_core import OccupancyMap
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

    def test_location_payload_requires_a_safe_name_and_finite_pose(self):
        location = build_location_payload(
            {'map': 'room1', 'name': 'Lab door', 'x': 1, 'y': 2, 'yaw': 0.5}
        )
        self.assertEqual(
            location,
            {
                'map': 'room1',
                'name': 'Lab door',
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.5,
            },
        )
        with self.assertRaisesRegex(ValidationError, 'name is required'):
            build_location_payload({'map': 'room1', 'name': '', 'x': 1, 'y': 2})
        with self.assertRaises(ValidationError):
            build_location_payload(
                {'map': 'room1', 'name': 'Bad', 'x': math.nan, 'y': 2}
            )

    def test_occupancy_map_encodes_and_rejects_non_free_goals(self):
        occupancy = OccupancyMap(
            'room1',
            occupied_threshold=50,
            minimum_clearance_m=0.0,
        )
        with self.assertRaises(ConflictError):
            occupancy.payload()

        cells = [0] * 36
        cells[2 * 6 + 2] = -1
        cells[4 * 6 + 4] = 100
        occupancy.update(
            frame='map',
            width=6,
            height=6,
            resolution=0.1,
            origin_x=-0.3,
            origin_y=-0.3,
            origin_yaw=0.0,
            data=cells,
        )
        payload = occupancy.payload()
        decoded = []
        for value, count in zip(payload['runs'][::2], payload['runs'][1::2]):
            decoded.extend([value] * count)
        self.assertEqual(decoded, cells)
        self.assertNotIn('data', payload)
        self.assertEqual(occupancy.summary()['revision'], 1)

        self.assertTrue(
            occupancy.validate_waypoints([{'label': 'Free', 'x': -0.25, 'y': -0.25}])
        )
        cases = [
            ({'label': 'Unknown', 'x': -0.05, 'y': -0.05}, 'unknown'),
            ({'label': 'Wall', 'x': 0.15, 'y': 0.15}, 'obstacle'),
            ({'label': 'Outside', 'x': 2.0, 'y': 2.0}, 'outside'),
        ]
        for waypoint, message in cases:
            with self.subTest(waypoint=waypoint):
                with self.assertRaisesRegex(ValidationError, message):
                    occupancy.validate_waypoints([waypoint])

    def test_occupancy_map_applies_origin_rotation_and_clearance(self):
        occupancy = OccupancyMap('rotated', minimum_clearance_m=0.1)
        cells = [0] * 49
        cells[3 * 7 + 4] = 100
        occupancy.update(
            frame='map',
            width=7,
            height=7,
            resolution=0.1,
            origin_x=1.0,
            origin_y=2.0,
            origin_yaw=math.pi / 2,
            data=cells,
        )
        # Local cell (3, 3) becomes world (0.65, 2.35) after a +90° origin.
        with self.assertRaisesRegex(ValidationError, 'obstacle'):
            occupancy.validate_waypoints([
                {'label': 'Near wall', 'x': 0.65, 'y': 2.35},
            ])

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

    def test_task_store_upserts_and_deletes_named_locations(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            first = store.save_location(
                build_location_payload(
                    {'map': 'room1', 'name': 'Lab', 'x': 1, 'y': 2, 'yaw': 0}
                )
            )
            updated = store.save_location(
                build_location_payload(
                    {'map': 'room1', 'name': 'lab', 'x': 3, 'y': 4, 'yaw': 1}
                )
            )
            store.save_location(
                build_location_payload(
                    {'map': 'room2', 'name': 'Lab', 'x': 0, 'y': 0, 'yaw': 0}
                )
            )

            self.assertEqual(updated['id'], first['id'])
            self.assertEqual(updated['x'], 3.0)
            self.assertEqual(len(store.list_locations('room1')), 1)
            self.assertEqual(len(store.list_locations('room2')), 1)
            store.delete_location(first['id'], 'room1')
            self.assertEqual(store.list_locations('room1'), [])
            with self.assertRaises(KeyError):
                store.delete_location(first['id'], 'room1')
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
