import math
from pathlib import Path
import tempfile
import unittest

from dogzilla_slam.web_core import build_delivery_payload
from dogzilla_slam.web_core import build_location_payload
from dogzilla_slam.web_core import build_keepout_zone_payload
from dogzilla_slam.web_core import build_patrol_area_payload
from dogzilla_slam.web_core import build_patrol_payload
from dogzilla_slam.web_core import build_route_payload
from dogzilla_slam.web_core import classify_robot_mode
from dogzilla_slam.web_core import ConflictError
from dogzilla_slam.web_core import EventBus
from dogzilla_slam.web_core import OccupancyMap
from dogzilla_slam.web_core import patrol_vision_readiness
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
    def test_patrol_requires_complete_non_actuating_hazard_coverage(self):
        status = {
            'state': 'ready',
            'mode': 'patrol',
            'action_output': 'disabled',
            'danger_confirmation': {
                'topic': '/vision/danger_confirmed',
                'minimum_confidence': 0.65,
                'minimum_observations': 3,
                'minimum_duration_seconds': 0.8,
                'minimum_iou': 0.35,
                'maximum_gap_seconds': 1.5,
                'cooldown_seconds': 8.0,
            },
            'object_detection': {
                'ready': True,
                'dangerous_coverage_complete': True,
                'missing_dangerous_classes': [],
                'person_detection_ready': True,
                'models': ['generic', 'custom'],
            },
            'face_detection': {
                'ready': True,
                'method': 'opencv-haar-frontal-face',
                'identification': False,
            },
        }
        self.assertEqual(patrol_vision_readiness(status), (True, 'ready'))

        incomplete = {
            **status,
            'object_detection': {
                **status['object_detection'],
                'dangerous_coverage_complete': False,
                'missing_dangerous_classes': ['bolt', 'wire'],
            },
        }
        ready, reason = patrol_vision_readiness(incomplete)
        self.assertFalse(ready)
        self.assertIn('bolt, wire', reason)

        contradictory = {
            **status,
            'object_detection': {
                **status['object_detection'],
                'missing_dangerous_classes': ['bolt'],
            },
        }
        self.assertFalse(patrol_vision_readiness(contradictory)[0])

        armed = {**status, 'action_output': 'enabled'}
        self.assertFalse(patrol_vision_readiness(armed)[0])
        weak = {
            **status,
            'danger_confirmation': {
                **status['danger_confirmation'],
                'minimum_observations': 1,
            },
        }
        self.assertFalse(patrol_vision_readiness(weak)[0])
        loose_timing = {
            **status,
            'danger_confirmation': {
                **status['danger_confirmation'],
                'maximum_gap_seconds': 5.0,
            },
        }
        self.assertFalse(patrol_vision_readiness(loose_timing)[0])

    def test_robot_graph_distinguishes_vision_control(self):
        self.assertEqual(
            classify_robot_mode(
                ['/dogzilla_vision', '/dogzilla_safe_base'],
            ),
            'vision_control',
        )
        self.assertEqual(classify_robot_mode(['/dogzilla_vision']), 'vision')
        self.assertEqual(
            classify_robot_mode(
                ['/dogzilla_vision', '/dogzilla_safe_base', '/bt_navigator'],
                nav_available=True,
            ),
            'navigation',
        )

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

    def test_patrol_polygon_rejects_crossing_and_unsafe_geometry(self):
        area = build_patrol_area_payload({
            'map': 'room1',
            'name': 'Workshop floor',
            'spacing_m': 0.5,
            'polygon': [
                {'x': 0, 'y': 0},
                {'x': 2, 'y': 0},
                {'x': 2, 'y': 1},
                {'x': 0, 'y': 1},
            ],
        })
        self.assertEqual(area['spacing_m'], 0.5)
        self.assertEqual(len(area['polygon']), 4)

        with self.assertRaisesRegex(ValidationError, 'must not cross'):
            build_patrol_area_payload({
                'name': 'Crossed',
                'polygon': [
                    {'x': 0, 'y': 0},
                    {'x': 2, 'y': 2},
                    {'x': 0, 'y': 2},
                    {'x': 2, 'y': 0},
                ],
            })

    def test_keepout_polygon_accepts_small_furniture_and_rejects_crossing(self):
        zone = build_keepout_zone_payload({
            'map': 'room1',
            'name': 'Movable chair',
            'polygon': [
                {'x': 0.0, 'y': 0.0},
                {'x': 0.4, 'y': 0.0},
                {'x': 0.4, 'y': 0.4},
                {'x': 0.0, 'y': 0.4},
            ],
        })
        self.assertEqual(zone['name'], 'Movable chair')
        self.assertEqual(len(zone['polygon']), 4)

        with self.assertRaisesRegex(ValidationError, 'must not cross'):
            build_keepout_zone_payload({
                'name': 'Crossed',
                'polygon': [
                    {'x': 0, 'y': 0},
                    {'x': 1, 'y': 1},
                    {'x': 0, 'y': 1},
                    {'x': 1, 'y': 0},
                ],
            })
        with self.assertRaisesRegex(ValidationError, 'area must be'):
            build_patrol_area_payload({
                'name': 'Tiny',
                'polygon': [
                    {'x': 0, 'y': 0},
                    {'x': 0.1, 'y': 0},
                    {'x': 0, 'y': 0.1},
                ],
            })

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

    def test_occupancy_map_generates_safe_serpentine_patrol(self):
        occupancy = OccupancyMap('room1', minimum_clearance_m=0.0)
        cells = [0] * 100
        cells[5 * 10 + 5] = 100
        occupancy.update(
            frame='map',
            width=10,
            height=10,
            resolution=0.25,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=cells,
        )
        polygon = [
            {'x': 0.25, 'y': 0.25},
            {'x': 2.25, 'y': 0.25},
            {'x': 2.25, 'y': 2.25},
            {'x': 0.25, 'y': 2.25},
        ]
        waypoints = occupancy.generate_patrol_waypoints(polygon, 0.5)

        self.assertGreaterEqual(len(waypoints), 8)
        self.assertLessEqual(len(waypoints), 120)
        self.assertTrue(occupancy.validate_waypoints(waypoints))
        self.assertTrue(all(point['label'].startswith('Patrol ') for point in waypoints))
        self.assertFalse(
            any(
                math.floor(point['x'] / 0.25) == 5
                and math.floor(point['y'] / 0.25) == 5
                for point in waypoints
            )
        )

        area = {'id': 'area-1', 'map': 'room1', 'name': 'Workshop'}
        task = build_patrol_payload(
            {'name': 'Morning patrol', 'repeats': 3, 'dwell_seconds': 1.5},
            area,
            waypoints,
        )
        self.assertEqual(task['kind'], 'patrol')
        self.assertEqual(task['repeats'], 3)
        self.assertEqual(task['waypoints'][0]['dwell_seconds'], 1.5)

    def test_keepout_mask_and_goal_validation_use_the_same_polygon(self):
        occupancy = OccupancyMap(
            'room1',
            minimum_clearance_m=0.0,
            keepout_clearance_m=0.32,
        )
        occupancy.update(
            frame='map',
            width=4,
            height=4,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            data=[0] * 16,
        )
        zones = [{
            'id': 'zone-1',
            'name': 'Table',
            'polygon': [
                {'x': 0.0, 'y': 0.0},
                {'x': 2.0, 'y': 0.0},
                {'x': 2.0, 'y': 2.0},
                {'x': 0.0, 'y': 2.0},
            ],
        }]
        mask = occupancy.keepout_mask(zones)

        self.assertEqual(mask['data'].count(100), 4)
        self.assertEqual(mask['data'][0], 100)
        self.assertEqual(mask['data'][-1], 0)
        with self.assertRaisesRegex(ValidationError, "keepout zone 'Table'"):
            occupancy.validate_waypoints(
                [{'label': 'Goal', 'x': 1.0, 'y': 1.0}],
                zones,
            )
        with self.assertRaisesRegex(ValidationError, 'too close'):
            occupancy.validate_waypoints(
                [{'label': 'Body overlap', 'x': 2.2, 'y': 1.0}],
                zones,
            )
        self.assertTrue(occupancy.validate_waypoints(
            [{'label': 'Goal', 'x': 3.0, 'y': 3.0}],
            zones,
        ))

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

    def test_task_store_upserts_and_deletes_patrol_areas(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            base = build_patrol_area_payload({
                'map': 'room1',
                'name': 'Lab floor',
                'polygon': [
                    {'x': 0, 'y': 0},
                    {'x': 2, 'y': 0},
                    {'x': 2, 'y': 1},
                    {'x': 0, 'y': 1},
                ],
                'spacing_m': 0.5,
            })
            first = store.save_patrol_area(base)
            updated_payload = dict(base)
            updated_payload['name'] = 'lab FLOOR'
            updated_payload['spacing_m'] = 0.8
            updated = store.save_patrol_area(updated_payload)

            self.assertEqual(updated['id'], first['id'])
            self.assertEqual(updated['spacing_m'], 0.8)
            self.assertEqual(store.get_patrol_area(first['id'], 'room1'), updated)
            self.assertEqual(len(store.list_patrol_areas('room1')), 1)
            store.delete_patrol_area(first['id'], 'room1')
            self.assertEqual(store.list_patrol_areas('room1'), [])
            with self.assertRaises(KeyError):
                store.delete_patrol_area(first['id'], 'room1')
            store.close()

    def test_task_store_upserts_and_deletes_keepout_zones(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            payload = build_keepout_zone_payload({
                'map': 'room1',
                'name': 'Rolling cabinet',
                'polygon': [
                    {'x': 0, 'y': 0},
                    {'x': 1, 'y': 0},
                    {'x': 1, 'y': 1},
                    {'x': 0, 'y': 1},
                ],
            })
            first = store.save_keepout_zone(payload)
            payload['name'] = 'rolling CABINET'
            payload['polygon'][1]['x'] = 1.5
            updated = store.save_keepout_zone(payload)

            self.assertEqual(updated['id'], first['id'])
            self.assertEqual(updated['polygon'][1]['x'], 1.5)
            self.assertEqual(len(store.list_keepout_zones('room1')), 1)
            store.delete_keepout_zone(first['id'], 'room1')
            self.assertEqual(store.list_keepout_zones('room1'), [])
            with self.assertRaises(KeyError):
                store.delete_keepout_zone(first['id'], 'room1')
            store.close()

    def test_task_store_records_hazard_at_robot_observation_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            observation = store.record_hazard({
                'task_id': 'patrol-1',
                'map': 'room1',
                'label': 'knife',
                'risk': 'danger',
                'confidence': 0.88,
                'box': [10, 20, 30, 40],
                'robot_pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.2},
            })

            self.assertEqual(observation['label'], 'knife')
            self.assertEqual(observation['box'], [10, 20, 30, 40])
            self.assertEqual(observation['robot_pose']['x'], 1.0)
            self.assertEqual(store.list_hazards('room1'), [observation])
            self.assertEqual(store.list_hazards('room2'), [])
            store.close()

    def test_task_store_keeps_only_the_latest_25_vision_alerts(self):
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / 'tasks.sqlite3')
            removed = []
            for index in range(27):
                _, expired = store.record_vision_alert({
                    'task_id': 'patrol-1',
                    'map': 'room1',
                    'category': 'person' if index % 2 else 'danger',
                    'label': 'person' if index % 2 else 'knife',
                    'confidence': 0.9,
                    'box': [10, 20, 30, 40],
                    'robot_pose': None,
                    'confirmation': {
                        'mode': 'patrol',
                        'observations': 3,
                    },
                    'photo_name': f'alert-{index:032x}.jpg',
                })
                removed.extend(expired)

            alerts = store.list_vision_alerts('room1', 25)
            self.assertEqual(len(alerts), 25)
            self.assertEqual(
                removed,
                ['alert-00000000000000000000000000000000.jpg',
                 'alert-00000000000000000000000000000001.jpg'],
            )
            self.assertIsNotNone(store.get_vision_alert(alerts[0]['id']))
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
