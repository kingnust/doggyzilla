import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from dogzilla_slam.web_gateway import DogzillaWebGateway
from dogzilla_slam.web_core import ConflictError, ValidationError


class WebGatewayNodeTest(unittest.TestCase):
    def test_keepout_api_state_rasterizes_and_publishes_without_hardware(self):
        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        with tempfile.TemporaryDirectory() as directory, patch.dict(
            os.environ,
            {
                'DOGZILLA_WEB_PASSWORD': 'yahboom',
                'DOGZILLA_WEB_TOKEN': '0123456789abcdefghijklmn',
                'DOGZILLA_WEB_DATABASE': str(
                    Path(directory) / 'tasks.sqlite3'
                ),
                'DOGZILLA_WEB_MAP_NAME': 'test1',
                'DOGZILLA_WEB_PORT': '18082',
            },
        ):
            rclpy.init()
            node = DogzillaWebGateway()
            try:
                self.assertFalse(node.manual_drive_enabled)
                with self.assertRaises(ConflictError):
                    node.set_manual_drive({'direction': 'forward'})
                stopped = node.set_manual_drive({'direction': 'stop'})
                self.assertEqual(stopped['direction'], 'stop')

                frame = CompressedImage()
                frame.format = 'jpeg'
                frame.data = b'\xff\xd8annotated-frame\xff\xd9'
                node._on_vision_frame(frame)
                message = OccupancyGrid()
                message.header.frame_id = 'map'
                message.info.resolution = 1.0
                message.info.width = 4
                message.info.height = 4
                message.info.origin.orientation.w = 1.0
                message.data = [0] * 16
                node._on_map(message)

                zone = node.save_keepout_zone({
                    'map': 'test1',
                    'name': 'Furniture',
                    'polygon': [
                        {'x': 0.0, 'y': 0.0},
                        {'x': 2.0, 'y': 0.0},
                        {'x': 2.0, 'y': 2.0},
                        {'x': 0.0, 'y': 2.0},
                    ],
                })
                mask = node.occupancy_map.keepout_mask(
                    node.list_keepout_zones()
                )

                self.assertEqual(zone['name'], 'Furniture')
                self.assertEqual(mask['data'].count(100), 4)
                self.assertEqual(
                    node._keepout_info_publisher.topic_name,
                    '/keepout_filter_info',
                )
                self.assertEqual(
                    node._keepout_mask_publisher.topic_name,
                    '/keepout_filter_mask',
                )
                mask_messages = Recorder()
                info_messages = Recorder()
                node._keepout_mask_publisher = mask_messages
                node._keepout_info_publisher = info_messages
                node._publish_keepout_filter()
                self.assertEqual(
                    mask_messages.messages[-1].data.count(100),
                    4,
                )
                information = info_messages.messages[-1]
                self.assertEqual(information.type, 0)
                self.assertEqual(
                    information.filter_mask_topic,
                    '/keepout_filter_mask',
                )
                self.assertEqual(information.base, 0.0)
                self.assertEqual(information.multiplier, 1.0)

                with self.assertRaises(ConflictError):
                    node.switch_map({'map': 'room2'})
                node.prepare_map_switch({'map': 'room2'})
                switched = node.switch_map({'map': 'room2'})
                self.assertEqual(switched['map'], 'room2')
                self.assertEqual(node.list_keepout_zones(), [])
                node._on_map(message)
                with self.assertRaises(ValidationError):
                    node.save_keepout_zone({
                        'map': 'test1',
                        'name': 'Wrong map',
                        'polygon': [
                            {'x': 0.0, 'y': 0.0},
                            {'x': 1.0, 'y': 0.0},
                            {'x': 1.0, 'y': 1.0},
                        ],
                    })
                room_zone = node.save_keepout_zone({
                    'map': 'room2',
                    'name': 'Room two furniture',
                    'polygon': [
                        {'x': 0.0, 'y': 0.0},
                        {'x': 1.0, 'y': 0.0},
                        {'x': 1.0, 'y': 1.0},
                    ],
                })
                self.assertEqual(node.list_keepout_zones(), [room_zone])
                node.prepare_map_switch({'map': 'test1'})
                node.switch_map({'map': 'test1'})
                node._on_map(message)
                self.assertEqual(
                    [item['name'] for item in node.list_keepout_zones()],
                    ['Furniture'],
                )

                raw_detection = String()
                raw_detection.data = json.dumps({
                    'mode': 'dangerous-objects',
                    'detections': [{
                        'kind': 'object',
                        'label': 'knife',
                        'confidence': 0.99,
                        'box': [20, 20, 40, 30],
                        'dangerous': True,
                        'floor_candidate': False,
                        'floor_hazard': False,
                    }],
                })
                node._on_vision_detections(raw_detection)
                self.assertEqual(node.list_hazards(10), [])

                confirmed = String()
                confirmed.data = json.dumps({
                    'schema_version': 1,
                    'kind': 'danger-confirmation',
                    'confirmation_sequence': 1,
                    'mode': 'dangerous-objects',
                    'source_frame': 'camera',
                    'stamp': {'sec': 10, 'nanosec': 0},
                    'detection': {
                        'kind': 'object',
                        'label': 'knife',
                        'confidence': 0.9,
                        'box': [20, 20, 40, 30],
                        'dangerous': True,
                        'floor_candidate': False,
                        'floor_hazard': False,
                    },
                    'confirmation': {
                        'observations': 3,
                        'duration_seconds': 0.8,
                        'lowest_confidence': 0.85,
                        'minimum_observed_iou': 0.7,
                        'criteria': {
                            'minimum_observations': 3,
                            'minimum_duration_seconds': 0.8,
                            'minimum_confidence': 0.65,
                            'minimum_iou': 0.35,
                            'maximum_gap_seconds': 1.5,
                            'cooldown_seconds': 8.0,
                        },
                    },
                })
                node._on_danger_confirmed(confirmed)
                node._on_danger_confirmed(confirmed)
                hazards = node.list_hazards(10)
                self.assertEqual(len(hazards), 1)
                self.assertEqual(hazards[0]['label'], 'knife')
                self.assertEqual(
                    hazards[0]['confirmation']['observations'],
                    3,
                )
                self.assertIn(
                    'hazard.confirmed',
                    [event['type'] for event in node.events.after(0, 0)],
                )
                alerts = node.list_alerts(25)
                self.assertEqual(len(alerts), 1)
                self.assertEqual(alerts[0]['category'], 'danger')
                self.assertTrue(
                    node.get_alert_photo(alerts[0]['id']).startswith(b'\xff\xd8')
                )

                estop_messages = Recorder()
                priority_stop_messages = Recorder()
                direct_stop_messages = Recorder()
                node._estop_publisher = estop_messages
                node._priority_stop_publisher = priority_stop_messages
                node._direct_stop_publisher = direct_stop_messages
                node._active = {
                    'task_id': 'patrol-1',
                    'payload': {'kind': 'patrol'},
                }
                patrol_hazard = json.loads(confirmed.data)
                patrol_hazard['mode'] = 'patrol'
                patrol_hazard['detection'].update({
                    'label': 'broken glass',
                    'box': [250, 200, 60, 40],
                    'floor_candidate': True,
                    'floor_hazard': True,
                })
                patrol_confirmation = String()
                patrol_confirmation.data = json.dumps(patrol_hazard)
                node._on_danger_confirmed(patrol_confirmation)

                self.assertFalse(node._estop_latched)
                self.assertEqual(node._cancel_requests, set())
                self.assertEqual(estop_messages.messages, [])
                self.assertEqual(priority_stop_messages.messages, [])
                self.assertEqual(direct_stop_messages.messages, [])
                self.assertNotIn(
                    'safety.estop',
                    [event['type'] for event in node.events.after(0, 0)],
                )
                self.assertEqual(node.list_hazards(10)[0]['label'], 'glass shard')
                node._active = None

                person = {
                    'mode': 'patrol',
                    'source_frame': 'camera',
                    'stamp': {'sec': 11, 'nanosec': 0},
                    'detections': [{
                        'kind': 'object',
                        'label': 'person',
                        'confidence': 0.92,
                        'box': [100, 40, 80, 220],
                        'dangerous': False,
                        'floor_candidate': False,
                        'floor_hazard': False,
                    }],
                }
                now = time.monotonic()
                node._vision_frame_received = now
                node._person_tracker.observe(
                    person['detections'],
                    now=now - 0.81,
                )
                node._person_tracker.observe(
                    person['detections'],
                    now=now - 0.4,
                )
                detection = String()
                detection.data = json.dumps({**person, 'sequence': 3})
                node._on_vision_detections(detection)

                alerts = node.list_alerts(25)
                self.assertEqual(len(alerts), 3)
                self.assertEqual(alerts[0]['category'], 'person')
                event_types = [
                    event['type'] for event in node.events.after(0, 0)
                ]
                self.assertEqual(event_types.count('person.confirmed'), 1)
                node._recent_alerts.clear()
                node._restore_alert_deduplication()
                duplicate = node._record_vision_alert(
                    category='person',
                    detection=person['detections'][0],
                    confirmation={'observations': 3},
                    mode='patrol',
                )
                self.assertIsNone(duplicate)
                self.assertEqual(len(node.list_alerts(25)), 3)
                second_person = node._record_vision_alert(
                    category='person',
                    detection={
                        **person['detections'][0],
                        'box': [400, 40, 80, 220],
                    },
                    confirmation={'observations': 3},
                    mode='patrol',
                )
                self.assertIsNotNone(second_person)
                self.assertEqual(len(node.list_alerts(25)), 4)

                node._vision_frame_received = time.monotonic()
                for index in range(27):
                    node._record_vision_alert(
                        category='danger',
                        detection={
                            'label': f'test object {index}',
                            'confidence': 0.9,
                            'box': [10, 20, 30, 40],
                        },
                        confirmation={'observations': 3},
                        mode='patrol',
                    )
                self.assertEqual(len(node.list_alerts(25)), 25)
                self.assertEqual(
                    len(list(node.alert_directory.glob('alert-*.jpg'))),
                    25,
                )
            finally:
                node.close()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
