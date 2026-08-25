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

            def get_subscription_count(self):
                return 1

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

                diagnostic_stops = Recorder()
                diagnostic_estops = Recorder()
                node._priority_stop_publisher = diagnostic_stops
                node._direct_stop_publisher = diagnostic_stops
                node._estop_publisher = diagnostic_estops
                diagnostics = String()
                diagnostics.data = json.dumps({
                    'kind': 'navigation-diagnostics',
                    'state': 'warning',
                    'warning_only': True,
                    'movement_action': 'none',
                    'warnings': [{
                        'code': 'angular_oscillation',
                        'severity': 'warning',
                        'message': 'rapid turn reversals were commanded',
                    }],
                    'metrics': {'angular_flip_count': 5},
                })
                node._on_navigation_diagnostics(diagnostics)
                node._on_navigation_diagnostics(diagnostics)
                diagnostic_events = [
                    event['type'] for event in node.events.after(0, 0)
                ]
                self.assertEqual(
                    diagnostic_events.count('navigation.warning'),
                    1,
                )
                self.assertEqual(
                    node.telemetry.get('navigation_diagnostics')[
                        'value'
                    ]['state'],
                    'warning',
                )
                self.assertEqual(diagnostic_stops.messages, [])
                self.assertEqual(diagnostic_estops.messages, [])

                diagnostics.data = json.dumps({
                    'kind': 'navigation-diagnostics',
                    'state': 'healthy',
                    'warning_only': True,
                    'movement_action': 'none',
                    'warnings': [],
                    'metrics': {},
                })
                node._on_navigation_diagnostics(diagnostics)
                diagnostic_events = [
                    event['type'] for event in node.events.after(0, 0)
                ]
                self.assertEqual(
                    diagnostic_events.count('navigation.warning_cleared'),
                    1,
                )
                self.assertEqual(diagnostic_stops.messages, [])
                self.assertEqual(diagnostic_estops.messages, [])

                tuning_markers = Recorder()
                node._navigation_tuning_marker_publisher = tuning_markers
                tuning_status = String()
                tuning_status.data = json.dumps({
                    'schema_version': 1,
                    'kind': 'navigation-tuning-recorder',
                    'state': 'recording',
                    'detail': 'Recording synchronized Nav2 tuning evidence',
                    'goal_id': 'a' * 32,
                    'operator_markers': 0,
                    'artifact': None,
                    'control_action': 'none',
                })
                node._on_navigation_tuning_status(tuning_status)
                marker = node.mark_navigation_tuning({
                    'note': 'Turned left then right repeatedly',
                })
                self.assertEqual(marker['control_action'], 'none')
                self.assertEqual(len(tuning_markers.messages), 1)
                marker_payload = json.loads(tuning_markers.messages[0].data)
                self.assertEqual(
                    marker_payload['note'],
                    'Turned left then right repeatedly',
                )
                self.assertEqual(diagnostic_stops.messages, [])
                self.assertEqual(diagnostic_estops.messages, [])
                self.assertEqual(
                    node.telemetry.get('navigation_tuning')['value']['state'],
                    'recording',
                )

                tuning_status.data = json.dumps({
                    'schema_version': 1,
                    'kind': 'navigation-tuning-recorder',
                    'state': 'complete',
                    'detail': 'Trial finished: aborted',
                    'goal_id': None,
                    'operator_markers': 0,
                    'artifact': {
                        'data': '/logs/navigation-tuning/trial.jsonl',
                        'summary': (
                            '/logs/navigation-tuning/trial.summary.json'
                        ),
                        'bytes': 4096,
                        'records': 20,
                        'truncated': False,
                        'maximum_bytes': 8388608,
                    },
                    'control_action': 'none',
                })
                node._on_navigation_tuning_status(tuning_status)
                tuning_events = [
                    event['type'] for event in node.events.after(0, 0)
                ]
                self.assertIn('navigation.tuning_started', tuning_events)
                self.assertIn('navigation.tuning_marker', tuning_events)
                self.assertIn('navigation.tuning_complete', tuning_events)

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

                self.assertEqual(
                    node.get_state()['configuration']['localization'][
                        'state'
                    ],
                    'awaiting-initial-pose',
                )
                initial_poses = Recorder()
                node._initial_pose_publisher = initial_poses
                localization = node.set_initial_pose({
                    'map': 'test1',
                    'x': 2.0,
                    'y': 2.0,
                    'yaw': 1.5708,
                })
                self.assertEqual(localization['state'], 'matching')
                self.assertEqual(localization['movement_action'], 'stop-only')
                self.assertEqual(len(initial_poses.messages), 1)
                initial_message = initial_poses.messages[0]
                self.assertEqual(initial_message.header.frame_id, 'map')
                self.assertAlmostEqual(
                    initial_message.pose.pose.position.x,
                    2.0,
                )
                self.assertGreater(initial_message.pose.covariance[0], 0.0)
                started_ns = node._localization_started_ns
                for sample in range(1, 21):
                    node._update_localization_progress(
                        started_ns + sample,
                        (2.25, 2.05, 1.70),
                    )
                localization_state = node.get_state()[
                    'configuration'
                ]['localization']
                self.assertEqual(localization_state['state'], 'matching')
                self.assertEqual(localization_state['stable_samples'], 20)
                self.assertTrue(
                    localization_state['pose_correction']['within_limit']
                )
                self.assertAlmostEqual(
                    localization_state['pose_correction']['distance_m'],
                    0.255,
                    places=3,
                )
                self.assertAlmostEqual(
                    localization_state['pose_correction']['resolved_pose'][
                        'x'
                    ],
                    2.25,
                )

                rejected_scan = {
                    'finite_rays': 100,
                    'known_endpoints': 80,
                    'matched_endpoints': 16,
                    'contradicted_rays': 35,
                    'coverage_ratio': 0.80,
                    'endpoint_match_ratio': 0.20,
                    'contradiction_ratio': 0.35,
                    'mean_endpoint_error_m': 0.25,
                    'quality': 0.13,
                }
                node._update_scan_validation(
                    started_ns + 100,
                    metrics=rejected_scan,
                )
                localization_state = node.get_state()[
                    'configuration'
                ]['localization']
                self.assertEqual(localization_state['state'], 'matching')
                self.assertEqual(
                    localization_state['scan_validation']['state'],
                    'rejected',
                )
                self.assertIn(
                    'wall agreement',
                    localization_state['scan_validation']['reason'],
                )

                verified_scan = {
                    'finite_rays': 120,
                    'known_endpoints': 100,
                    'matched_endpoints': 78,
                    'contradicted_rays': 5,
                    'coverage_ratio': 0.8333,
                    'endpoint_match_ratio': 0.78,
                    'contradiction_ratio': 0.0417,
                    'mean_endpoint_error_m': 0.06,
                    'quality': 0.7475,
                }
                for sample in range(1, 11):
                    node._update_scan_validation(
                        started_ns + 100 + sample,
                        metrics=verified_scan,
                    )
                localization_state = node.get_state()[
                    'configuration'
                ]['localization']
                self.assertEqual(localization_state['state'], 'ready')
                self.assertEqual(localization_state['stable_samples'], 20)
                self.assertEqual(
                    localization_state['scan_validation']['state'],
                    'verified',
                )
                self.assertEqual(
                    localization_state['scan_validation']['good_samples'],
                    10,
                )
                autonomy_updates = node._autonomy_parameter_updates(4, 4)
                self.assertEqual(
                    autonomy_updates['controller'][0].value,
                    0.20,
                )
                self.assertEqual(
                    autonomy_updates['controller'][1].value,
                    0.40,
                )
                self.assertEqual(
                    autonomy_updates['smoother'][0].value,
                    [0.20, 0.0, 0.40],
                )

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
                published_masks = len(mask_messages.messages)
                node._on_map(message)
                self.assertEqual(
                    len(mask_messages.messages),
                    published_masks,
                )
                information = info_messages.messages[-1]
                self.assertEqual(information.type, 0)
                self.assertEqual(
                    information.filter_mask_topic,
                    '/keepout_filter_mask',
                )
                self.assertEqual(information.base, 0.0)
                self.assertEqual(information.multiplier, 1.0)

                node.set_initial_pose({
                    'map': 'test1',
                    'x': 2.0,
                    'y': 2.0,
                    'yaw': 0.0,
                })
                correction_started_ns = node._localization_started_ns
                for sample in range(1, 11):
                    node._update_localization_progress(
                        correction_started_ns + sample,
                        (3.0, 2.0, 0.0),
                    )
                correction_state = node.get_state()
                correction = correction_state['configuration'][
                    'localization'
                ]
                self.assertEqual(
                    correction['state'],
                    'reposition-required',
                )
                self.assertAlmostEqual(
                    correction['pose_correction']['distance_m'],
                    1.0,
                )
                self.assertFalse(
                    correction['pose_correction']['within_limit']
                )
                self.assertEqual(
                    correction['scan_validation']['good_samples'],
                    0,
                )
                self.assertIn(
                    'set the initial pose again',
                    correction_state['safety']['task_gate_reason'],
                )

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
                self.assertEqual(
                    node.list_hazards(10)[0]['label'],
                    'glass shard',
                )
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
