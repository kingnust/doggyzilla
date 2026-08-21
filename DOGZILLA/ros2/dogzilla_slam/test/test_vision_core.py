import unittest

import cv2
import numpy as np

from dogzilla_slam.vision_core import COLOR_PRESETS
from dogzilla_slam.vision_core import DangerConfirmationTracker
from dogzilla_slam.vision_core import QR_ACTIONS
from dogzilla_slam.vision_core import SYSTEM_FACE_CASCADE
from dogzilla_slam.vision_core import validate_request
from dogzilla_slam.vision_core import validate_danger_confirmation
from dogzilla_slam.vision_core import validate_face_detection_payload
from dogzilla_slam.vision_core import VisionConfigurationError
from dogzilla_slam.vision_core import VisionProcessor


class FakeObjectPerception:
    def __init__(self, detections):
        self._detections = detections

    @staticmethod
    def coverage():
        return {
            'requested_classes': ['bottle', 'knife', 'person'],
            'covered_classes': ['bottle', 'knife', 'person'],
            'missing_classes': [],
            'missing_dangerous_classes': [],
            'dangerous_coverage_complete': True,
            'person_detection_ready': True,
            'models': ['fixture'],
        }

    def detect(self, _frame, *, focus_floor=False):
        self.focus_floor = focus_floor
        return list(self._detections)

    @staticmethod
    def annotate(frame, _detections):
        return frame.copy()


class VisionCoreTest(unittest.TestCase):
    def test_face_detector_uses_validated_system_cascade(self):
        processor = VisionProcessor(mode='face')

        self.assertFalse(processor._face.empty())
        self.assertEqual(
            processor.face_status()['cascade'],
            SYSTEM_FACE_CASCADE,
        )

    def test_request_accepts_detection_and_disarmed_proposal_modes(self):
        self.assertEqual(
            validate_request({'mode': 'color_track', 'color': 'Blue'}),
            {'mode': 'color-track', 'color': 'blue'},
        )
        self.assertEqual(
            validate_request({'mode': 'qr-action', 'color': 'red'}),
            {'mode': 'qr-action', 'color': 'red'},
        )
        self.assertEqual(
            validate_request({'mode': 'face-handshake', 'color': 'red'}),
            {'mode': 'watchdog', 'color': 'red'},
        )
        with self.assertRaises(VisionConfigurationError):
            validate_request({'mode': 'color', 'color': 'purple'})

    def test_color_detector_finds_yahboom_red_target(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (430, 210), 42, (0, 0, 255), -1)
        processor = VisionProcessor(mode='color-track', color='red')

        annotated, result = processor.process(frame)

        self.assertEqual(annotated.shape, frame.shape)
        self.assertTrue(result['detected'])
        self.assertEqual(result['detections'][0]['kind'], 'color')
        self.assertGreater(result['detections'][0]['error_x'], 0.2)
        self.assertEqual(result['action_output'], 'disabled')
        self.assertEqual(result['action_proposals'], [])

    def test_color_action_matches_yahboom_trigger_but_stays_disarmed(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 240), 72, (0, 0, 255), -1)

        _, result = VisionProcessor(
            mode='color-action',
            color='red',
        ).process(frame)

        self.assertEqual(result['action_output'], 'disabled')
        self.assertEqual(len(result['action_proposals']), 1)
        proposal = result['action_proposals'][0]
        self.assertEqual(proposal['action_id'], 14)
        self.assertEqual(proposal['name'], 'stretch')
        self.assertTrue(proposal['requires_explicit_arming'])
        self.assertFalse(proposal['executed'])

    def test_watchdog_matches_installed_notebook_face_trigger(self):
        class FaceDetector:
            @staticmethod
            def detectMultiScale(*_args, **_kwargs):
                return [(260, 180, 120, 100)]

        processor = VisionProcessor(mode='watchdog')
        processor._face = FaceDetector()

        _, result = processor.process(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        self.assertEqual(result['action_output'], 'disabled')
        self.assertEqual(result['action_proposals'][0]['action_id'], 19)
        self.assertEqual(result['action_proposals'][0]['name'], 'handshake')
        self.assertFalse(result['action_proposals'][0]['executed'])

    def test_line_detector_uses_yahboom_saved_hsv_range(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        midpoint = tuple(
            (lower + upper) // 2
            for lower, upper in zip(*(
                ((55, 214, 183), (125, 253, 255))
            ))
        )
        hsv = np.zeros_like(frame)
        cv2.rectangle(hsv, (285, 280), (355, 479), midpoint, -1)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        processor = VisionProcessor(mode='line')

        _, result = processor.process(frame)

        self.assertTrue(result['detected'])
        self.assertEqual(result['detections'][0]['kind'], 'line')
        self.assertAlmostEqual(
            result['detections'][0]['error_x'],
            0.0,
            places=2,
        )

    def test_line_follow_proposes_but_never_executes_velocity(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        midpoint = tuple(
            (lower + upper) // 2
            for lower, upper in zip(*(
                ((55, 214, 183), (125, 253, 255))
            ))
        )
        hsv = np.zeros_like(frame)
        cv2.rectangle(hsv, (285, 280), (355, 479), midpoint, -1)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        _, result = VisionProcessor(mode='line-follow').process(frame)

        self.assertEqual(result['action_output'], 'disabled')
        self.assertEqual(len(result['action_proposals']), 1)
        self.assertEqual(
            result['action_proposals'][0]['kind'],
            'velocity-intent',
        )
        self.assertFalse(result['action_proposals'][0]['executed'])

    def test_blank_frames_produce_json_safe_empty_results(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        for mode in (
            'raw',
            'color',
            'color-action',
            'face',
            'watchdog',
            'qr',
            'qr-action',
            'line',
            'line-follow',
            'objects',
            'dangerous-objects',
            'floor-hazards',
            'patrol',
        ):
            with self.subTest(mode=mode):
                _, result = VisionProcessor(mode=mode).process(frame)
                self.assertFalse(result['detected'])
                self.assertEqual(result['detections'], [])
                self.assertEqual(result['action_proposals'], [])

    def test_object_modes_use_injected_opencv_perception(self):
        detections = [
            {
                'kind': 'object',
                'label': 'knife',
                'confidence': 0.88,
                'box': [200, 300, 80, 100],
                'floor_candidate': True,
                'floor_hazard': True,
                'dangerous': True,
                'small_floor_hazard': False,
            },
            {
                'kind': 'object',
                'label': 'person',
                'confidence': 0.91,
                'box': [300, 80, 100, 260],
                'floor_candidate': False,
                'floor_hazard': False,
                'dangerous': False,
                'small_floor_hazard': False,
            },
            {
                'kind': 'object',
                'label': 'bottle',
                'confidence': 0.71,
                'box': [10, 20, 30, 80],
                'floor_candidate': False,
                'floor_hazard': False,
                'dangerous': False,
                'small_floor_hazard': False,
            },
        ]
        perception = FakeObjectPerception(detections)

        _, all_objects = VisionProcessor(
            mode='objects',
            object_perception=perception,
        ).process(np.zeros((480, 640, 3), dtype=np.uint8))
        _, dangerous_only = VisionProcessor(
            mode='dangerous-objects',
            object_perception=perception,
        ).process(np.zeros((480, 640, 3), dtype=np.uint8))
        _, floor_only = VisionProcessor(
            mode='floor-hazards',
            object_perception=perception,
        ).process(np.zeros((480, 640, 3), dtype=np.uint8))
        patrol_processor = VisionProcessor(
            mode='patrol',
            object_perception=perception,
        )

        class FaceDetector:
            @staticmethod
            def detectMultiScale(*_args, **_kwargs):
                return [(320, 100, 48, 48)]

        patrol_processor._face = FaceDetector()
        _, patrol = patrol_processor.process(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        self.assertEqual(len(all_objects['detections']), 3)
        self.assertEqual(len(floor_only['detections']), 1)
        self.assertEqual(
            [item['label'] for item in dangerous_only['detections']],
            ['knife'],
        )
        self.assertEqual(floor_only['floor_hazard_count'], 1)
        self.assertTrue(perception.focus_floor)
        self.assertTrue(all_objects['object_detection']['ready'])
        self.assertEqual(
            [item.get('label') for item in patrol['detections']],
            ['knife', 'person', None],
        )
        self.assertEqual(patrol['dangerous_object_count'], 1)
        self.assertEqual(patrol['person_count'], 1)
        self.assertEqual(patrol['face_count'], 1)
        self.assertEqual(patrol['detections'][-1]['kind'], 'face')
        validate_face_detection_payload(patrol['detections'][-1])

    def test_danger_confirmation_requires_time_confidence_and_spatial_match(self):
        tracker = DangerConfirmationTracker(
            minimum_confidence=0.65,
            minimum_observations=3,
            minimum_duration_seconds=0.8,
            minimum_iou=0.35,
            maximum_gap_seconds=0.9,
            cooldown_seconds=8.0,
        )

        def detection(x=100, confidence=0.9):
            return {
                'kind': 'object',
                'label': 'knife',
                'confidence': confidence,
                'box': [x, 100, 80, 60],
                'dangerous': True,
                'floor_candidate': True,
                'floor_hazard': True,
            }

        self.assertEqual(tracker.observe([detection()], now=0.0), [])
        self.assertEqual(tracker.observe([detection(102)], now=0.4), [])
        confirmations = tracker.observe([detection(104)], now=0.8)
        self.assertEqual(len(confirmations), 1)
        self.assertEqual(confirmations[0]['detection']['label'], 'knife')
        self.assertEqual(confirmations[0]['confirmation']['observations'], 3)
        self.assertGreaterEqual(
            confirmations[0]['confirmation']['duration_seconds'], 0.8
        )
        payload = {
            'kind': 'danger-confirmation',
            'mode': 'dangerous-objects',
            **confirmations[0],
        }
        self.assertEqual(
            validate_danger_confirmation(payload)['detection']['label'],
            'knife',
        )
        patrol_payload = {**payload, 'mode': 'patrol'}
        self.assertEqual(
            validate_danger_confirmation(patrol_payload)['mode'],
            'patrol',
        )

        person_tracker = DangerConfirmationTracker(
            required_label='person',
            require_dangerous=False,
        )
        person = {
            **detection(),
            'label': 'person',
            'dangerous': False,
            'floor_hazard': False,
        }
        self.assertEqual(person_tracker.observe([person], now=50.0), [])
        self.assertEqual(person_tracker.observe([person], now=50.4), [])
        self.assertEqual(
            len(person_tracker.observe([person], now=50.81)),
            1,
        )
        one_frame = {
            **payload,
            'confirmation': {
                **payload['confirmation'],
                'observations': 1,
            },
        }
        with self.assertRaisesRegex(ValueError, 'observation count'):
            validate_danger_confirmation(one_frame)
        loose_gap = {
            **payload,
            'confirmation': {
                **payload['confirmation'],
                'criteria': {
                    **payload['confirmation']['criteria'],
                    'maximum_gap_seconds': 5.0,
                },
            },
        }
        with self.assertRaisesRegex(ValueError, 'maximum gap'):
            validate_danger_confirmation(loose_gap)

        rapid = DangerConfirmationTracker()
        self.assertEqual(rapid.observe([detection()], now=1.0), [])
        self.assertEqual(rapid.observe([detection()], now=1.1), [])
        self.assertEqual(rapid.observe([detection()], now=1.2), [])

        pi_rate = DangerConfirmationTracker()
        self.assertEqual(pi_rate.observe([detection()], now=30.0), [])
        self.assertEqual(pi_rate.observe([detection()], now=31.2), [])
        self.assertEqual(
            len(pi_rate.observe([detection()], now=32.4)),
            1,
        )
        missed = DangerConfirmationTracker()
        self.assertEqual(missed.observe([detection()], now=40.0), [])
        self.assertEqual(missed.observe([detection()], now=41.6), [])
        self.assertEqual(missed.observe([detection()], now=42.8), [])

        tracker.reset()
        self.assertEqual(tracker.observe([detection()], now=20.0), [])
        self.assertEqual(tracker.observe([detection(400)], now=20.4), [])
        self.assertEqual(tracker.observe([detection(400)], now=20.8), [])
        self.assertEqual(
            tracker.observe([detection(400, confidence=0.64)], now=21.2),
            [],
        )

    def test_qr_detector_decodes_generated_payload(self):
        qr = cv2.QRCodeEncoder_create().encode('DOGZILLA TEST')
        qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[90:390, 170:470] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)

        _, result = VisionProcessor(mode='qr').process(frame)

        self.assertTrue(result['detected'])
        self.assertEqual(result['detections'][0]['text'], 'DOGZILLA TEST')

    def test_qr_action_requires_an_exact_allowlisted_phrase(self):
        def qr_frame(text):
            qr = cv2.QRCodeEncoder_create().encode(text)
            qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
            frame = np.full((480, 640, 3), 255, dtype=np.uint8)
            frame[90:390, 170:470] = cv2.cvtColor(
                qr,
                cv2.COLOR_GRAY2BGR,
            )
            return frame

        _, allowed = VisionProcessor(mode='qr-action').process(
            qr_frame('STAND UP')
        )
        _, arbitrary = VisionProcessor(mode='qr-action').process(
            qr_frame('rm -rf /')
        )

        self.assertEqual(allowed['action_proposals'][0]['action_id'], 2)
        self.assertFalse(allowed['action_proposals'][0]['executed'])
        self.assertEqual(arbitrary['action_proposals'], [])
        self.assertIn('HANDSHAKE', QR_ACTIONS)

    def test_yahboom_color_presets_remain_explicit(self):
        self.assertEqual(
            set(COLOR_PRESETS),
            {'red', 'green', 'blue', 'yellow'},
        )


if __name__ == '__main__':
    unittest.main()
