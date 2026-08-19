import unittest

from dogzilla_slam.object_acceptance import AcceptanceError
from dogzilla_slam.object_acceptance import ObjectAcceptanceSession
from dogzilla_slam.object_acceptance import validate_arguments


def status(*covered):
    return {
        'state': 'ready',
        'mode': 'objects',
        'action_output': 'disabled',
        'object_detection': {
            'ready': True,
            'covered_classes': list(covered),
            'models': ['test-model'],
        },
    }


def frame(sequence, *, label='hammer', confidence=0.8, floor=True):
    dangerous = label in {'hammer', 'bolt'}
    return {
        'mode': 'objects',
        'sequence': sequence,
        'detections': [{
            'kind': 'object',
            'label': label,
            'confidence': confidence,
            'box': [10, 20, 30, 40],
            'dangerous': dangerous,
            'floor_candidate': floor,
            'floor_hazard': dangerous and floor,
        }],
    }


class ObjectAcceptanceTest(unittest.TestCase):
    def test_arguments_are_normalized_and_bounded(self):
        self.assertEqual(
            validate_arguments('Circuit_Board', 10, 0.6, 4),
            ('circuit board', 10.0, 0.6, 4),
        )
        for duration in (1.9, 121):
            with self.assertRaises(AcceptanceError):
                validate_arguments('hammer', duration, 0.6, 4)
        with self.assertRaises(AcceptanceError):
            validate_arguments('../hammer', 10, 0.6, 4)

    def test_status_requires_coverage_and_disabled_actions(self):
        session = ObjectAcceptanceSession('bolt')
        with self.assertRaisesRegex(AcceptanceError, 'not covered'):
            session.accept_status(status('hammer'))
        armed = status('bolt')
        armed['action_output'] = 'enabled'
        with self.assertRaisesRegex(AcceptanceError, 'disabled'):
            session.accept_status(armed)

    def test_repeated_unique_floor_hits_pass(self):
        session = ObjectAcceptanceSession(
            'hammer',
            require_floor=True,
            confidence=0.55,
            minimum_hits=3,
        )
        session.accept_status(status('hammer', 'bottle'))
        self.assertFalse(session.accept_detections(frame(1, confidence=0.4)))
        self.assertTrue(session.accept_detections(frame(2)))
        self.assertFalse(session.accept_detections(frame(2)))
        self.assertFalse(session.accept_detections(frame(3, floor=False)))
        self.assertTrue(session.accept_detections(frame(4, confidence=0.7)))
        self.assertTrue(session.accept_detections(frame(5, confidence=0.9)))

        report = session.report(duration=3.2)

        self.assertTrue(report['passed'])
        self.assertEqual(report['frames'], 5)
        self.assertEqual(report['matching_frames'], 3)
        self.assertEqual(report['floor_matching_frames'], 3)
        self.assertEqual(report['best_confidence'], 0.9)

    def test_timeout_report_explains_missing_evidence(self):
        session = ObjectAcceptanceSession('bottle', minimum_hits=2)
        session.accept_status(status('bottle'))
        session.accept_detections(
            frame(1, label='bottle', confidence=0.7, floor=False)
        )

        report = session.report(duration=15.0)

        self.assertFalse(report['passed'])
        self.assertIn('needed 2', report['reason'])

    def test_detector_policy_contradictions_fail_closed(self):
        session = ObjectAcceptanceSession('hammer')
        bad = frame(1)
        bad['detections'][0]['dangerous'] = False
        with self.assertRaisesRegex(AcceptanceError, 'contradicts policy'):
            session.accept_detections(bad)

    def test_patrol_acceptance_ignores_valid_anonymous_face_boxes(self):
        session = ObjectAcceptanceSession('hammer', minimum_hits=1)
        session.accept_status(status('hammer', 'person'))
        payload = frame(1)
        payload['mode'] = 'patrol'
        payload['detections'].append({
            'kind': 'face',
            'box': [100, 80, 40, 40],
            'x_px': 120.0,
            'y_px': 100.0,
            'radius_px': 28.28,
            'error_x': -0.625,
            'error_y': -0.5833,
        })

        self.assertTrue(session.accept_detections(payload))


if __name__ == '__main__':
    unittest.main()
