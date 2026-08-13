import unittest

import cv2
import numpy as np

from dogzilla_slam.vision_core import COLOR_PRESETS
from dogzilla_slam.vision_core import validate_request
from dogzilla_slam.vision_core import VisionConfigurationError
from dogzilla_slam.vision_core import VisionProcessor


class VisionCoreTest(unittest.TestCase):
    def test_request_accepts_only_supported_read_only_modes(self):
        self.assertEqual(
            validate_request({'mode': 'color_track', 'color': 'Blue'}),
            {'mode': 'color-track', 'color': 'blue'},
        )
        with self.assertRaises(VisionConfigurationError):
            validate_request({'mode': 'qr-action', 'color': 'red'})
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

    def test_blank_frames_produce_json_safe_empty_results(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        for mode in ('raw', 'color', 'face', 'qr', 'line'):
            with self.subTest(mode=mode):
                _, result = VisionProcessor(mode=mode).process(frame)
                self.assertFalse(result['detected'])
                self.assertEqual(result['detections'], [])

    def test_qr_detector_decodes_generated_payload(self):
        qr = cv2.QRCodeEncoder_create().encode('DOGZILLA TEST')
        qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[90:390, 170:470] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)

        _, result = VisionProcessor(mode='qr').process(frame)

        self.assertTrue(result['detected'])
        self.assertEqual(result['detections'][0]['text'], 'DOGZILLA TEST')

    def test_yahboom_color_presets_remain_explicit(self):
        self.assertEqual(
            set(COLOR_PRESETS),
            {'red', 'green', 'blue', 'yellow'},
        )


if __name__ == '__main__':
    unittest.main()
