import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from sensor_msgs.msg import Image

from dogzilla_slam.vision_node import DogzillaVisionNode
from dogzilla_slam.vision_node import image_to_bgr


class VisionImageConversionTest(unittest.TestCase):
    def test_node_exposes_confirmed_danger_signal_with_safe_defaults(self):
        rclpy.init()
        node = DogzillaVisionNode()
        try:
            self.assertEqual(
                node._danger_publisher.topic_name,
                '/vision/danger_confirmed',
            )
            status = node._status('ready')
            confirmation = status['danger_confirmation']
            self.assertTrue(status['face_detection']['ready'])
            self.assertFalse(status['face_detection']['identification'])
            self.assertGreaterEqual(confirmation['minimum_confidence'], 0.6)
            self.assertGreaterEqual(confirmation['minimum_observations'], 3)
            self.assertGreaterEqual(
                confirmation['minimum_duration_seconds'], 0.75
            )
            self.assertGreaterEqual(confirmation['minimum_iou'], 0.25)
            self.assertEqual(confirmation['maximum_gap_seconds'], 1.5)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def test_node_publishes_only_after_persistent_matching_danger(self):
        class DangerousProcessor:
            mode = 'dangerous-objects'

            @staticmethod
            def process(frame):
                return frame.copy(), {
                    'mode': 'dangerous-objects',
                    'detections': [{
                        'kind': 'object',
                        'label': 'knife',
                        'confidence': 0.9,
                        'box': [20, 20, 40, 30],
                        'dangerous': True,
                        'floor_candidate': True,
                        'floor_hazard': True,
                    }],
                }

        class Recorder:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        rclpy.init()
        node = DogzillaVisionNode()
        danger = Recorder()
        node._processor = DangerousProcessor()
        node._object_async = False
        node._danger_publisher = danger
        node._detections_publisher = Recorder()
        node._frame_publisher = Recorder()
        node._object_process_hz = 10.0
        message = Image()
        message.header.frame_id = 'camera'
        message.encoding = 'bgr8'
        message.width = 2
        message.height = 2
        message.step = 6
        message.data = bytes(12)
        try:
            with patch(
                'dogzilla_slam.vision_node.time.monotonic',
                side_effect=(10.0, 10.4, 10.8),
            ):
                node._on_image(message)
                self.assertEqual(danger.messages, [])
                node._on_image(message)
                self.assertEqual(danger.messages, [])
                node._on_image(message)

            self.assertEqual(len(danger.messages), 1)
            confirmed = json.loads(danger.messages[0].data)
            self.assertEqual(confirmed['kind'], 'danger-confirmation')
            self.assertEqual(confirmed['mode'], 'dangerous-objects')
            self.assertEqual(confirmed['detection']['label'], 'knife')
            self.assertEqual(
                confirmed['confirmation']['observations'],
                3,
            )
            self.assertGreaterEqual(
                confirmed['confirmation']['duration_seconds'],
                0.8,
            )
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def test_bgr_image_ignores_row_padding(self):
        row_a = bytes([1, 2, 3, 4, 5, 6, 99, 99])
        row_b = bytes([7, 8, 9, 10, 11, 12, 99, 99])
        message = SimpleNamespace(
            encoding='bgr8',
            width=2,
            height=2,
            step=8,
            data=row_a + row_b,
        )

        frame = image_to_bgr(message)

        np.testing.assert_array_equal(
            frame,
            np.array(
                [
                    [[1, 2, 3], [4, 5, 6]],
                    [[7, 8, 9], [10, 11, 12]],
                ],
                dtype=np.uint8,
            ),
        )

    def test_rgb_is_converted_to_bgr(self):
        message = SimpleNamespace(
            encoding='rgb8',
            width=1,
            height=1,
            step=3,
            data=bytes([10, 20, 30]),
        )
        frame = image_to_bgr(message)
        self.assertEqual(frame.tolist(), [[[30, 20, 10]]])

    def test_rejects_short_or_unsupported_images(self):
        with self.assertRaises(ValueError):
            image_to_bgr(SimpleNamespace(
                encoding='yuv422', width=1, height=1, step=2, data=b'00'
            ))
        with self.assertRaises(ValueError):
            image_to_bgr(SimpleNamespace(
                encoding='bgr8', width=2, height=2, step=6, data=b'0'
            ))


if __name__ == '__main__':
    unittest.main()
