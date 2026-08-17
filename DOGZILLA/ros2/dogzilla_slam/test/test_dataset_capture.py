from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from dogzilla_slam.dataset_capture import FrameWriter
from dogzilla_slam.dataset_capture import validate_capture_arguments


class DatasetCaptureTest(unittest.TestCase):
    def test_writer_uses_interval_and_atomic_readable_jpegs(self):
        frame = np.full((32, 48, 3), 80, dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            writer = FrameWriter(directory, 'bolt', 2, 1.0)

            first = writer.accept(
                frame,
                monotonic_time=10.0,
                timestamp_ns=100,
            )
            skipped = writer.accept(
                frame,
                monotonic_time=10.5,
                timestamp_ns=101,
            )
            second = writer.accept(
                frame,
                monotonic_time=11.0,
                timestamp_ns=102,
            )

            self.assertEqual(first.name, '100-0001.jpg')
            self.assertIsNone(skipped)
            self.assertEqual(second.name, '102-0002.jpg')
            self.assertTrue(writer.complete)
            self.assertEqual(
                len(list(Path(directory).rglob('*.jpg'))),
                2,
            )
            self.assertIsNotNone(cv2.imread(str(first)))
            self.assertFalse(list(Path(directory).rglob('*.partial')))

    def test_capture_arguments_are_bounded(self):
        self.assertEqual(
            validate_capture_arguments('Bolt_1', 4, 0.5),
            ('bolt_1', 4, 0.5),
        )
        for label in ('../bolt', 'bolt name', ''):
            with self.assertRaises(ValueError):
                validate_capture_arguments(label, 4, 0.5)
        with self.assertRaises(ValueError):
            validate_capture_arguments('bolt', 501, 0.5)
        with self.assertRaises(ValueError):
            validate_capture_arguments('bolt', 4, 0.01)
