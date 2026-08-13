from types import SimpleNamespace
import unittest

import numpy as np

from dogzilla_slam.vision_node import image_to_bgr


class VisionImageConversionTest(unittest.TestCase):
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
