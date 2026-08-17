"""Capture spaced, unannotated camera frames for a custom detector dataset."""

import argparse
import os
from pathlib import Path
import re
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .vision_node import image_to_bgr


LABEL_PATTERN = re.compile(r'^[a-z][a-z0-9_-]{0,31}$')


def validate_capture_arguments(label, count, interval):
    """Normalize and bound operator-provided capture arguments."""
    normalized = str(label).strip().lower()
    if not LABEL_PATTERN.fullmatch(normalized):
        raise ValueError(
            'label must start with a letter and contain only '
            'a-z, 0-9, underscore, or hyphen'
        )
    count = int(count)
    interval = float(interval)
    if not 1 <= count <= 500:
        raise ValueError('count must be from 1 to 500')
    if not 0.1 <= interval <= 30.0:
        raise ValueError('interval must be from 0.1 to 30 seconds')
    return normalized, count, interval


class FrameWriter:
    """Write bounded-rate JPEG samples with collision-free names."""

    def __init__(self, output_root, label, count, interval):
        label, count, interval = validate_capture_arguments(
            label,
            count,
            interval,
        )
        root = Path(output_root).resolve()
        self.directory = root / label / 'unlabeled'
        self.directory.mkdir(parents=True, exist_ok=True)
        if root not in self.directory.resolve().parents:
            raise ValueError('capture directory escapes the output root')
        self.label = label
        self.target_count = count
        self.interval = interval
        self.written = 0
        self._last_capture = None

    @property
    def complete(self):
        return self.written >= self.target_count

    def accept(self, frame, *, monotonic_time=None, timestamp_ns=None):
        """Write a frame when the capture interval has elapsed."""
        if self.complete:
            return None
        now = time.monotonic() if monotonic_time is None else monotonic_time
        if (
            self._last_capture is not None
            and now - self._last_capture < self.interval
        ):
            return None
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
            raise TypeError('capture frame must be a uint8 numpy array')
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError('capture frame must have non-empty BGR shape')
        success, encoded = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 92],
        )
        if not success:
            raise RuntimeError('OpenCV JPEG encoding failed')
        stamp = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
        number = self.written + 1
        destination = self.directory / f'{stamp}-{number:04d}.jpg'
        temporary = destination.with_suffix('.jpg.partial')
        temporary.write_bytes(encoded.tobytes())
        os.chmod(temporary, 0o664)
        os.replace(temporary, destination)
        self._last_capture = float(now)
        self.written = number
        return destination


class DatasetCaptureNode(Node):
    """Capture raw ROS camera frames without opening a display."""

    def __init__(self, writer, image_topic='/camera/image_raw'):
        super().__init__('dogzilla_dataset_capture')
        self.writer = writer
        self.failure = None
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Capturing {writer.target_count} {writer.label} dataset frames '
            f'every {writer.interval:.1f}s into {writer.directory}'
        )

    def _on_image(self, message):
        if self.writer.complete or self.failure is not None:
            return
        try:
            path = self.writer.accept(
                image_to_bgr(message),
                timestamp_ns=(
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                ),
            )
        except Exception as exc:
            self.failure = exc
            self.get_logger().error(f'Dataset capture failed: {exc}')
            return
        if path is not None:
            self.get_logger().info(
                f'Captured {self.writer.written}/'
                f'{self.writer.target_count}: {path.name}'
            )


def parser():
    """Build the capture command-line parser."""
    value = argparse.ArgumentParser(
        description='Capture raw camera frames for custom object training.',
    )
    value.add_argument(
        '--label',
        required=True,
        help='Dataset category, for example multimeter or chair.',
    )
    value.add_argument('--count', type=int, default=60)
    value.add_argument('--interval', type=float, default=1.0)
    value.add_argument('--output-root', default='/datasets')
    value.add_argument('--image-topic', default='/camera/image_raw')
    return value


def main(argv=None):
    """Capture until the requested sample count is reached."""
    arguments, ros_arguments = parser().parse_known_args(argv)
    try:
        writer = FrameWriter(
            arguments.output_root,
            arguments.label,
            arguments.count,
            arguments.interval,
        )
    except (OSError, TypeError, ValueError) as exc:
        parser().error(str(exc))
    rclpy.init(args=ros_arguments)
    node = DatasetCaptureNode(writer, arguments.image_topic)
    try:
        while rclpy.ok() and not writer.complete and node.failure is None:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.failure is not None:
        return 1
    print(f'Captured {writer.written} images in {writer.directory}')
    return 0 if writer.complete else 130


if __name__ == '__main__':
    raise SystemExit(main())
