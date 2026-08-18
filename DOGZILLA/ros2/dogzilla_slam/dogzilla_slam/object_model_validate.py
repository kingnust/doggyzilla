"""Validate a custom DOGZILLA YOLOv8 ONNX model before installation."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from .object_detector import canonical_label
from .object_detector import load_labels
from .object_detector import ObjectDetectorError
from .object_detector import SMALL_FLOOR_HAZARD_CLASSES
from .object_detector import YoloV8OpenCvDetector


CUSTOM_INPUT_SIZE = 416
YOLOE_INPUT_SIZE = 640
YOLOE_MASK_CHANNELS = 32


def sha256_file(path):
    """Return a streaming SHA-256 digest for a model file."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate_custom_model(
    model_path,
    labels_path,
    *,
    required_labels=(),
    network=None,
):
    """Load, warm up, and report one fixed-size custom YOLOv8 model."""
    model = Path(model_path)
    labels = load_labels(labels_path)
    required = tuple(canonical_label(value) for value in required_labels)
    missing = [value for value in required if value not in labels]
    if missing:
        raise ObjectDetectorError(
            'custom model labels are missing: ' + ', '.join(missing)
        )
    detector = YoloV8OpenCvDetector(
        model,
        dict(enumerate(labels)),
        name='dogzilla-custom-yolov8',
        input_size=CUSTOM_INPUT_SIZE,
        output_class_count=len(labels),
        network=network,
    )
    detector.detect(
        np.full(
            (CUSTOM_INPUT_SIZE, CUSTOM_INPUT_SIZE, 3),
            114,
            dtype=np.uint8,
        )
    )
    return {
        'schema_version': 1,
        'format': 'ultralytics-yolov8-detect-onnx',
        'input_size': CUSTOM_INPUT_SIZE,
        'labels': list(labels),
        'required_labels': list(required),
        'sha256': sha256_file(model) if network is None else None,
    }


def validate_yoloe_model(
    model_path,
    labels_path,
    *,
    required_labels=SMALL_FLOOR_HAZARD_CLASSES,
    network=None,
):
    """Validate one static-prompt YOLOE segmentation export for OpenCV."""
    model = Path(model_path)
    labels = load_labels(labels_path)
    required = tuple(canonical_label(value) for value in required_labels)
    missing = [value for value in required if value not in labels]
    if missing:
        raise ObjectDetectorError(
            'YOLOE model labels are missing: ' + ', '.join(missing)
        )
    detector = YoloV8OpenCvDetector(
        model,
        dict(enumerate(labels)),
        name='yoloe-small-floor-hazards',
        input_size=YOLOE_INPUT_SIZE,
        output_class_count=len(labels),
        output_extra_channels=YOLOE_MASK_CHANNELS,
        network=network,
    )
    detector.detect(
        np.full(
            (YOLOE_INPUT_SIZE, YOLOE_INPUT_SIZE, 3),
            114,
            dtype=np.uint8,
        )
    )
    return {
        'schema_version': 1,
        'format': 'ultralytics-yoloe-seg-onnx-static-prompts',
        'input_size': YOLOE_INPUT_SIZE,
        'mask_channels': YOLOE_MASK_CHANNELS,
        'labels': list(labels),
        'required_labels': list(required),
        'sha256': sha256_file(model) if network is None else None,
    }


def parser():
    """Build the command-line parser."""
    value = argparse.ArgumentParser(
        description='Validate a custom DOGZILLA YOLOv8 ONNX detector.',
    )
    value.add_argument('model', help='Path to the ONNX model.')
    value.add_argument('labels', help='Path to one-label-per-line text file.')
    value.add_argument(
        '--model-format',
        choices=('custom', 'yoloe'),
        default='custom',
        help='Expected ONNX graph type; default: custom.',
    )
    value.add_argument(
        '--require-label',
        action='append',
        default=[],
        help='Required canonical label; may be repeated.',
    )
    return value


def main(argv=None):
    """Validate the model and print a machine-readable report."""
    arguments = parser().parse_args(argv)
    required = arguments.require_label
    try:
        if arguments.model_format == 'yoloe':
            report = validate_yoloe_model(
                arguments.model,
                arguments.labels,
                required_labels=(required or SMALL_FLOOR_HAZARD_CLASSES),
            )
        else:
            report = validate_custom_model(
                arguments.model,
                arguments.labels,
                required_labels=required,
            )
    except (OSError, ObjectDetectorError, ValueError) as exc:
        print(f'Object model validation failed: {exc}', file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
