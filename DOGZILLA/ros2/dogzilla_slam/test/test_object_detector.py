from pathlib import Path
import tempfile
import unittest

import numpy as np

from dogzilla_slam.object_detector import canonical_label
from dogzilla_slam.object_detector import DetectorMetadata
from dogzilla_slam.object_detector import ENGINEERING_CLASSES
from dogzilla_slam.object_detector import GENERAL_INDOOR_CLASSES
from dogzilla_slam.object_detector import load_labels
from dogzilla_slam.object_detector import ObjectDetectorError
from dogzilla_slam.object_detector import ObjectPerception
from dogzilla_slam.object_detector import OPEN_IMAGES_V7_RELEVANT_CLASSES
from dogzilla_slam.object_detector import SMALL_FLOOR_HAZARD_CLASSES
from dogzilla_slam.object_detector import validate_detection_payload
from dogzilla_slam.object_detector import YoloV8OpenCvDetector
from dogzilla_slam.object_detector import YoloXOpenCvDetector
from dogzilla_slam.object_model_validate import validate_custom_model
from dogzilla_slam.object_model_validate import validate_yoloe_model


class FakeDetector:
    def __init__(self, labels, detections=(), name='fake'):
        self.metadata = DetectorMetadata(name, tuple(labels), '/fake/model.onnx')
        self._detections = list(detections)
        self.call_shapes = []

    def detect(self, frame):
        self.call_shapes.append(frame.shape)
        return list(self._detections)


class FakeNetwork:
    def __init__(self, output):
        self.output = output
        self.input_shape = None

    def setInput(self, value):
        self.input_shape = value.shape

    def forward(self, *_args):
        return self.output


class FakeMultiOutputNetwork(FakeNetwork):
    def getUnconnectedOutLayersNames(self):
        return ('detections', 'masks')


class ObjectDetectorTest(unittest.TestCase):
    def test_label_loading_is_normalized_and_rejects_duplicates(self):
        with tempfile.TemporaryDirectory() as directory:
            labels = Path(directory) / 'labels.txt'
            labels.write_text('Bottle\nhand_gun\n', encoding='utf-8')
            self.assertEqual(load_labels(labels), ('bottle', 'gun'))
            labels.write_text('pistol\ngun\n', encoding='utf-8')
            with self.assertRaises(ObjectDetectorError):
                load_labels(labels)
        self.assertEqual(canonical_label('FireArm'), 'gun')
        self.assertEqual(canonical_label('NAILS'), 'nail')
        self.assertEqual(canonical_label('broken glass'), 'glass shard')
        self.assertEqual(canonical_label('thumb tacks'), 'thumbtack')

    def test_coverage_never_claims_classes_missing_from_model(self):
        perception = ObjectPerception([
            FakeDetector(('bottle', 'knife', 'scissors')),
        ])

        coverage = perception.coverage()

        self.assertEqual(
            coverage['covered_classes'],
            ['scissors', 'bottle', 'knife'],
        )
        self.assertIn('gun', coverage['missing_dangerous_classes'])
        self.assertIn('hammer', coverage['missing_classes'])
        self.assertFalse(coverage['dangerous_coverage_complete'])
        self.assertIn('glass shard', SMALL_FLOOR_HAZARD_CLASSES)
        self.assertEqual(coverage['floor_scan']['columns'], 2)

    def test_floor_risk_policy_is_derived_not_trusted_from_model(self):
        detector = FakeDetector(
            ('gun', 'bottle'),
            (
                {
                    'label': 'pistol',
                    'confidence': 0.91,
                    'box': (200, 300, 120, 150),
                    'class_id': 0,
                    'model': 'custom',
                },
                {
                    'label': 'bottle',
                    'confidence': 0.72,
                    'box': (30, 20, 80, 100),
                    'class_id': 1,
                    'model': 'coco',
                },
            ),
        )

        detections = ObjectPerception([detector]).detect(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        gun = detections[0]
        self.assertEqual(gun['label'], 'gun')
        self.assertEqual(gun['risk'], 'critical')
        self.assertTrue(gun['dangerous'])
        self.assertTrue(gun['floor_candidate'])
        self.assertTrue(gun['floor_hazard'])
        bottle = detections[1]
        self.assertFalse(bottle['dangerous'])
        self.assertFalse(bottle['floor_hazard'])

    def test_payload_validator_rejects_model_supplied_policy_lies(self):
        valid = {
            'kind': 'object',
            'label': 'knife',
            'confidence': 0.8,
            'box': [1, 2, 30, 40],
            'dangerous': True,
            'floor_candidate': True,
            'floor_hazard': True,
        }
        self.assertEqual(validate_detection_payload(valid)['label'], 'knife')
        invalid = {**valid, 'dangerous': False}
        with self.assertRaisesRegex(ValueError, 'contradicts policy'):
            validate_detection_payload(invalid)

        screw = {
            **valid,
            'label': 'screws',
            'small_floor_hazard': True,
        }
        self.assertTrue(
            validate_detection_payload(screw)['small_floor_hazard']
        )
        screw['small_floor_hazard'] = False
        with self.assertRaisesRegex(ValueError, 'small_floor_hazard'):
            validate_detection_payload(screw)

    def test_yolox_decoder_uses_opencv_network_without_torch(self):
        output = np.zeros((1, 3549, 7), dtype=np.float32)
        output[0, 0, :4] = [5.0, 5.0, 2.0, 2.0]
        output[0, 0, 4] = 0.95
        output[0, 0, 6] = 0.90
        network = FakeNetwork(output)
        detector = YoloXOpenCvDetector(
            '/unused.onnx',
            ('bottle', 'knife'),
            network=network,
        )

        detections = detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        self.assertEqual(network.input_shape, (1, 3, 416, 416))
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['label'], 'knife')
        self.assertGreater(detections[0]['confidence'], 0.8)

    def test_yolov8_open_images_decoder_selects_relevant_classes(self):
        output = np.zeros((1, 605, 2), dtype=np.float32)
        output[0, :4, 0] = [320.0, 320.0, 100.0, 100.0]
        output[0, 238 + 4, 0] = 0.92
        network = FakeNetwork(output)
        detector = YoloV8OpenCvDetector(
            '/unused.onnx',
            OPEN_IMAGES_V7_RELEVANT_CLASSES,
            network=network,
        )

        detections = detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        self.assertEqual(network.input_shape, (1, 3, 640, 640))
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['label'], 'gun')
        self.assertEqual(detections[0]['class_id'], 238)
        self.assertGreater(detections[0]['confidence'], 0.9)

    def test_yoloe_segmentation_decoder_ignores_mask_output(self):
        labels = ('screw', 'glass shard')
        output = np.zeros((1, 4 + len(labels) + 32, 2), dtype=np.float32)
        output[0, :4, 0] = [320.0, 320.0, 80.0, 30.0]
        output[0, 4, 0] = 0.88
        masks = np.zeros((1, 32, 160, 160), dtype=np.float32)
        network = FakeMultiOutputNetwork([output, masks])
        detector = YoloV8OpenCvDetector(
            '/unused.onnx',
            dict(enumerate(labels)),
            output_class_count=len(labels),
            output_extra_channels=32,
            network=network,
        )

        detections = detector.detect(
            np.zeros((480, 640, 3), dtype=np.uint8)
        )

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]['label'], 'screw')
        self.assertGreater(detections[0]['confidence'], 0.8)

    def test_floor_focus_runs_overlapping_small_hazard_crops_only(self):
        small = FakeDetector(('nail',))
        general = FakeDetector(('bottle',))
        perception = ObjectPerception([small, general])
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        perception.detect(frame, focus_floor=True)

        self.assertEqual(len(small.call_shapes), 3)
        self.assertEqual(small.call_shapes[0], (480, 640, 3))
        self.assertTrue(all(shape[0] == 231 for shape in small.call_shapes[1:]))
        self.assertEqual(len(general.call_shapes), 1)

    def test_open_images_catalog_covers_engineering_and_indoor_classes(self):
        expected = {
            82: 'camera',
            170: 'drill',
            228: 'grinder',
            234: 'hammer',
            441: 'screwdriver',
            598: 'wrench',
            136: 'sofa',
            153: 'desk',
            164: 'door',
            332: 'microwave',
            587: 'window',
        }
        for class_id, label in expected.items():
            self.assertEqual(
                OPEN_IMAGES_V7_RELEVANT_CLASSES[class_id],
                label,
            )
        self.assertIn('multimeter', ENGINEERING_CLASSES)
        self.assertIn('power outlet', GENERAL_INDOOR_CLASSES)
        self.assertGreaterEqual(len(ENGINEERING_CLASSES), 50)
        self.assertGreaterEqual(len(GENERAL_INDOOR_CLASSES), 70)

    def test_custom_model_validator_checks_requested_labels_and_warmup(self):
        output = np.zeros((1, 6, 1), dtype=np.float32)
        output[0, :4, 0] = [208.0, 208.0, 40.0, 20.0]
        output[0, 4, 0] = 0.91
        network = FakeNetwork(output)
        with tempfile.TemporaryDirectory() as directory:
            labels = Path(directory) / 'custom.labels'
            labels.write_text('bolt\nmultimeter\n', encoding='utf-8')
            report = validate_custom_model(
                '/unused.onnx',
                labels,
                required_labels=('bolt', 'multimeter'),
                network=network,
            )

            self.assertEqual(report['labels'], ['bolt', 'multimeter'])
            self.assertEqual(report['input_size'], 416)
            self.assertEqual(network.input_shape, (1, 3, 416, 416))

            labels.write_text('bottle\n', encoding='utf-8')
            with self.assertRaisesRegex(ObjectDetectorError, 'missing: bolt'):
                validate_custom_model(
                    '/unused.onnx',
                    labels,
                    required_labels=('bolt',),
                    network=network,
                )

    def test_yoloe_validator_requires_prompts_and_warms_segmentation_graph(self):
        labels_value = ('screw', 'glass shard')
        output = np.zeros(
            (1, 4 + len(labels_value) + 32, 1),
            dtype=np.float32,
        )
        network = FakeNetwork(output)
        with tempfile.TemporaryDirectory() as directory:
            labels = Path(directory) / 'yoloe.labels'
            labels.write_text('screw\nglass shard\n', encoding='utf-8')
            report = validate_yoloe_model(
                '/unused.onnx',
                labels,
                required_labels=labels_value,
                network=network,
            )

            self.assertEqual(report['input_size'], 640)
            self.assertEqual(report['mask_channels'], 32)
            self.assertEqual(network.input_shape, (1, 3, 640, 640))

            labels.write_text('screw\n', encoding='utf-8')
            with self.assertRaisesRegex(ObjectDetectorError, 'glass shard'):
                validate_yoloe_model(
                    '/unused.onnx',
                    labels,
                    required_labels=labels_value,
                    network=network,
                )


if __name__ == '__main__':
    unittest.main()
