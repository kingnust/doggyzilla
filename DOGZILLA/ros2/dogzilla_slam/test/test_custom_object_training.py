import importlib.util
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np


REPOSITORY = Path(__file__).resolve().parents[3]
TRAINING_SCRIPT = (
    REPOSITORY / 'training' / 'custom_objects' / 'train_export.py'
)
YOLOE_EXPORT_SCRIPT = (
    REPOSITORY / 'training' / 'pretrained_yoloe' / 'export_yoloe.py'
)


def load_training_module():
    spec = importlib.util.spec_from_file_location(
        'dogzilla_custom_object_training',
        TRAINING_SCRIPT,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_yoloe_export_module():
    spec = importlib.util.spec_from_file_location(
        'dogzilla_pretrained_yoloe_export',
        YOLOE_EXPORT_SCRIPT,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CustomObjectTrainingTest(unittest.TestCase):
    def setUp(self):
        self.training = load_training_module()
        self.training.MINIMUM_COUNTS = {
            'train': {'images': 2, 'positive': 1, 'negative': 1},
            'val': {'images': 2, 'positive': 1, 'negative': 1},
        }
        self.training.MINIMUM_INSTANCES = {'train': 1, 'val': 1}

    def _write_sample(self, root, split, name, positive, shade):
        image_dir = root / 'images' / split
        label_dir = root / 'labels' / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        image = np.full((20, 30, 3), shade, dtype=np.uint8)
        self.assertTrue(cv2.imwrite(str(image_dir / f'{name}.jpg'), image))
        content = (
            '0 0.5 0.5 0.2 0.1\n1 0.3 0.4 0.1 0.2\n'
            if positive else ''
        )
        (label_dir / f'{name}.txt').write_text(content)

    def test_dataset_validator_requires_positive_negative_and_no_leakage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_sample(root, 'train', 'positive', True, 20)
            self._write_sample(root, 'train', 'negative', False, 40)
            self._write_sample(root, 'val', 'positive', True, 60)
            self._write_sample(root, 'val', 'negative', False, 80)

            classes = ('bolt', 'multimeter')
            report = self.training.validate_dataset(root, classes)

            self.assertEqual(report['train']['images'], 2)
            self.assertEqual(report['val']['positive'], 1)
            self.assertEqual(
                report['train']['instances']['multimeter'],
                1,
            )

            duplicate = root / 'images' / 'val' / 'negative.jpg'
            duplicate.write_bytes(
                (root / 'images' / 'train' / 'negative.jpg').read_bytes()
            )
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                self.training.validate_dataset(root, classes)

    def test_label_validator_rejects_wrong_class_or_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            label = Path(directory) / 'sample.txt'
            label.write_text('2 0.5 0.5 0.1 0.1\n')
            with self.assertRaisesRegex(ValueError, 'out of range'):
                self.training.validate_label(label, 2)
            label.write_text('0 1.2 0.5 0.1 0.1\n')
            with self.assertRaisesRegex(ValueError, 'normalized'):
                self.training.validate_label(label, 2)

    def test_class_catalog_is_ordered_unique_and_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            classes = Path(directory) / 'classes.txt'
            classes.write_text('Bolt\nmultimeter\n')
            self.assertEqual(
                self.training.load_classes(classes),
                ('bolt', 'multimeter'),
            )
            classes.write_text('bolt\nbolt\n')
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                self.training.load_classes(classes)

    def test_pretrained_yoloe_prompts_are_static_and_need_no_training(self):
        exporter = load_yoloe_export_module()
        prompts = exporter.load_prompts(
            REPOSITORY / 'training' / 'pretrained_yoloe' / 'prompts.txt'
        )

        self.assertIn('screw', prompts)
        self.assertIn('glass shard', prompts)
        self.assertIn('staple', prompts)
        source = YOLOE_EXPORT_SCRIPT.read_text()
        self.assertIn('model.set_classes', source)
        self.assertIn("format='onnx'", source)
        self.assertNotIn('.train(', source)
