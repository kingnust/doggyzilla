"""Validate, train, evaluate, and export a custom DOGZILLA detector."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


IMAGE_SUFFIXES = frozenset({'.jpg', '.jpeg', '.png'})
MINIMUM_COUNTS = {
    'train': {'images': 80, 'positive': 40, 'negative': 20},
    'val': {'images': 20, 'positive': 10, 'negative': 5},
}
MINIMUM_INSTANCES = {'train': 20, 'val': 5}
CLASS_PATTERN = re.compile(r'^[a-z][a-z0-9 -]{0,39}$')


def sha256_file(path):
    """Return a streaming SHA-256 digest."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_classes(path):
    """Load a bounded, ordered custom class catalog."""
    values = [
        ' '.join(line.strip().lower().split())
        for line in Path(path).read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]
    if not 1 <= len(values) <= 64:
        raise ValueError('classes file must contain from 1 to 64 labels')
    if len(values) != len(set(values)):
        raise ValueError('classes file contains duplicate labels')
    invalid = [value for value in values if not CLASS_PATTERN.fullmatch(value)]
    if invalid:
        raise ValueError(f'invalid class label: {invalid[0]}')
    return tuple(values)


def validate_label(path, class_count):
    """Validate one YOLO label file and return its class IDs."""
    lines = [line.strip() for line in path.read_text().splitlines()]
    lines = [line for line in lines if line]
    class_ids = []
    for line_number, line in enumerate(lines, start=1):
        values = line.split()
        if len(values) != 5:
            raise ValueError(f'{path}:{line_number}: expected five values')
        try:
            class_id = int(values[0])
            center_x, center_y, width, height = map(float, values[1:])
        except ValueError as exc:
            raise ValueError(
                f'{path}:{line_number}: values must be numeric'
            ) from exc
        if not 0 <= class_id < class_count:
            raise ValueError(
                f'{path}:{line_number}: class ID {class_id} is out of range'
            )
        if not (
            0.0 <= center_x <= 1.0
            and 0.0 <= center_y <= 1.0
            and 0.0 < width <= 1.0
            and 0.0 < height <= 1.0
        ):
            raise ValueError(
                f'{path}:{line_number}: coordinates must be normalized'
            )
        class_ids.append(class_id)
    return tuple(class_ids)


def validate_split(dataset, split, classes):
    """Validate image/label pairs and return split statistics."""
    image_dir = dataset / 'images' / split
    label_dir = dataset / 'labels' / split
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise ValueError(f'{split}: image and label directories are required')
    images = sorted(
        path for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    positive = 0
    negative = 0
    instances = [0] * len(classes)
    hashes = {}
    for image in images:
        label = label_dir / f'{image.stem}.txt'
        if not label.is_file():
            raise ValueError(f'missing label file for {image}')
        class_ids = validate_label(label, len(classes))
        if class_ids:
            positive += 1
            for class_id in class_ids:
                instances[class_id] += 1
        else:
            negative += 1
        digest = sha256_file(image)
        if digest in hashes:
            raise ValueError(f'duplicate images: {hashes[digest]} and {image}')
        hashes[digest] = str(image)
    expected = MINIMUM_COUNTS[split]
    actual = {
        'images': len(images),
        'positive': positive,
        'negative': negative,
        'instances': {
            label: instances[index] for index, label in enumerate(classes)
        },
    }
    for key, minimum in expected.items():
        if actual[key] < minimum:
            raise ValueError(
                f'{split}: needs at least {minimum} {key}; found {actual[key]}'
            )
    for index, label in enumerate(classes):
        minimum = MINIMUM_INSTANCES[split]
        if instances[index] < minimum:
            raise ValueError(
                f'{split}: class {label!r} needs at least {minimum} '
                f'instances; found {instances[index]}'
            )
    return actual, hashes


def validate_dataset(dataset, classes):
    """Reject incomplete datasets and train/validation leakage."""
    root = Path(dataset).resolve()
    train, train_hashes = validate_split(root, 'train', classes)
    validation, validation_hashes = validate_split(root, 'val', classes)
    overlap = set(train_hashes).intersection(validation_hashes)
    if overlap:
        digest = next(iter(overlap))
        raise ValueError(
            'train/validation duplicate: '
            f'{train_hashes[digest]} and {validation_hashes[digest]}'
        )
    return {'train': train, 'val': validation}


def parser():
    """Build the training command-line parser."""
    value = argparse.ArgumentParser()
    value.add_argument('--dataset', default='dataset')
    value.add_argument('--classes', default='classes.txt')
    value.add_argument('--output', default='artifacts')
    value.add_argument('--epochs', type=int, default=120)
    value.add_argument('--batch', type=int, default=16)
    value.add_argument('--device', default='0')
    return value


def main(argv=None):
    """Train on a desktop GPU and export a fixed OpenCV-compatible model."""
    arguments = parser().parse_args(argv)
    if not 20 <= arguments.epochs <= 500:
        parser().error('epochs must be from 20 to 500')
    if not 1 <= arguments.batch <= 128:
        parser().error('batch must be from 1 to 128')
    dataset = Path(arguments.dataset).resolve()
    try:
        classes = load_classes(arguments.classes)
        dataset_counts = validate_dataset(dataset, classes)
    except (OSError, ValueError) as exc:
        parser().error(str(exc))

    from ultralytics import YOLO

    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    dataset_yaml = output / 'dataset.yaml'
    dataset_yaml.write_text(
        json.dumps({
            'path': str(dataset),
            'train': 'images/train',
            'val': 'images/val',
            'names': list(classes),
        }, indent=2) + '\n',
        encoding='utf-8',
    )
    model = YOLO('yolov8n.pt')
    result = model.train(
        data=str(dataset_yaml),
        epochs=arguments.epochs,
        batch=arguments.batch,
        imgsz=416,
        device=arguments.device,
        seed=42,
        deterministic=True,
        patience=30,
        project=str(output / 'runs'),
        name='custom_objects',
        exist_ok=False,
    )
    best = Path(result.save_dir) / 'weights' / 'best.pt'
    trained = YOLO(str(best))
    metrics = trained.val(
        data=str(dataset_yaml),
        imgsz=416,
        split='val',
        device=arguments.device,
    )
    exported = Path(
        trained.export(
            format='onnx',
            imgsz=416,
            opset=12,
            dynamic=False,
            simplify=False,
        )
    )
    model_target = output / 'dogzilla_custom.onnx'
    labels_target = output / 'dogzilla_custom.labels'
    shutil.copy2(exported, model_target)
    labels_target.write_text('\n'.join(classes) + '\n', encoding='utf-8')
    report = {
        'schema_version': 1,
        'format': 'ultralytics-yolov8-detect-onnx',
        'input_size': 416,
        'labels': list(classes),
        'dataset': dataset_counts,
        'metrics': {
            'map50': float(metrics.box.map50),
            'map50_95': float(metrics.box.map),
            'precision': float(metrics.box.mp),
            'recall': float(metrics.box.mr),
        },
        'model_sha256': sha256_file(model_target),
    }
    (output / 'dogzilla_custom.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
