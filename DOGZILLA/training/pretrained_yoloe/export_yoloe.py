"""Bake DOGZILLA floor-hazard prompts into a pretrained YOLOE ONNX model."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


LABEL_PATTERN = re.compile(r'^[a-z][a-z0-9 -]{0,39}$')


def sha256_file(path):
    """Return the SHA-256 digest of one generated artifact."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_prompts(path):
    """Load a deterministic prompt vocabulary safe for runtime labels."""
    values = [
        ' '.join(line.strip().lower().split())
        for line in Path(path).read_text(encoding='utf-8').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]
    if not 1 <= len(values) <= 64:
        raise ValueError('prompts file must contain from 1 to 64 labels')
    if len(values) != len(set(values)):
        raise ValueError('prompts file contains duplicate labels')
    invalid = [value for value in values if not LABEL_PATTERN.fullmatch(value)]
    if invalid:
        raise ValueError(f'invalid prompt label: {invalid[0]}')
    return tuple(values)


def parser():
    value = argparse.ArgumentParser(
        description=(
            'Export a pretrained, static-prompt YOLOE segmentation model. '
            'This does not train or fine-tune the model.'
        ),
    )
    value.add_argument('--prompts', default='prompts.txt')
    value.add_argument('--output', default='artifacts')
    value.add_argument('--checkpoint', default='yoloe-26n-seg.pt')
    value.add_argument('--device', default='cpu')
    return value


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        prompts = load_prompts(arguments.prompts)
    except (OSError, ValueError) as exc:
        parser().error(str(exc))

    from ultralytics import YOLOE

    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    model = YOLOE(arguments.checkpoint)
    model.set_classes(list(prompts))
    exported = Path(model.export(
        format='onnx',
        imgsz=640,
        opset=12,
        dynamic=False,
        simplify=False,
        nms=False,
        batch=1,
        device=arguments.device,
    ))
    if not exported.is_file():
        raise RuntimeError(f'Ultralytics did not create the export: {exported}')

    model_target = output / 'yoloe_small_hazards.onnx'
    labels_target = output / 'yoloe_small_hazards.labels'
    shutil.copy2(exported, model_target)
    labels_target.write_text('\n'.join(prompts) + '\n', encoding='utf-8')
    report = {
        'schema_version': 1,
        'format': 'ultralytics-yoloe-seg-onnx-static-prompts',
        'checkpoint': arguments.checkpoint,
        'input_size': 640,
        'mask_channels': 32,
        'training_performed': False,
        'labels': list(prompts),
        'model_sha256': sha256_file(model_target),
    }
    (output / 'yoloe_small_hazards.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
