# Custom engineering and indoor-object detector training

The COCO and Open Images models already cover many indoor objects and tools.
The custom model fills site-specific gaps such as small fasteners, electronics
bench equipment, cables, hand tools, stationery, and room equipment. The
starter `classes.txt` contains both engineering and indoor gaps; remove classes
that are not useful in your rooms before labeling. Its line order becomes the
permanent numeric class order for that model. Training is intentionally done
on a desktop GPU or Colab, not on the Raspberry Pi.

## 1. Capture raw images on DOGZILLA

Start camera perception, then collect several short sets while changing the
room, object type, distance, camera direction, and lighting. Repeat the capture
command for each class:

```bash
dogzilla vision floor-hazards
dogzilla object-dataset-capture bolt 60 1.0
dogzilla object-dataset-capture screwdriver 60 1.0
dogzilla object-dataset-capture multimeter 60 1.0
dogzilla stop
```

The capture command writes raw frames under
`DOGZILLA/datasets/LABEL/unlabeled/`. It does not move the robot. Include
images containing multiple engineering and indoor objects plus negative images
with none of the custom classes. Avoid near-identical consecutive frames. The
capture label is a directory-safe collection name (`circuit_board`, for
example); the final detector names still come from `classes.txt`.

## 2. Label and split on a desktop

Use any YOLO bounding-box annotation tool. Import `classes.txt` in exactly its
listed order and put tight rectangles around every visible target object. An
image with none of the configured classes must have an empty `.txt` label.
Arrange the result as:

```text
dataset/
  images/train/
  images/val/
  labels/train/
  labels/val/
```

Keep entire capture sequences in only one split. The training script rejects
duplicate images across train and validation, malformed coordinates, missing
labels, out-of-range class IDs, classes with too few examples, and undersized
datasets. Its minimum is 80 train and 20 validation images, including positive
and negative examples, plus at least 20 train and five validation instances per
class. More diverse data is better.

## 3. Train and export

Run from this directory on a Linux desktop with an NVIDIA GPU:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --requirement requirements.txt
python3 train_export.py \
  --dataset /path/to/dataset \
  --classes ./classes.txt \
  --device 0
```

This produces `artifacts/dogzilla_custom.onnx`,
`artifacts/dogzilla_custom.labels`, and an evaluation report. Ultralytics and
its exported weights use AGPL-3.0; review that licence before redistribution.

## 4. Install and verify on the Pi

Copy the `.onnx` and `.labels` files to the Pi, then use the validated atomic
installer:

```bash
dogzilla object-model-custom-install /path/to/dogzilla_custom.onnx /path/to/dogzilla_custom.labels
dogzilla object-model-status
```

The installer performs a real OpenCV warm-up inference in an offline Docker
container before replacing the active custom model. An invalid labels file, a
wrong input size, or an incompatible ONNX output is rejected.

Finally test every class with objects and rooms that were not used for
training. Object detection is an aid, not a certified safety device; keep the
patrol speed low and retain physical supervision.
