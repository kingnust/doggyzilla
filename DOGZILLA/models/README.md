# DOGZILLA object models

Runtime weights live here and are intentionally excluded from Git. Install the
pinned base model with:

```bash
dogzilla object-model-install
```

The installer retrieves two checksum-pinned models. Both are run directly by
OpenCV DNN; PyTorch and the Ultralytics Python package are not installed.

- Official Apache-2.0 YOLOX-Nano COCO export for common indoor objects.
- Ultralytics YOLOv8-Nano Open Images V7 export from X-AnyLabeling for a broad
  selection of tools, hazards, furniture, appliances, office supplies, and
  everyday room objects. These weights are AGPL-3.0; review that licence
  before redistributing a system containing the downloaded file.

Full requested hazard coverage needs both files below:

- `dogzilla_custom.onnx`
- `dogzilla_custom.labels`

The labels file must contain one class per line. The custom model must be a
fixed 416x416 Ultralytics YOLOv8 detection export with exactly the same number
and order of output classes. Install it through
`dogzilla object-model-custom-install MODEL LABELS`; the command performs an
offline OpenCV warm-up inference before replacing the active files.

Open Images and COCO cover a broad indoor and engineering catalog, including
drills, screwdrivers, wrenches, hammers, nails, knives, guns, pens, scissors,
cameras, furniture, doors, windows, office supplies, kitchen equipment, and
appliances. Small hardware, electronics-bench equipment, cables, and several
site-specific room items are not reliably covered, so those classes need a
custom model. The configurable capture, validation, training, export, and
installation workflow is documented in
`training/custom_objects/README.md`. A site-specific model is recommended for
all small floor objects because generic pretrained models are not a safety
guarantee.

When labeled training data is not available, the optional prompt-baked YOLOE
model uses pretrained open-vocabulary weights:

- `yoloe_small_hazards.onnx`
- `yoloe_small_hazards.labels`

Generate it with `training/pretrained_yoloe/export_yoloe.py`, then install it
with `dogzilla object-model-yoloe-install MODEL LABELS`. Prompt baking is an
export step, not training. The runtime validates the fixed 640x640
segmentation graph through OpenCV and scans two overlapping floor crops in
addition to the full frame. The model targets screws, nails, bolts, staples,
needles, small blades, splinters, wire, and glass, ceramic, or metal shards.
The segmentation mask output is intentionally ignored for now; patrol uses
the model's bounding boxes and the configured floor polygon.

The software reports missing coverage explicitly. It does not treat an absent
class as evidence that the room is safe. Marked-area patrol also stays locked
while any configured dangerous class is missing, the coverage metadata is
inconsistent, the detector is stale, or camera vision can directly actuate the
robot.

## Verify a real object

After starting standalone camera-only Vision, use `object-check` to require
three detections above 0.55 confidence from distinct frames:

```bash
dogzilla vision objects
dogzilla object-check bottle 15
dogzilla object-check hammer 15 --floor
dogzilla stop
```

The first argument is the target label. `15` is the maximum test duration in
seconds. `--floor` additionally requires the bottom of the detected box to be
inside the configured floor region. The check refuses armed Vision Control,
fails immediately when the loaded models do not cover the requested label,
and saves a JSON report under `logs/object-checks/`.
