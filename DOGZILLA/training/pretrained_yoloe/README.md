# Pretrained YOLOE small-hazard model

This optional model adds a focused 34-label zero-shot vocabulary for hazardous
tools, engineering components, sharp fragments, loose wiring, puddles, and
similar floor hazards. It uses pretrained YOLOE weights and **does not train or
fine-tune a model**. Prompts are baked into a static ONNX graph because OpenCV
DNN cannot change text prompts at runtime. The default checkpoint is the
smallest YOLOE-26N prompt model.

The deployed vocabulary is the exact ordered list in `prompts.txt`. It includes
common tools such as hammers, drills, scissors, and utility knives; components
such as nuts, washers, bearings, circuit boards, and connectors; and hazards
such as broken glass, metal shavings, loose cables, exposed wires, batteries,
and puddles. Labels describe what the model is asked to find, not a guarantee
that every object will be detected.

`broken glass` replaces the older `glass shard` prompt because the runtime
safety policy treats those phrases as the same canonical hazard label.

Export on a 64-bit Linux desktop or Raspberry Pi. The requirements select the
official CPU-only PyTorch build, so a GPU and CUDA are not needed:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --requirement requirements.txt
python3 export_yoloe.py --prompts prompts.txt --device cpu
```

The export environment is intentionally separate from the DOGZILLA runtime.
PyTorch and Ultralytics are needed only to produce the ONNX file; the deployed
robot loads that file through OpenCV DNN.

The command creates these files under `artifacts/`:

- `yoloe_small_hazards.onnx`
- `yoloe_small_hazards.labels`
- `yoloe_small_hazards.json`

Copy the ONNX and labels files to DOGZILLA, then install them atomically:

```bash
dogzilla object-model-yoloe-install yoloe_small_hazards.onnx yoloe_small_hazards.labels
dogzilla object-model-status
```

The installer loads the graph through the same OpenCV version used at runtime
and performs a blank-image inference before replacing the active files. The
runtime scans both the full frame and two overlapping floor crops for these
small classes. This increases sensitivity but also increases CPU time.

Zero-shot vocabulary is not proof of reliable detection. Before patrol is
trusted, test each real hazard under the actual camera angle, floor, distance,
and lighting with `dogzilla object-check LABEL 30 --floor`. Never scatter
sharp objects where the robot or a person can step on them; use safe replicas
or contained samples for testing.
