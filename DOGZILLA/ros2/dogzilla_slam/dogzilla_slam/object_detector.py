"""Lightweight OpenCV-DNN object detection and floor-hazard policy."""

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np


ORIGINAL_REQUESTED_CLASSES = (
    'pen',
    'scissors',
    'bottle',
    'knife',
    'gun',
    'nail',
    'bolt',
    'hammer',
)

ENGINEERING_CLASSES = (
    'adhesive tape',
    'allen key',
    'axe',
    'battery',
    'beaker',
    'bottle opener',
    'bolt',
    'breadboard',
    'caliper',
    'can opener',
    'chainsaw',
    'chisel',
    'circuit board',
    'clamp',
    'crimping tool',
    'drill',
    'electrical tape',
    'extension cord',
    'file',
    'fire extinguisher',
    'flashlight',
    'glove',
    'grinder',
    'hacksaw',
    'hammer',
    'heat gun',
    'helmet',
    'hex key',
    'ladder',
    'level',
    'multimeter',
    'nail',
    'nut',
    'oscilloscope',
    'paint can',
    'paper cutter',
    'pliers',
    'power supply',
    'pry bar',
    'ratchet',
    'ruler',
    'safety glasses',
    'saw',
    'screw',
    'screwdriver',
    'socket',
    'soldering iron',
    'stapler',
    'tape measure',
    'tool',
    'toolbox',
    'tripod',
    'utility knife',
    'vise',
    'washer',
    'wire',
    'wire cutter',
    'work light',
    'wrench',
)

# Small hazards need a separate catalog because most are only a few pixels in
# a full 640x480 frame. Some are supplied by Open Images (for example nail),
# while the remaining labels can be baked into an optional YOLOE ONNX export.
SMALL_FLOOR_HAZARD_CLASSES = (
    'blade fragment',
    'bolt',
    'ceramic shard',
    'drill bit',
    'glass shard',
    'metal shard',
    'nail',
    'needle',
    'razor blade',
    'screw',
    'sharp debris',
    'splinter',
    'staple',
    'thumbtack',
    'wire',
)

GENERAL_INDOOR_CLASSES = (
    'backpack',
    'bed',
    'binder',
    'blender',
    'book',
    'bowl',
    'box',
    'briefcase',
    'cabinet',
    'calculator',
    'camera',
    'candle',
    'chair',
    'charger',
    'clock',
    'coat',
    'coffee maker',
    'countertop',
    'cup',
    'curtain',
    'cutting board',
    'desk',
    'dishwasher',
    'door',
    'drawer',
    'eraser',
    'envelope',
    'fan',
    'fork',
    'frying pan',
    'hair dryer',
    'handbag',
    'headphones',
    'heater',
    'humidifier',
    'jug',
    'kettle',
    'keyboard',
    'lamp',
    'laptop',
    'laundry basket',
    'light bulb',
    'light switch',
    'marker',
    'microwave',
    'mirror',
    'monitor',
    'mouse',
    'nightstand',
    'notebook',
    'oven',
    'paper',
    'pencil',
    'pencil case',
    'pencil sharpener',
    'picture frame',
    'pillow',
    'plant',
    'plastic bag',
    'plate',
    'phone',
    'power outlet',
    'poster',
    'printer',
    'refrigerator',
    'remote control',
    'router',
    'scale',
    'shelf',
    'sink',
    'smoke detector',
    'soap dispenser',
    'sofa',
    'spoon',
    'stool',
    'suitcase',
    'table',
    'tablet',
    'toaster',
    'toilet',
    'tin can',
    'toothbrush',
    'towel',
    'tv',
    'umbrella',
    'vacuum cleaner',
    'vase',
    'washing machine',
    'waste bin',
    'whiteboard',
    'window',
    'window blind',
)

CORE_REQUESTED_CLASSES = tuple(dict.fromkeys(
    ORIGINAL_REQUESTED_CLASSES
    + ENGINEERING_CLASSES
    + GENERAL_INDOOR_CLASSES
    + SMALL_FLOOR_HAZARD_CLASSES
))

DANGEROUS_CLASSES = frozenset({
    'allen key',
    'axe',
    'battery',
    'gun',
    'knife',
    'nail',
    'bolt',
    'chainsaw',
    'chisel',
    'clamp',
    'drill',
    'extension cord',
    'file',
    'grinder',
    'hacksaw',
    'hammer',
    'heat gun',
    'hex key',
    'multimeter',
    'nut',
    'paper cutter',
    'pliers',
    'power supply',
    'pry bar',
    'ratchet',
    'saw',
    'screw',
    'screwdriver',
    'socket',
    'soldering iron',
    'sword',
    'syringe',
    'tape measure',
    'toolbox',
    'utility knife',
    'vise',
    'washer',
    'wire',
    'wire cutter',
    'wrench',
}) | frozenset(SMALL_FLOOR_HAZARD_CLASSES)

CAUTION_CLASSES = frozenset({'scissors'})

INDOOR_CLASSES = frozenset({
    'backpack',
    'bed',
    'book',
    'bottle',
    'bowl',
    'chair',
    'clock',
    'couch',
    'cup',
    'dining table',
    'fork',
    'hammer',
    'keyboard',
    'knife',
    'laptop',
    'mouse',
    'nail',
    'pen',
    'remote',
    'scissors',
    'spoon',
    'toothbrush',
    'tv',
    'vase',
}) | (
    frozenset(ENGINEERING_CLASSES)
    | frozenset(GENERAL_INDOOR_CLASSES)
    | frozenset(SMALL_FLOOR_HAZARD_CLASSES)
)

LABEL_ALIASES = {
    'alarm clock': 'clock',
    'bathroom cabinet': 'cabinet',
    'bookcase': 'shelf',
    'cell phone': 'phone',
    'chest of drawers': 'drawer',
    'coffee cup': 'cup',
    'coffee table': 'table',
    'coffeemaker': 'coffee maker',
    'firearm': 'gun',
    'handgun': 'gun',
    'hand gun': 'gun',
    'pistol': 'gun',
    'revolver': 'gun',
    'rifle': 'gun',
    'shotgun': 'gun',
    'guns': 'gun',
    'nails': 'nail',
    'nail construction': 'nail',
    'bolts': 'bolt',
    'blade fragments': 'blade fragment',
    'broken ceramic': 'ceramic shard',
    'broken glass': 'glass shard',
    'ceramic fragment': 'ceramic shard',
    'ceramic fragments': 'ceramic shard',
    'ceramic shards': 'ceramic shard',
    'drill bits': 'drill bit',
    'glass fragment': 'glass shard',
    'glass fragments': 'glass shard',
    'glass shards': 'glass shard',
    'metal fragment': 'metal shard',
    'metal fragments': 'metal shard',
    'metal shards': 'metal shard',
    'needles': 'needle',
    'razor': 'razor blade',
    'razor blades': 'razor blade',
    'screws': 'screw',
    'sharp fragment': 'sharp debris',
    'sharp fragments': 'sharp debris',
    'splinters': 'splinter',
    'staples': 'staple',
    'tack': 'thumbtack',
    'tacks': 'thumbtack',
    'thumb tack': 'thumbtack',
    'thumb tacks': 'thumbtack',
    'thumbtacks': 'thumbtack',
    'cable': 'wire',
    'cables': 'wire',
    'ceiling fan': 'fan',
    'corded phone': 'phone',
    'couch': 'sofa',
    'cupboard': 'cabinet',
    'cabinetry': 'cabinet',
    'dagger': 'knife',
    'drill tool': 'drill',
    'dining table': 'table',
    'mallet': 'hammer',
    'mechanical fan': 'fan',
    'mobile phone': 'phone',
    'mug': 'cup',
    'kitchen & dining room table': 'table',
    'kitchen and dining room table': 'table',
    'loveseat': 'sofa',
    'measuring tape': 'tape measure',
    'microwave oven': 'microwave',
    'potted plant': 'plant',
    'power plugs and sockets': 'power outlet',
    'ratchet device': 'ratchet',
    'remote': 'remote control',
    'ring binder': 'binder',
    'sofa bed': 'sofa',
    'studio couch': 'sofa',
    'telephone': 'phone',
    'torch': 'flashlight',
    'waste container': 'waste bin',
    'wardrobe': 'cabinet',
    'kitchen knife': 'knife',
}

# Class IDs from the official Open Images V7 601-class ordering. Only labels
# relevant to this robot are emitted; COCO remains the general indoor model.
OPEN_IMAGES_V7_RELEVANT_CLASSES = {
    1: 'adhesive tape',
    4: 'clock',
    14: 'axe',
    15: 'backpack',
    30: 'cabinet',
    32: 'beaker',
    34: 'bed',
    50: 'blender',
    54: 'book',
    55: 'cabinet',
    57: 'bottle',
    58: 'bottle opener',
    60: 'bowl',
    62: 'box',
    66: 'briefcase',
    77: 'cabinet',
    80: 'calculator',
    82: 'camera',
    83: 'can opener',
    85: 'candle',
    100: 'fan',
    103: 'chainsaw',
    104: 'chair',
    107: 'drawer',
    110: 'chisel',
    113: 'clock',
    114: 'cabinet',
    116: 'coat',
    121: 'cup',
    122: 'table',
    123: 'coffee maker',
    127: 'keyboard',
    128: 'monitor',
    129: 'mouse',
    136: 'sofa',
    137: 'countertop',
    147: 'cabinet',
    148: 'curtain',
    149: 'cutting board',
    150: 'knife',
    153: 'desk',
    159: 'dishwasher',
    164: 'door',
    168: 'drawer',
    170: 'drill',
    180: 'envelope',
    181: 'eraser',
    189: 'cabinet',
    194: 'flashlight',
    196: 'plant',
    204: 'fork',
    211: 'frying pan',
    218: 'glove',
    220: 'safety glasses',
    228: 'grinder',
    231: 'hair dryer',
    234: 'hammer',
    237: 'handbag',
    238: 'gun',
    244: 'headphones',
    245: 'heater',
    248: 'helmet',
    258: 'plant',
    272: 'humidifier',
    286: 'jug',
    289: 'kettle',
    290: 'table',
    292: 'knife',
    296: 'knife',
    298: 'ladder',
    301: 'lamp',
    304: 'laptop',
    308: 'light bulb',
    309: 'light switch',
    329: 'fan',
    332: 'microwave',
    335: 'mirror',
    339: 'phone',
    345: 'cup',
    350: 'nail',
    352: 'nightstand',
    360: 'oven',
    367: 'paper cutter',
    376: 'pen',
    377: 'pencil case',
    378: 'pencil sharpener',
    386: 'picture frame',
    388: 'pillow',
    393: 'plant',
    394: 'plastic bag',
    395: 'plate',
    403: 'poster',
    405: 'power outlet',
    408: 'printer',
    415: 'ratchet',
    419: 'refrigerator',
    420: 'remote control',
    423: 'gun',
    424: 'binder',
    429: 'ruler',
    436: 'scale',
    438: 'scissors',
    441: 'screwdriver',
    453: 'shelf',
    457: 'gun',
    460: 'sink',
    475: 'soap dispenser',
    477: 'sofa',
    483: 'spoon',
    490: 'stapler',
    494: 'stool',
    499: 'sofa',
    503: 'suitcase',
    512: 'sword',
    513: 'syringe',
    514: 'table',
    516: 'tablet',
    526: 'phone',
    527: 'tv',
    535: 'tin can',
    537: 'toaster',
    538: 'toilet',
    541: 'tool',
    542: 'toothbrush',
    545: 'towel',
    555: 'tripod',
    562: 'umbrella',
    565: 'vase',
    574: 'cabinet',
    575: 'washing machine',
    576: 'waste bin',
    585: 'whiteboard',
    587: 'window',
    588: 'window blind',
    598: 'wrench',
}

DEFAULT_FLOOR_ROI = (
    (0.08, 0.52),
    (0.92, 0.52),
    (1.00, 1.00),
    (0.00, 1.00),
)


class ObjectDetectorError(RuntimeError):
    """Raised when a configured object model cannot be used safely."""


def canonical_label(value):
    """Normalize model labels before applying the hazard policy."""
    label = ' '.join(
        str(value).strip().lower().replace('_', ' ').replace('-', ' ').split()
    )
    return LABEL_ALIASES.get(label, label)


def load_labels(path):
    """Load one non-empty label per line with deterministic ordering."""
    label_path = Path(path)
    try:
        labels = [
            canonical_label(line)
            for line in label_path.read_text(encoding='utf-8').splitlines()
            if line.strip() and not line.lstrip().startswith('#')
        ]
    except OSError as exc:
        raise ObjectDetectorError(
            f'cannot read object labels: {label_path}: {exc}'
        ) from exc
    if not labels:
        raise ObjectDetectorError(f'object label file is empty: {label_path}')
    if len(labels) != len(set(labels)):
        raise ObjectDetectorError(
            f'object label file contains duplicate canonical labels: {label_path}'
        )
    return tuple(labels)


@dataclass(frozen=True)
class DetectorMetadata:
    """Immutable model identity and declared class coverage."""

    name: str
    labels: tuple
    model_path: str


class YoloXOpenCvDetector:
    """Run a decoded-or-raw YOLOX ONNX model using only OpenCV DNN."""

    def __init__(
        self,
        model_path,
        labels,
        *,
        name='yolox',
        input_size=416,
        confidence_threshold=0.35,
        nms_threshold=0.45,
        maximum_detections=50,
        network=None,
    ):
        size = int(input_size)
        confidence = float(confidence_threshold)
        nms = float(nms_threshold)
        maximum = int(maximum_detections)
        if size < 160 or size > 1280 or size % 32:
            raise ValueError('input_size must be a multiple of 32 from 160 to 1280')
        if not 0.05 <= confidence <= 0.99:
            raise ValueError('confidence_threshold must be from 0.05 to 0.99')
        if not 0.05 <= nms <= 0.95:
            raise ValueError('nms_threshold must be from 0.05 to 0.95')
        if not 1 <= maximum <= 300:
            raise ValueError('maximum_detections must be from 1 to 300')
        normalized_labels = tuple(canonical_label(item) for item in labels)
        if not normalized_labels or len(normalized_labels) != len(set(normalized_labels)):
            raise ValueError('labels must contain unique non-empty values')

        self.input_size = size
        self.confidence_threshold = confidence
        self.nms_threshold = nms
        self.maximum_detections = maximum
        self.metadata = DetectorMetadata(
            name=str(name).strip() or 'yolox',
            labels=normalized_labels,
            model_path=str(model_path),
        )
        if network is None:
            model = Path(model_path)
            if not model.is_file() or model.stat().st_size < 1024:
                raise ObjectDetectorError(
                    f'object model is missing or invalid: {model}'
                )
            try:
                network = cv2.dnn.readNetFromONNX(str(model))
            except cv2.error as exc:
                raise ObjectDetectorError(
                    f'OpenCV could not load object model {model}: {exc}'
                ) from exc
        self._network = network
        self._grid, self._strides = self._make_grid(size)

    @staticmethod
    def _make_grid(size):
        grids = []
        expanded_strides = []
        for stride in (8, 16, 32):
            count = size // stride
            grid_y, grid_x = np.meshgrid(
                np.arange(count),
                np.arange(count),
                indexing='ij',
            )
            grid = np.stack((grid_x, grid_y), axis=2).reshape(-1, 2)
            grids.append(grid)
            expanded_strides.append(
                np.full((grid.shape[0], 1), stride, dtype=np.float32)
            )
        return (
            np.concatenate(grids).astype(np.float32),
            np.concatenate(expanded_strides).astype(np.float32),
        )

    def _preprocess(self, frame):
        height, width = frame.shape[:2]
        ratio = min(self.input_size / height, self.input_size / width)
        resized_width = max(1, int(width * ratio))
        resized_height = max(1, int(height * ratio))
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
        padded = np.full(
            (self.input_size, self.input_size, 3),
            114,
            dtype=np.uint8,
        )
        padded[:resized_height, :resized_width] = resized
        blob = np.ascontiguousarray(
            padded.transpose(2, 0, 1)[None],
            dtype=np.float32,
        )
        return blob, ratio

    def _decode(self, output):
        predictions = np.asarray(output, dtype=np.float32)
        predictions = predictions.reshape(-1, predictions.shape[-1]).copy()
        if predictions.shape[1] != len(self.metadata.labels) + 5:
            raise ObjectDetectorError(
                'YOLOX output class count does not match the labels file: '
                f'{predictions.shape[1] - 5} != {len(self.metadata.labels)}'
            )
        if predictions.shape[0] == self._grid.shape[0]:
            predictions[:, :2] = (
                predictions[:, :2] + self._grid
            ) * self._strides
            predictions[:, 2:4] = (
                np.exp(np.clip(predictions[:, 2:4], -10.0, 10.0))
                * self._strides
            )
        return predictions

    def detect(self, frame):
        """Return normalized box detections for one BGR image."""
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
            raise TypeError('object detector frame must be a uint8 numpy array')
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError('object detector frame must have non-empty BGR shape')
        blob, ratio = self._preprocess(frame)
        try:
            self._network.setInput(blob)
            predictions = self._decode(self._network.forward())
        except cv2.error as exc:
            raise ObjectDetectorError(f'OpenCV object inference failed: {exc}') from exc

        class_ids = np.argmax(predictions[:, 5:], axis=1)
        scores = predictions[:, 4] * predictions[
            np.arange(len(predictions)), class_ids + 5
        ]
        selected = np.flatnonzero(scores >= self.confidence_threshold)
        if not len(selected):
            return []

        boxes_xywh = predictions[selected, :4].copy()
        boxes_xywh[:, 0] -= boxes_xywh[:, 2] / 2.0
        boxes_xywh[:, 1] -= boxes_xywh[:, 3] / 2.0
        boxes_xywh /= ratio
        frame_height, frame_width = frame.shape[:2]
        boxes = []
        filtered_scores = []
        filtered_classes = []
        for box, score, class_id in zip(
            boxes_xywh,
            scores[selected],
            class_ids[selected],
        ):
            x = max(0.0, min(float(box[0]), frame_width - 1.0))
            y = max(0.0, min(float(box[1]), frame_height - 1.0))
            width = max(0.0, min(float(box[2]), frame_width - x))
            height = max(0.0, min(float(box[3]), frame_height - y))
            if width < 2.0 or height < 2.0:
                continue
            boxes.append([x, y, width, height])
            filtered_scores.append(float(score))
            filtered_classes.append(int(class_id))
        if not boxes:
            return []

        indices = []
        for class_id in sorted(set(filtered_classes)):
            candidates = [
                index for index, value in enumerate(filtered_classes)
                if value == class_id
            ]
            keep = cv2.dnn.NMSBoxes(
                [boxes[index] for index in candidates],
                [filtered_scores[index] for index in candidates],
                self.confidence_threshold,
                self.nms_threshold,
            )
            if keep is None or not len(keep):
                continue
            indices.extend(
                candidates[int(index)]
                for index in np.asarray(keep).reshape(-1)
            )
        indices.sort(key=lambda index: filtered_scores[index], reverse=True)
        detections = []
        for index in indices[:self.maximum_detections]:
            x, y, width, height = boxes[index]
            class_id = filtered_classes[index]
            detections.append({
                'label': self.metadata.labels[class_id],
                'confidence': filtered_scores[index],
                'box': (x, y, width, height),
                'class_id': class_id,
                'model': self.metadata.name,
            })
        return detections


class YoloV8OpenCvDetector:
    """Run a raw Ultralytics YOLOv8 ONNX detector with OpenCV DNN only."""

    def __init__(
        self,
        model_path,
        class_map,
        *,
        name='yolov8',
        input_size=640,
        output_class_count=601,
        confidence_threshold=0.35,
        nms_threshold=0.45,
        maximum_detections=50,
        output_extra_channels=0,
        network=None,
    ):
        size = int(input_size)
        class_count = int(output_class_count)
        confidence = float(confidence_threshold)
        nms = float(nms_threshold)
        maximum = int(maximum_detections)
        extra_channels = int(output_extra_channels)
        if size < 160 or size > 1280 or size % 32:
            raise ValueError('input_size must be a multiple of 32 from 160 to 1280')
        if class_count < 1:
            raise ValueError('output_class_count must be positive')
        if not 0.05 <= confidence <= 0.99:
            raise ValueError('confidence_threshold must be from 0.05 to 0.99')
        if not 0.05 <= nms <= 0.95:
            raise ValueError('nms_threshold must be from 0.05 to 0.95')
        if not 1 <= maximum <= 300:
            raise ValueError('maximum_detections must be from 1 to 300')
        if not 0 <= extra_channels <= 256:
            raise ValueError('output_extra_channels must be from 0 to 256')
        try:
            normalized_map = {
                int(class_id): canonical_label(label)
                for class_id, label in class_map.items()
            }
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError('class_map must map integer IDs to labels') from exc
        if not normalized_map or any(
            not 0 <= class_id < class_count or not label
            for class_id, label in normalized_map.items()
        ):
            raise ValueError('class_map contains an invalid class ID or label')

        self.input_size = size
        self.output_class_count = class_count
        self.class_map = normalized_map
        self.confidence_threshold = confidence
        self.nms_threshold = nms
        self.maximum_detections = maximum
        self.output_extra_channels = extra_channels
        self.metadata = DetectorMetadata(
            name=str(name).strip() or 'yolov8',
            labels=tuple(dict.fromkeys(normalized_map.values())),
            model_path=str(model_path),
        )
        if network is None:
            model = Path(model_path)
            if not model.is_file() or model.stat().st_size < 1024:
                raise ObjectDetectorError(
                    f'object model is missing or invalid: {model}'
                )
            try:
                network = cv2.dnn.readNetFromONNX(str(model))
            except cv2.error as exc:
                raise ObjectDetectorError(
                    f'OpenCV could not load object model {model}: {exc}'
                ) from exc
        self._network = network

    def _preprocess(self, frame):
        height, width = frame.shape[:2]
        ratio = min(self.input_size / height, self.input_size / width)
        resized_width = max(1, round(width * ratio))
        resized_height = max(1, round(height * ratio))
        resized = cv2.resize(
            frame,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )
        horizontal = self.input_size - resized_width
        vertical = self.input_size - resized_height
        left = horizontal // 2
        top = vertical // 2
        padded = cv2.copyMakeBorder(
            resized,
            top,
            vertical - top,
            left,
            horizontal - left,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(self.input_size, self.input_size),
            swapRB=True,
            crop=False,
        )
        return blob, ratio, left, top

    def _predictions(self, output):
        outputs = output if isinstance(output, (list, tuple)) else (output,)
        expected = (
            self.output_class_count + 4 + self.output_extra_channels
        )
        shapes = []
        for candidate in outputs:
            predictions = np.asarray(candidate, dtype=np.float32)
            shapes.append(tuple(predictions.shape))
            if predictions.ndim == 3 and predictions.shape[0] == 1:
                predictions = predictions[0]
            if predictions.ndim != 2:
                continue
            if predictions.shape[0] == expected:
                return predictions.T
            if predictions.shape[1] == expected:
                return predictions
        raise ObjectDetectorError(
            'YOLOv8 output class count does not match configuration: '
            f'{shapes} does not contain {expected} channels'
        )

    def _forward(self):
        get_names = getattr(
            self._network,
            'getUnconnectedOutLayersNames',
            None,
        )
        if get_names is None:
            return self._network.forward()
        names = tuple(get_names())
        if len(names) <= 1:
            return self._network.forward()
        return self._network.forward(names)

    def detect(self, frame):
        """Return selected Open Images detections for one BGR image."""
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
            raise TypeError('object detector frame must be a uint8 numpy array')
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError('object detector frame must have non-empty BGR shape')
        blob, ratio, pad_left, pad_top = self._preprocess(frame)
        try:
            self._network.setInput(blob)
            predictions = self._predictions(self._forward())
        except cv2.error as exc:
            raise ObjectDetectorError(f'OpenCV object inference failed: {exc}') from exc

        selected_ids = np.asarray(tuple(self.class_map), dtype=np.int64)
        selected_scores = predictions[:, selected_ids + 4]
        best_offsets = np.argmax(selected_scores, axis=1)
        scores = selected_scores[np.arange(len(predictions)), best_offsets]
        candidates = np.flatnonzero(scores >= self.confidence_threshold)
        if not len(candidates):
            return []

        frame_height, frame_width = frame.shape[:2]
        boxes = []
        filtered_scores = []
        filtered_ids = []
        for index in candidates:
            center_x, center_y, width, height = predictions[index, :4]
            x = (float(center_x - width / 2.0) - pad_left) / ratio
            y = (float(center_y - height / 2.0) - pad_top) / ratio
            width = float(width) / ratio
            height = float(height) / ratio
            x = max(0.0, min(x, frame_width - 1.0))
            y = max(0.0, min(y, frame_height - 1.0))
            width = max(0.0, min(width, frame_width - x))
            height = max(0.0, min(height, frame_height - y))
            if width < 2.0 or height < 2.0:
                continue
            boxes.append([x, y, width, height])
            filtered_scores.append(float(scores[index]))
            filtered_ids.append(int(selected_ids[best_offsets[index]]))
        if not boxes:
            return []

        indices = []
        labels = [self.class_map[class_id] for class_id in filtered_ids]
        for label in sorted(set(labels)):
            group = [index for index, value in enumerate(labels) if value == label]
            keep = cv2.dnn.NMSBoxes(
                [boxes[index] for index in group],
                [filtered_scores[index] for index in group],
                self.confidence_threshold,
                self.nms_threshold,
            )
            if keep is None or not len(keep):
                continue
            indices.extend(group[int(index)] for index in np.asarray(keep).reshape(-1))
        indices.sort(key=lambda index: filtered_scores[index], reverse=True)
        detections = []
        for index in indices[:self.maximum_detections]:
            x, y, width, height = boxes[index]
            detections.append({
                'label': labels[index],
                'confidence': filtered_scores[index],
                'box': (x, y, width, height),
                'class_id': filtered_ids[index],
                'model': self.metadata.name,
            })
        return detections


class ObjectPerception:
    """Merge detector outputs and add an explicit, auditable risk policy."""

    def __init__(
        self,
        detectors=(),
        *,
        floor_roi=DEFAULT_FLOOR_ROI,
        requested_classes=CORE_REQUESTED_CLASSES,
        floor_scan_columns=2,
        floor_scan_overlap=0.18,
    ):
        self.detectors = tuple(detectors)
        if not self.detectors:
            raise ValueError('at least one object detector is required')
        try:
            roi = tuple((float(x), float(y)) for x, y in floor_roi)
        except (TypeError, ValueError) as exc:
            raise ValueError('floor_roi must be normalized x/y points') from exc
        if len(roi) < 3 or any(
            not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0)
            for x, y in roi
        ):
            raise ValueError('floor_roi needs at least three normalized points')
        self.floor_roi = roi
        columns = int(floor_scan_columns)
        overlap = float(floor_scan_overlap)
        if not 1 <= columns <= 4:
            raise ValueError('floor_scan_columns must be from 1 to 4')
        if not 0.0 <= overlap <= 0.45:
            raise ValueError('floor_scan_overlap must be from 0 to 0.45')
        self.floor_scan_columns = columns
        self.floor_scan_overlap = overlap
        self.requested_classes = tuple(
            canonical_label(item) for item in requested_classes
        )

    @property
    def available_classes(self):
        labels = set()
        for detector in self.detectors:
            labels.update(detector.metadata.labels)
        return frozenset(labels)

    def coverage(self):
        available = self.available_classes
        missing = [
            label for label in self.requested_classes if label not in available
        ]
        missing_dangerous = [
            label for label in sorted(DANGEROUS_CLASSES)
            if label not in available
        ]
        return {
            'requested_classes': list(self.requested_classes),
            'covered_classes': [
                label for label in self.requested_classes if label in available
            ],
            'missing_classes': missing,
            'missing_dangerous_classes': missing_dangerous,
            'dangerous_coverage_complete': not missing_dangerous,
            'models': [detector.metadata.name for detector in self.detectors],
            'small_floor_hazard_classes': list(SMALL_FLOOR_HAZARD_CLASSES),
            'small_floor_hazard_covered_classes': [
                label for label in SMALL_FLOOR_HAZARD_CLASSES
                if label in available
            ],
            'floor_scan': {
                'enabled': True,
                'columns': self.floor_scan_columns,
                'overlap': self.floor_scan_overlap,
            },
        }

    def _is_floor_candidate(self, box, width, height):
        x, y, box_width, box_height = box
        point = (
            (x + box_width / 2.0) / width,
            (y + box_height) / height,
        )
        polygon = np.asarray(self.floor_roi, dtype=np.float32)
        return cv2.pointPolygonTest(polygon, point, False) >= 0

    def _floor_tiles(self, frame):
        """Return overlapping lower-image crops and their pixel offsets."""
        height, width = frame.shape[:2]
        minimum_y = min(point[1] for point in self.floor_roi)
        top = max(0, min(height - 2, int(math.floor(minimum_y * height))))
        floor_height = height - top
        if floor_height < 32 or width < 64:
            return ()

        columns = self.floor_scan_columns
        if columns == 1:
            return ((frame[top:height, 0:width], 0, top, 'floor-tile-1'),)
        nominal_width = width / columns
        margin = nominal_width * self.floor_scan_overlap
        tiles = []
        for index in range(columns):
            left = max(0, int(math.floor(index * nominal_width - margin)))
            right = min(
                width,
                int(math.ceil((index + 1) * nominal_width + margin)),
            )
            if right - left < 32:
                continue
            tiles.append((
                frame[top:height, left:right],
                left,
                top,
                f'floor-tile-{index + 1}',
            ))
        return tuple(tiles)

    @staticmethod
    def _remap_detections(detections, offset_x, offset_y, scan):
        remapped = []
        for item in detections:
            x, y, width, height = (float(value) for value in item['box'])
            remapped.append({
                **item,
                'box': (x + offset_x, y + offset_y, width, height),
                'scan': scan,
            })
        return remapped

    def _detect_with_floor_focus(self, detector, frame):
        detections = self._remap_detections(
            detector.detect(frame),
            0,
            0,
            'full-frame',
        )
        labels = frozenset(detector.metadata.labels)
        if not labels.intersection(SMALL_FLOOR_HAZARD_CLASSES):
            return detections
        for tile, offset_x, offset_y, scan in self._floor_tiles(frame):
            detections.extend(self._remap_detections(
                detector.detect(tile),
                offset_x,
                offset_y,
                scan,
            ))
        return detections

    def detect(self, frame, *, focus_floor=False):
        height, width = frame.shape[:2]
        combined = []
        for detector in self.detectors:
            if focus_floor:
                combined.extend(
                    self._detect_with_floor_focus(detector, frame)
                )
            else:
                combined.extend(self._remap_detections(
                    detector.detect(frame),
                    0,
                    0,
                    'full-frame',
                ))
        combined.sort(key=lambda item: float(item['confidence']), reverse=True)

        detections = []
        for item in combined:
            label = canonical_label(item['label'])
            x, y, box_width, box_height = (
                float(value) for value in item['box']
            )
            if any(
                existing['label'] == label
                and self._intersection_over_union(
                    (x, y, box_width, box_height),
                    existing['_raw_box'],
                ) >= 0.55
                for existing in detections
            ):
                continue
            floor_candidate = self._is_floor_candidate(
                (x, y, box_width, box_height),
                width,
                height,
            )
            dangerous = label in DANGEROUS_CLASSES
            small_floor_hazard = label in SMALL_FLOOR_HAZARD_CLASSES
            caution = label in CAUTION_CLASSES
            if label == 'gun':
                risk = 'critical'
            elif dangerous:
                risk = 'danger'
            elif caution:
                risk = 'caution'
            else:
                risk = 'normal'
            center_x = x + box_width / 2.0
            center_y = y + box_height / 2.0
            detection = {
                'kind': 'object',
                'label': label,
                'confidence': round(float(item['confidence']), 4),
                'box': [
                    int(round(x)),
                    int(round(y)),
                    int(round(box_width)),
                    int(round(box_height)),
                ],
                'x_px': round(center_x, 2),
                'y_px': round(center_y, 2),
                'error_x': round((center_x - width / 2.0) / (width / 2.0), 4),
                'error_y': round((center_y - height / 2.0) / (height / 2.0), 4),
                'category': 'indoor' if label in INDOOR_CLASSES else 'general',
                'risk': risk,
                'dangerous': dangerous,
                'small_floor_hazard': small_floor_hazard,
                'floor_candidate': floor_candidate,
                'floor_hazard': bool(dangerous and floor_candidate),
                'model': str(item.get('model', 'unknown')),
                'class_id': int(item.get('class_id', -1)),
                'scan': str(item.get('scan', 'full-frame')),
                '_raw_box': (x, y, box_width, box_height),
            }
            detections.append(detection)
        for detection in detections:
            detection.pop('_raw_box', None)
        return detections

    @staticmethod
    def _intersection_over_union(left, right):
        left_x, left_y, left_width, left_height = left
        right_x, right_y, right_width, right_height = right
        x1 = max(left_x, right_x)
        y1 = max(left_y, right_y)
        x2 = min(left_x + left_width, right_x + right_width)
        y2 = min(left_y + left_height, right_y + right_height)
        intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        union = left_width * left_height + right_width * right_height - intersection
        return intersection / union if union > 0.0 else 0.0

    @staticmethod
    def annotate(frame, detections):
        """Draw readable risk-coded boxes without modifying the input frame."""
        annotated = frame.copy()
        for detection in detections:
            x, y, width, height = detection['box']
            if detection['floor_hazard']:
                color = (30, 30, 255)
            elif detection['dangerous']:
                color = (0, 110, 255)
            elif detection['risk'] == 'caution':
                color = (0, 210, 255)
            else:
                color = (80, 220, 120)
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                color,
                2,
            )
            floor_label = ' · FLOOR' if detection['floor_candidate'] else ''
            label = (
                f"{detection['label']} {detection['confidence']:.2f}"
                f'{floor_label}'
            )
            cv2.putText(
                annotated,
                label,
                (max(0, x), max(20, y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )
        return annotated


def validate_detection_payload(value):
    """Validate one detector result before it enters patrol safety logic."""
    if not isinstance(value, dict) or value.get('kind') != 'object':
        raise ValueError('object detection must be a JSON object with kind=object')
    label = canonical_label(value.get('label', ''))
    if not label:
        raise ValueError('object detection label is missing')
    confidence = float(value.get('confidence', math.nan))
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError('object detection confidence must be from 0 to 1')
    box = value.get('box')
    if not isinstance(box, list) or len(box) != 4:
        raise ValueError('object detection box must contain x, y, width, height')
    if any(not isinstance(item, int) or item < 0 for item in box):
        raise ValueError('object detection box values must be non-negative integers')
    dangerous = label in DANGEROUS_CLASSES
    small_floor_hazard = label in SMALL_FLOOR_HAZARD_CLASSES
    floor_candidate = value.get('floor_candidate') is True
    if value.get('dangerous') is not dangerous:
        raise ValueError('object detection dangerous flag contradicts policy')
    if value.get('floor_hazard') is not bool(dangerous and floor_candidate):
        raise ValueError('object detection floor_hazard flag contradicts policy')
    supplied_small = value.get('small_floor_hazard', small_floor_hazard)
    if supplied_small is not small_floor_hazard:
        raise ValueError(
            'object detection small_floor_hazard flag contradicts policy'
        )
    return {
        **value,
        'label': label,
        'confidence': confidence,
        'small_floor_hazard': small_floor_hazard,
    }
