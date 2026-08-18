"""ROS 2 camera-topic processor for safe DOGZILLA vision lessons."""

import json
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from .object_detector import CORE_REQUESTED_CLASSES
from .object_detector import load_labels
from .object_detector import OPEN_IMAGES_V7_RELEVANT_CLASSES
from .object_detector import ObjectDetectorError
from .object_detector import ObjectPerception
from .object_detector import YoloV8OpenCvDetector
from .object_detector import YoloXOpenCvDetector
from .vision_core import validate_request
from .vision_core import VisionConfigurationError
from .vision_core import VisionProcessor


def image_to_bgr(message):
    """Convert common ROS Image encodings without requiring cv_bridge."""
    encodings = {
        'bgr8': (3, None),
        'rgb8': (3, cv2.COLOR_RGB2BGR),
        'bgra8': (4, cv2.COLOR_BGRA2BGR),
        'rgba8': (4, cv2.COLOR_RGBA2BGR),
        'mono8': (1, cv2.COLOR_GRAY2BGR),
    }
    if message.encoding not in encodings:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    channels, conversion = encodings[message.encoding]
    width = int(message.width)
    height = int(message.height)
    row_bytes = int(message.step)
    minimum_step = width * channels
    if width <= 0 or height <= 0 or row_bytes < minimum_step:
        raise ValueError('invalid image dimensions or row step')
    required = row_bytes * height
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < required:
        raise ValueError('image payload is shorter than height * step')
    packed = raw[:required].reshape(height, row_bytes)[:, :minimum_step]
    if channels == 1:
        image = packed.reshape(height, width)
    else:
        image = packed.reshape(height, width, channels)
    if conversion is not None:
        image = cv2.cvtColor(image, conversion)
    return np.ascontiguousarray(image)


class DogzillaVisionNode(Node):
    """Annotate the shared camera stream and publish JSON detections."""

    def __init__(self):
        super().__init__('dogzilla_vision')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('mode', 'raw')
        self.declare_parameter('color', 'red')
        self.declare_parameter('process_hz', 10.0)
        self.declare_parameter('object_process_hz', 2.0)
        self.declare_parameter('jpeg_quality', 75)
        package_share = get_package_share_directory('dogzilla_slam')
        self.declare_parameter(
            'object_model_path',
            '/models/yolox_nano.onnx',
        )
        self.declare_parameter(
            'object_labels_path',
            os.path.join(package_share, 'config', 'coco80.txt'),
        )
        self.declare_parameter(
            'open_images_model_path',
            '/models/yolov8n-oiv7.onnx',
        )
        self.declare_parameter(
            'custom_object_model_path',
            '/models/dogzilla_custom.onnx',
        )
        self.declare_parameter(
            'custom_object_labels_path',
            '/models/dogzilla_custom.labels',
        )
        self.declare_parameter(
            'yoloe_object_model_path',
            '/models/yoloe_small_hazards.onnx',
        )
        self.declare_parameter(
            'yoloe_object_labels_path',
            '/models/yoloe_small_hazards.labels',
        )
        self.declare_parameter('object_confidence', 0.35)
        self.declare_parameter('object_nms', 0.45)
        self.declare_parameter('floor_scan_columns', 2)
        self.declare_parameter('floor_scan_overlap', 0.18)

        mode = str(self.get_parameter('mode').value)
        color = str(self.get_parameter('color').value)
        object_perception = self._load_object_perception()
        self._processor = VisionProcessor(
            mode=mode,
            color=color,
            object_perception=object_perception,
        )
        self._lock = threading.RLock()
        self._last_process = 0.0
        self._sequence = 0
        self._frame_failures = 0
        self._process_hz = float(self.get_parameter('process_hz').value)
        self._object_process_hz = float(
            self.get_parameter('object_process_hz').value
        )
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not 1.0 <= self._process_hz <= 30.0:
            raise ValueError('process_hz must be between 1 and 30')
        if not 0.2 <= self._object_process_hz <= 10.0:
            raise ValueError('object_process_hz must be between 0.2 and 10')
        if not 40 <= self._jpeg_quality <= 95:
            raise ValueError('jpeg_quality must be between 40 and 95')

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._frame_publisher = self.create_publisher(
            CompressedImage,
            '/vision/annotated/compressed',
            qos_profile_sensor_data,
        )
        self._detections_publisher = self.create_publisher(
            String,
            '/vision/detections',
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            '/vision/status',
            status_qos,
        )
        self._image_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self._command_subscription = self.create_subscription(
            String,
            '/vision/mode_command',
            self._on_command,
            10,
        )
        self.add_on_set_parameters_callback(self._parameters_changed)
        self._status_timer = self.create_timer(
            2.0,
            lambda: self._publish_status('ready'),
        )
        self._publish_status('ready')
        self.get_logger().info(
            'Safe vision processor active: '
            f'mode={self._processor.mode}, color={self._processor.color}, '
            f'input={self.get_parameter("image_topic").value}, '
            'robot action output disabled'
        )

    def _load_object_perception(self):
        confidence = float(self.get_parameter('object_confidence').value)
        nms = float(self.get_parameter('object_nms').value)
        detectors = []
        configurations = (
            (
                'coco-yolox-nano',
                str(self.get_parameter('object_model_path').value),
                str(self.get_parameter('object_labels_path').value),
                False,
            ),
        )
        for name, model_path, labels_path, optional in configurations:
            model_exists = os.path.isfile(model_path)
            labels_exist = os.path.isfile(labels_path)
            if optional and not model_exists and not labels_exist:
                continue
            if not model_exists or not labels_exist:
                self.get_logger().warn(
                    f'Object model {name} is incomplete: model={model_exists}, '
                    f'labels={labels_exist}'
                )
                continue
            try:
                detector = YoloXOpenCvDetector(
                    model_path,
                    load_labels(labels_path),
                    name=name,
                    confidence_threshold=confidence,
                    nms_threshold=nms,
                )
            except (ObjectDetectorError, ValueError) as exc:
                self.get_logger().error(
                    f'Object model {name} was not loaded: {exc}'
                )
                continue
            detectors.append(detector)
            self.get_logger().info(
                f'Loaded object model {name}: '
                f'{len(detector.metadata.labels)} classes'
            )
        open_images_path = str(
            self.get_parameter('open_images_model_path').value
        )
        if os.path.isfile(open_images_path):
            try:
                detector = YoloV8OpenCvDetector(
                    open_images_path,
                    OPEN_IMAGES_V7_RELEVANT_CLASSES,
                    name='open-images-yolov8-nano',
                    confidence_threshold=confidence,
                    nms_threshold=nms,
                )
            except (ObjectDetectorError, ValueError) as exc:
                self.get_logger().error(
                    f'Open Images model was not loaded: {exc}'
                )
            else:
                detectors.append(detector)
                self.get_logger().info(
                    'Loaded Open Images model for indoor, tool, and hazard '
                    'coverage'
                )
        yoloe_model_path = str(
            self.get_parameter('yoloe_object_model_path').value
        )
        yoloe_labels_path = str(
            self.get_parameter('yoloe_object_labels_path').value
        )
        yoloe_model_exists = os.path.isfile(yoloe_model_path)
        yoloe_labels_exist = os.path.isfile(yoloe_labels_path)
        if yoloe_model_exists and yoloe_labels_exist:
            try:
                yoloe_labels = load_labels(yoloe_labels_path)
                detector = YoloV8OpenCvDetector(
                    yoloe_model_path,
                    dict(enumerate(yoloe_labels)),
                    name='yoloe-small-floor-hazards',
                    input_size=640,
                    output_class_count=len(yoloe_labels),
                    output_extra_channels=32,
                    confidence_threshold=confidence,
                    nms_threshold=nms,
                )
                detector.detect(np.full((640, 640, 3), 114, dtype=np.uint8))
            except (ObjectDetectorError, ValueError) as exc:
                self.get_logger().error(
                    f'YOLOE floor-hazard model was not loaded: {exc}'
                )
            else:
                detectors.append(detector)
                self.get_logger().info(
                    'Loaded prompt-baked YOLOE floor-hazard model: '
                    f'{len(yoloe_labels)} classes'
                )
        elif yoloe_model_exists or yoloe_labels_exist:
            self.get_logger().warn(
                'YOLOE floor-hazard model is incomplete: '
                f'model={yoloe_model_exists}, labels={yoloe_labels_exist}'
            )
        custom_model_path = str(
            self.get_parameter('custom_object_model_path').value
        )
        custom_labels_path = str(
            self.get_parameter('custom_object_labels_path').value
        )
        custom_model_exists = os.path.isfile(custom_model_path)
        custom_labels_exist = os.path.isfile(custom_labels_path)
        if custom_model_exists and custom_labels_exist:
            try:
                custom_labels = load_labels(custom_labels_path)
                detector = YoloV8OpenCvDetector(
                    custom_model_path,
                    dict(enumerate(custom_labels)),
                    name='dogzilla-custom-yolov8',
                    input_size=416,
                    output_class_count=len(custom_labels),
                    confidence_threshold=confidence,
                    nms_threshold=nms,
                )
                # A blank inference verifies the graph and output dimensions.
                # Declared labels alone must never make patrol coverage ready.
                detector.detect(np.full((416, 416, 3), 114, dtype=np.uint8))
            except (ObjectDetectorError, ValueError) as exc:
                self.get_logger().error(
                    f'Custom YOLOv8 model was not loaded: {exc}'
                )
            else:
                detectors.append(detector)
                self.get_logger().info(
                    'Loaded custom YOLOv8 object model: '
                    f'{len(custom_labels)} classes'
                )
        elif custom_model_exists or custom_labels_exist:
            self.get_logger().warn(
                'Custom object model is incomplete: '
                f'model={custom_model_exists}, labels={custom_labels_exist}'
            )
        if not detectors:
            self.get_logger().warn(
                'No object model is ready; object modes will report unavailable'
            )
            return None
        perception = ObjectPerception(
            detectors,
            requested_classes=CORE_REQUESTED_CLASSES,
            floor_scan_columns=int(
                self.get_parameter('floor_scan_columns').value
            ),
            floor_scan_overlap=float(
                self.get_parameter('floor_scan_overlap').value
            ),
        )
        missing = perception.coverage()['missing_dangerous_classes']
        if missing:
            self.get_logger().warn(
                'Dangerous-object coverage is incomplete; custom model needed '
                f'for: {", ".join(missing)}'
            )
        return perception

    def _status(self, state, error=None):
        value = {
            'schema_version': 1,
            'state': state,
            'mode': self._processor.mode,
            'color': self._processor.color,
            'process_hz': self._process_hz,
            'object_process_hz': self._object_process_hz,
            'action_output': 'disabled',
            'object_detection': self._processor.object_status(),
        }
        if error:
            value['error'] = str(error)
        return value

    def _publish_status(self, state, error=None):
        message = String()
        message.data = json.dumps(
            self._status(state, error),
            separators=(',', ':'),
            allow_nan=False,
        )
        self._status_publisher.publish(message)

    def _configure(self, request):
        validated = validate_request(
            request,
            default_mode=self._processor.mode,
            default_color=self._processor.color,
        )
        with self._lock:
            self._processor.configure(
                validated['mode'],
                validated['color'],
            )
        self._publish_status('ready')
        self.get_logger().info(
            f'Vision mode changed: {validated["mode"]}, '
            f'color={validated["color"]}'
        )
        return validated

    def _on_command(self, message):
        try:
            value = json.loads(message.data)
            self._configure(value)
        except (json.JSONDecodeError, VisionConfigurationError) as exc:
            self.get_logger().warn(f'Ignored invalid vision command: {exc}')
            self._publish_status('ready', exc)

    def _parameters_changed(self, parameters):
        requested = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in {'mode', 'color'}
        }
        if not requested:
            return SetParametersResult(successful=True)
        try:
            self._configure(requested)
        except VisionConfigurationError as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _on_image(self, message):
        now = time.monotonic()
        rate = (
            self._object_process_hz
            if self._processor.mode in {'objects', 'floor-hazards'}
            else self._process_hz
        )
        if now - self._last_process < 1.0 / rate:
            return
        self._last_process = now
        try:
            frame = image_to_bgr(message)
            with self._lock:
                annotated, result = self._processor.process(frame)
            encoded, jpeg = cv2.imencode(
                '.jpg',
                annotated,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not encoded:
                raise RuntimeError('OpenCV JPEG encoder returned no image')
        except Exception as exc:
            self._frame_failures += 1
            if self._frame_failures == 1 or self._frame_failures % 30 == 0:
                self.get_logger().error(
                    f'Vision frame processing failed '
                    f'({self._frame_failures} failures): {exc}'
                )
            return

        self._frame_failures = 0
        self._sequence += 1
        result.update({
            'sequence': self._sequence,
            'source_frame': message.header.frame_id,
            'stamp': {
                'sec': int(message.header.stamp.sec),
                'nanosec': int(message.header.stamp.nanosec),
            },
        })
        detection_message = String()
        detection_message.data = json.dumps(
            result,
            separators=(',', ':'),
            allow_nan=False,
        )
        self._detections_publisher.publish(detection_message)

        frame_message = CompressedImage()
        frame_message.header = message.header
        frame_message.format = 'jpeg'
        frame_message.data = jpeg.tobytes()
        self._frame_publisher.publish(frame_message)


def main(args=None):
    rclpy.init(args=args)
    node = DogzillaVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
