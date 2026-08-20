"""OpenCV processors adapted from Yahboom's DOGZILLA vision lessons."""

import math
import time
from copy import deepcopy

import cv2
import numpy as np

from .object_detector import CORE_REQUESTED_CLASSES
from .object_detector import DANGEROUS_CLASSES
from .object_detector import REQUIRED_DANGEROUS_CLASSES
from .object_detector import validate_detection_payload
from .vision_action_policy import COLOR_ACTIONS
from .vision_action_policy import QR_ACTIONS


VISION_MODES = (
    'raw',
    'color',
    'color-track',
    'color-action',
    'face',
    'face-track',
    'watchdog',
    'qr',
    'qr-action',
    'line',
    'line-follow',
    'objects',
    'dangerous-objects',
    'floor-hazards',
    'patrol',
)

VISION_MODE_ALIASES = {
    # The installed notebook is named face_handshake, while Yahboom's public
    # lesson index calls 8.6 Watchdog.
    'face-handshake': 'watchdog',
}

COLOR_PRESETS = {
    # These are the HSV ranges used by Yahboom's DOGZILLA lessons 8.1-8.3.
    'red': ((0, 43, 46), (10, 255, 255)),
    'green': ((35, 43, 46), (77, 255, 255)),
    'blue': ((100, 43, 46), (124, 255, 255)),
    'yellow': ((26, 43, 46), (34, 255, 255)),
}

LINE_DEFAULT = ((55, 214, 183), (125, 253, 255))


class VisionConfigurationError(ValueError):
    """Raised when an unsupported vision mode or colour is requested."""


def validate_mode(value):
    """Return one normalized supported vision mode."""
    mode = str(value).strip().lower().replace('_', '-')
    mode = VISION_MODE_ALIASES.get(mode, mode)
    if mode not in VISION_MODES:
        raise VisionConfigurationError(
            f'mode must be one of: {", ".join(VISION_MODES)}'
        )
    return mode


def validate_color(value):
    """Return one normalized Yahboom colour preset."""
    color = str(value).strip().lower()
    if color not in COLOR_PRESETS:
        raise VisionConfigurationError(
            f'color must be one of: {", ".join(COLOR_PRESETS)}'
        )
    return color


def validate_request(value, *, default_mode='raw', default_color='red'):
    """Validate a web/ROS mode request without accepting extra actions."""
    if not isinstance(value, dict):
        raise VisionConfigurationError('vision request must be an object')
    return {
        'mode': validate_mode(value.get('mode', default_mode)),
        'color': validate_color(value.get('color', default_color)),
    }


def validate_face_detection_payload(value):
    """Validate one anonymous face box without accepting identity data."""
    if not isinstance(value, dict) or value.get('kind') != 'face':
        raise ValueError('face detection kind is invalid')
    box = value.get('box')
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError('face detection box is invalid')
    normalized_box = []
    for index, raw in enumerate(box):
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError('face detection box is invalid')
        number = float(raw)
        if not math.isfinite(number) or number != int(number):
            raise ValueError('face detection box is invalid')
        number = int(number)
        if number < 0 or number > 16384:
            raise ValueError('face detection box is outside image limits')
        if index >= 2 and number < 1:
            raise ValueError('face detection size is invalid')
        normalized_box.append(number)

    normalized = deepcopy(value)
    normalized['box'] = normalized_box
    for field in ('x_px', 'y_px', 'radius_px', 'error_x', 'error_y'):
        try:
            number = float(value[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'face detection {field} is invalid') from exc
        if not math.isfinite(number):
            raise ValueError(f'face detection {field} is invalid')
        normalized[field] = number
    if normalized['x_px'] < 0.0 or normalized['y_px'] < 0.0:
        raise ValueError('face detection centre is outside image limits')
    if not 0.0 < normalized['radius_px'] <= 16384.0:
        raise ValueError('face detection radius is invalid')
    if not -1.0 <= normalized['error_x'] <= 1.0:
        raise ValueError('face detection horizontal error is invalid')
    if not -1.0 <= normalized['error_y'] <= 1.0:
        raise ValueError('face detection vertical error is invalid')
    return normalized


def validate_patrol_detection_payload(value):
    """Validate one object or anonymous face emitted by patrol mode."""
    if isinstance(value, dict) and value.get('kind') == 'object':
        return validate_detection_payload(value)
    return validate_face_detection_payload(value)


def _contours(mask):
    result = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    return result[0] if len(result) == 2 else result[1]


def _target_payload(x, y, radius, width, height):
    return {
        'x_px': round(float(x), 2),
        'y_px': round(float(y), 2),
        'radius_px': round(float(radius), 2),
        'error_x': round((float(x) - width / 2.0) / (width / 2.0), 4),
        'error_y': round((float(y) - height / 2.0) / (height / 2.0), 4),
    }


class DangerConfirmationTracker:
    """Confirm a dangerous object across time, confidence, and image space."""

    def __init__(
        self,
        *,
        minimum_confidence=0.65,
        minimum_observations=3,
        minimum_duration_seconds=0.8,
        minimum_iou=0.35,
        maximum_gap_seconds=1.5,
        cooldown_seconds=8.0,
        required_label=None,
        require_dangerous=True,
    ):
        confidence = float(minimum_confidence)
        observations = int(minimum_observations)
        duration = float(minimum_duration_seconds)
        iou = float(minimum_iou)
        maximum_gap = float(maximum_gap_seconds)
        cooldown = float(cooldown_seconds)
        if not 0.6 <= confidence <= 0.99:
            raise ValueError('minimum_confidence must be from 0.6 to 0.99')
        if not 3 <= observations <= 20:
            raise ValueError('minimum_observations must be from 3 to 20')
        if not 0.75 <= duration <= 10.0:
            raise ValueError(
                'minimum_duration_seconds must be from 0.75 to 10'
            )
        if not 0.25 <= iou <= 0.95:
            raise ValueError('minimum_iou must be from 0.25 to 0.95')
        if not 0.1 <= maximum_gap <= 5.0:
            raise ValueError('maximum_gap_seconds must be from 0.1 to 5')
        if not 1.0 <= cooldown <= 300.0:
            raise ValueError('cooldown_seconds must be from 1 to 300')
        self.minimum_confidence = confidence
        self.minimum_observations = observations
        self.minimum_duration_seconds = duration
        self.minimum_iou = iou
        self.maximum_gap_seconds = maximum_gap
        self.cooldown_seconds = cooldown
        self.required_label = (
            str(required_label).strip() if required_label is not None else None
        )
        if required_label is not None and not self.required_label:
            raise ValueError('required_label cannot be empty')
        self.require_dangerous = bool(require_dangerous)
        self._tracks = {}
        self._last_confirmation = {}

    def reset(self):
        self._tracks.clear()
        self._last_confirmation.clear()

    @staticmethod
    def _box_iou(first, second):
        first_x, first_y, first_width, first_height = (
            float(value) for value in first
        )
        second_x, second_y, second_width, second_height = (
            float(value) for value in second
        )
        overlap_width = max(
            0.0,
            min(first_x + first_width, second_x + second_width)
            - max(first_x, second_x),
        )
        overlap_height = max(
            0.0,
            min(first_y + first_height, second_y + second_height)
            - max(first_y, second_y),
        )
        intersection = overlap_width * overlap_height
        union = (
            first_width * first_height
            + second_width * second_height
            - intersection
        )
        return intersection / union if union > 0.0 else 0.0

    def observe(self, detections, now=None):
        """Return confirmations earned by this frame; one frame is never enough."""
        timestamp = time.monotonic() if now is None else float(now)
        if not math.isfinite(timestamp):
            raise ValueError('observation time must be finite')
        eligible = [
            detection for detection in detections
            if (
                not self.require_dangerous
                or detection.get('dangerous') is True
            )
            and (
                self.required_label is None
                or detection.get('label') == self.required_label
            )
            and float(detection.get('confidence', -1.0))
            >= self.minimum_confidence
        ]
        strongest_by_label = {}
        for detection in eligible:
            label = str(detection.get('label', '')).strip()
            box = detection.get('box')
            if not label or not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            current = strongest_by_label.get(label)
            if current is None or detection['confidence'] > current['confidence']:
                strongest_by_label[label] = detection

        expired = [
            label for label, track in self._tracks.items()
            if timestamp - track['last_time'] > self.maximum_gap_seconds
        ]
        for label in expired:
            del self._tracks[label]

        confirmations = []
        for label, detection in strongest_by_label.items():
            track = self._tracks.get(label)
            overlap = 0.0
            if track is not None:
                overlap = self._box_iou(track['box'], detection['box'])
            continuing = (
                track is not None
                and timestamp - track['last_time'] <= self.maximum_gap_seconds
                and overlap >= self.minimum_iou
            )
            if not continuing:
                track = {
                    'first_time': timestamp,
                    'last_time': timestamp,
                    'observations': 1,
                    'box': tuple(detection['box']),
                    'lowest_confidence': float(detection['confidence']),
                    'minimum_observed_iou': 1.0,
                }
            else:
                track['last_time'] = timestamp
                track['observations'] += 1
                track['box'] = tuple(detection['box'])
                track['lowest_confidence'] = min(
                    track['lowest_confidence'],
                    float(detection['confidence']),
                )
                track['minimum_observed_iou'] = min(
                    track['minimum_observed_iou'], overlap
                )
            self._tracks[label] = track
            duration = timestamp - track['first_time']
            last_confirmation = self._last_confirmation.get(label, -math.inf)
            ready = (
                track['observations'] >= self.minimum_observations
                and duration >= self.minimum_duration_seconds
                and timestamp - last_confirmation >= self.cooldown_seconds
            )
            if not ready:
                continue
            self._last_confirmation[label] = timestamp
            confirmations.append({
                'detection': deepcopy(detection),
                'confirmation': {
                    'observations': track['observations'],
                    'duration_seconds': round(duration, 3),
                    'lowest_confidence': round(
                        track['lowest_confidence'], 4
                    ),
                    'minimum_observed_iou': round(
                        track['minimum_observed_iou'], 4
                    ),
                    'criteria': {
                        'minimum_observations': self.minimum_observations,
                        'minimum_duration_seconds': (
                            self.minimum_duration_seconds
                        ),
                        'minimum_confidence': self.minimum_confidence,
                        'minimum_iou': self.minimum_iou,
                        'maximum_gap_seconds': self.maximum_gap_seconds,
                        'cooldown_seconds': self.cooldown_seconds,
                    },
                },
            })
        return confirmations


def validate_danger_confirmation(value):
    """Validate evidence that proves a danger persisted across several frames."""
    if not isinstance(value, dict):
        raise ValueError('danger confirmation must be a JSON object')
    if value.get('kind') != 'danger-confirmation':
        raise ValueError('danger confirmation kind is invalid')
    if value.get('mode') not in {
        'dangerous-objects', 'floor-hazards', 'patrol'
    }:
        raise ValueError('danger confirmation mode is invalid')
    detection = validate_detection_payload(value.get('detection'))
    if detection.get('dangerous') is not True:
        raise ValueError('confirmed detection is not dangerous')
    evidence = value.get('confirmation')
    if not isinstance(evidence, dict):
        raise ValueError('danger confirmation evidence is missing')
    criteria = evidence.get('criteria')
    if not isinstance(criteria, dict):
        raise ValueError('danger confirmation criteria are missing')

    observations = evidence.get('observations')
    required_observations = criteria.get('minimum_observations')
    if (
        isinstance(observations, bool)
        or not isinstance(observations, int)
        or isinstance(required_observations, bool)
        or not isinstance(required_observations, int)
        or required_observations < 3
        or observations < required_observations
    ):
        raise ValueError('danger confirmation observation count is invalid')

    numeric = {}
    for field, source in (
        ('duration_seconds', evidence),
        ('lowest_confidence', evidence),
        ('minimum_observed_iou', evidence),
        ('minimum_duration_seconds', criteria),
        ('minimum_confidence', criteria),
        ('minimum_iou', criteria),
        ('maximum_gap_seconds', criteria),
        ('cooldown_seconds', criteria),
    ):
        try:
            numeric[field] = float(source[field])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f'danger confirmation {field} is invalid'
            ) from exc
        if not math.isfinite(numeric[field]):
            raise ValueError(f'danger confirmation {field} is invalid')
    if (
        numeric['minimum_duration_seconds'] < 0.75
        or numeric['duration_seconds']
        < numeric['minimum_duration_seconds']
    ):
        raise ValueError('danger confirmation duration is insufficient')
    if (
        numeric['minimum_confidence'] < 0.6
        or numeric['lowest_confidence'] < numeric['minimum_confidence']
        or detection['confidence'] < numeric['minimum_confidence']
    ):
        raise ValueError('danger confirmation confidence is insufficient')
    if (
        numeric['minimum_iou'] < 0.25
        or numeric['minimum_observed_iou'] < numeric['minimum_iou']
    ):
        raise ValueError('danger confirmation spatial match is insufficient')
    if not 0.1 <= numeric['maximum_gap_seconds'] <= 2.0:
        raise ValueError('danger confirmation maximum gap is unsafe')
    if numeric['cooldown_seconds'] < 1.0:
        raise ValueError('danger confirmation cooldown is unsafe')
    return {
        **deepcopy(value),
        'detection': detection,
    }


class VisionProcessor:
    """Process BGR frames without owning a camera or robot controller."""

    def __init__(
        self,
        mode='raw',
        color='red',
        line_hsv=LINE_DEFAULT,
        object_perception=None,
    ):
        self.mode = validate_mode(mode)
        self.color = validate_color(color)
        self.line_hsv = self._validate_hsv_range(line_hsv)
        self.object_perception = object_perception
        cascade_path = (
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self._face = cv2.CascadeClassifier(cascade_path)
        if self._face.empty():
            raise RuntimeError(
                f'cannot load OpenCV face cascade: {cascade_path}'
            )
        self._qr = cv2.QRCodeDetector()

    @staticmethod
    def _validate_hsv_range(value):
        try:
            lower, upper = value
            lower = tuple(int(item) for item in lower)
            upper = tuple(int(item) for item in upper)
        except (TypeError, ValueError) as exc:
            raise VisionConfigurationError(
                'HSV range must contain two three-number sequences'
            ) from exc
        if len(lower) != 3 or len(upper) != 3:
            raise VisionConfigurationError(
                'HSV range must contain two three-number sequences'
            )
        limits = (179, 255, 255)
        if any(
            not 0 <= low <= high <= limit
            for low, high, limit in zip(lower, upper, limits)
        ):
            raise VisionConfigurationError(
                'HSV bounds are outside OpenCV limits'
            )
        return lower, upper

    def configure(self, mode, color):
        """Atomically apply a validated mode and colour preset."""
        requested = validate_request({'mode': mode, 'color': color})
        mode_changed = requested['mode'] != self.mode
        self.mode = requested['mode']
        self.color = requested['color']
        if (
            mode_changed
            and self.object_perception is not None
            and hasattr(self.object_perception, 'reset_cache')
        ):
            self.object_perception.reset_cache()
        return requested

    @staticmethod
    def _base_result(mode, width, height):
        return {
            'schema_version': 1,
            'mode': mode,
            'image': {'width': int(width), 'height': int(height)},
            'detected': False,
            'detections': [],
            'action_proposals': [],
            'action_output': 'disabled',
        }

    def process(self, frame):
        """Return an annotated BGR frame and JSON-safe detection result."""
        if not isinstance(frame, np.ndarray):
            raise TypeError('frame must be a numpy array')
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError('frame must have BGR shape height x width x 3')
        if frame.dtype != np.uint8:
            raise ValueError('frame must use uint8 pixels')
        if frame.size == 0:
            raise ValueError('frame cannot be empty')

        started = time.perf_counter()
        annotated = frame.copy()
        height, width = frame.shape[:2]
        result = self._base_result(self.mode, width, height)

        if self.mode in {'color', 'color-track', 'color-action'}:
            self._process_color(frame, annotated, result)
        elif self.mode in {'face', 'face-track', 'watchdog'}:
            self._process_face(frame, annotated, result)
        elif self.mode in {'qr', 'qr-action'}:
            self._process_qr(frame, annotated, result)
        elif self.mode in {'line', 'line-follow'}:
            self._process_line(frame, annotated, result)
        elif self.mode in {
            'objects', 'dangerous-objects', 'floor-hazards', 'patrol'
        }:
            self._process_objects(frame, annotated, result)

        self._add_action_proposals(result)

        label = self.mode.replace('-', ' ').upper()
        cv2.putText(
            annotated,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (80, 240, 165),
            2,
            cv2.LINE_AA,
        )
        result['processing_ms'] = round(
            (time.perf_counter() - started) * 1000.0,
            3,
        )
        return annotated, result

    def object_status(self):
        """Return explicit model readiness and requested-class coverage."""
        if self.object_perception is None:
            return {
                'ready': False,
                'requested_classes': list(CORE_REQUESTED_CLASSES),
                'covered_classes': [],
                'missing_classes': list(CORE_REQUESTED_CLASSES),
                'missing_dangerous_classes': sorted(
                    REQUIRED_DANGEROUS_CLASSES
                ),
                'dangerous_coverage_complete': False,
                'person_detection_ready': False,
                'required_dangerous_classes': sorted(
                    REQUIRED_DANGEROUS_CLASSES
                ),
                'optional_dangerous_classes_missing': sorted(
                    DANGEROUS_CLASSES - REQUIRED_DANGEROUS_CLASSES
                ),
                'models': [],
                'reason': 'no object model is loaded',
            }
        return {
            'ready': True,
            **self.object_perception.coverage(),
        }

    def face_status(self):
        """Return the readiness of the local, non-identifying face detector."""
        return {
            'ready': self._face is not None,
            'method': 'opencv-haar-frontal-face',
            'identification': False,
        }

    def _process_objects(self, frame, annotated, result):
        status = self.object_status()
        result['object_detection'] = status
        if self.object_perception is None:
            if self.mode == 'patrol':
                faces = self._detect_faces(frame, annotated)
                result.update(
                    detected=bool(faces),
                    detections=faces,
                    dangerous_object_count=0,
                    person_count=0,
                    face_count=len(faces),
                    floor_hazard_count=0,
                    small_floor_hazard_count=0,
                )
            return
        detections = self.object_perception.detect(
            frame,
            focus_floor=self.mode in {'floor-hazards', 'patrol'},
        )
        if self.mode == 'floor-hazards':
            detections = [
                detection for detection in detections
                if detection['floor_candidate']
            ]
        elif self.mode == 'dangerous-objects':
            detections = [
                detection for detection in detections
                if detection['dangerous'] is True
            ]
        elif self.mode == 'patrol':
            detections = [
                detection for detection in detections
                if detection['dangerous'] is True
                or detection['label'] == 'person'
            ]
        rendered = self.object_perception.annotate(frame, detections)
        annotated[:] = rendered
        faces = []
        if self.mode == 'patrol':
            faces = self._detect_faces(frame, annotated)
        result.update(
            detected=bool(detections or faces),
            detections=detections + faces,
            dangerous_object_count=sum(
                1 for detection in detections
                if detection['dangerous'] is True
            ),
            person_count=sum(
                1 for detection in detections
                if detection['label'] == 'person'
            ),
            face_count=len(faces),
            floor_hazard_count=sum(
                1 for detection in detections
                if detection['floor_hazard']
            ),
            small_floor_hazard_count=sum(
                1 for detection in detections
                if detection.get('small_floor_hazard') is True
                and detection['floor_candidate']
            ),
        )

    def render_object_result(self, frame, result=None):
        """Draw the newest object result over a current camera frame."""
        annotated = frame.copy()
        detections = [] if result is None else result.get('detections', [])
        objects = [
            detection for detection in detections
            if detection.get('kind') == 'object'
        ]
        if self.object_perception is not None and objects:
            annotated = self.object_perception.annotate(annotated, objects)
        for detection in detections:
            if detection.get('kind') != 'face':
                continue
            x, y, width, height = detection['box']
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                (255, 190, 75),
                2,
            )
            cv2.putText(
                annotated,
                'FACE',
                (x, max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 190, 75),
                2,
                cv2.LINE_AA,
            )
        mode = self.mode if result is None else result.get('mode', self.mode)
        cv2.putText(
            annotated,
            str(mode).replace('-', ' ').upper(),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (80, 240, 165),
            2,
            cv2.LINE_AA,
        )
        return annotated

    @staticmethod
    def _firmware_action_proposal(source, action_id, name):
        return {
            'kind': 'firmware-action',
            'source': source,
            'action_id': int(action_id),
            'name': str(name),
            'requires_explicit_arming': True,
            'executed': False,
        }

    def _add_action_proposals(self, result):
        """Attach Yahboom-compatible intent without executing robot motion."""
        detections = result['detections']
        width = result['image']['width']
        height = result['image']['height']
        proposals = result['action_proposals']

        if self.mode == 'color-action' and detections:
            target = detections[0]
            centered = (
                width * 220.0 / 640.0
                <= target['x_px']
                <= width * 420.0 / 640.0
                and height * 140.0 / 480.0
                <= target['y_px']
                <= height * 340.0 / 480.0
            )
            if target['radius_px'] > height * 60.0 / 480.0 and centered:
                action_id, name = COLOR_ACTIONS[self.color]
                proposals.append(self._firmware_action_proposal(
                    'yahboom-lesson-8.3',
                    action_id,
                    name,
                ))

        elif self.mode == 'watchdog' and detections:
            target = detections[0]
            _, _, face_width, face_height = target['box']
            centered = (
                width * 150.0 / 640.0
                <= target['x_px']
                <= width * 450.0 / 640.0
                and height * 100.0 / 480.0
                <= target['y_px']
                <= height * 380.0 / 480.0
            )
            if (
                face_width > width * 60.0 / 640.0
                and face_height > height * 60.0 / 480.0
                and centered
            ):
                proposals.append(self._firmware_action_proposal(
                    'yahboom-lesson-8.6',
                    19,
                    'handshake',
                ))

        elif self.mode == 'qr-action':
            for detection in detections:
                command = str(detection.get('text', '')).strip().upper()
                action = QR_ACTIONS.get(command)
                if action is None:
                    continue
                action_id, name = action
                proposal = self._firmware_action_proposal(
                    'yahboom-lesson-8.8',
                    action_id,
                    name,
                )
                proposal['matched_text'] = command
                proposals.append(proposal)

        elif self.mode == 'line-follow' and detections:
            target = detections[0]
            proposals.append({
                'kind': 'velocity-intent',
                'source': 'yahboom-lessons-8.11-8.12',
                'name': 'line-follow',
                'steering_error': target['error_x'],
                'reference_forward_command': 25,
                'requires_explicit_arming': True,
                'executed': False,
            })

    def _process_color(self, frame, annotated, result):
        height, width = frame.shape[:2]
        lower, upper = COLOR_PRESETS[self.color]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((5, 5), dtype=np.uint8),
        )
        contours = _contours(mask)
        if not contours:
            result['color'] = self.color
            return
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if area < 80.0 or radius < 5.0:
            result['color'] = self.color
            return
        target = _target_payload(x, y, radius, width, height)
        target.update({
            'kind': 'color',
            'color': self.color,
            'area_px': round(area, 2),
        })
        result.update(
            detected=True,
            color=self.color,
            detections=[target],
        )
        cv2.circle(
            annotated,
            (int(round(x)), int(round(y))),
            int(round(radius)),
            (255, 0, 255),
            2,
        )
        cv2.circle(
            annotated,
            (int(round(x)), int(round(y))),
            4,
            (255, 255, 255),
            -1,
        )
        cv2.putText(
            annotated,
            self.color,
            (max(0, int(x - radius)), max(48, int(y - radius - 8))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _detect_faces(self, frame, annotated):
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self._face.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=4,
            minSize=(32, 32),
        )
        detections = []
        for x, y, face_width, face_height in faces:
            center_x = x + face_width / 2.0
            center_y = y + face_height / 2.0
            radius = math.hypot(face_width, face_height) / 2.0
            target = _target_payload(
                center_x,
                center_y,
                radius,
                width,
                height,
            )
            target.update({
                'kind': 'face',
                'box': [int(x), int(y), int(face_width), int(face_height)],
            })
            detections.append(target)
            cv2.rectangle(
                annotated,
                (int(x), int(y)),
                (int(x + face_width), int(y + face_height)),
                (0, 220, 80),
                2,
            )
        detections.sort(key=lambda item: item['radius_px'], reverse=True)
        return detections

    def _process_face(self, frame, annotated, result):
        detections = self._detect_faces(frame, annotated)
        result.update(detected=bool(detections), detections=detections)

    def _process_qr(self, frame, annotated, result):
        detections = []
        try:
            found, decoded, points, _ = self._qr.detectAndDecodeMulti(frame)
        except (cv2.error, ValueError):
            found, decoded, points = False, (), None
        if found and points is not None:
            for text, corners in zip(decoded, points):
                polygon = np.asarray(corners, dtype=np.int32).reshape(-1, 2)
                cv2.polylines(annotated, [polygon], True, (0, 230, 255), 2)
                x, y, width, height = cv2.boundingRect(polygon)
                safe_text = str(text)[:256]
                detections.append({
                    'kind': 'qr',
                    'text': safe_text,
                    'box': [int(x), int(y), int(width), int(height)],
                })
                cv2.putText(
                    annotated,
                    safe_text[:48] or 'QR',
                    (int(x), max(48, int(y - 8))),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 230, 255),
                    2,
                    cv2.LINE_AA,
                )
        else:
            text, corners, _ = self._qr.detectAndDecode(frame)
            if corners is not None and len(corners):
                polygon = np.asarray(corners, dtype=np.int32).reshape(-1, 2)
                cv2.polylines(annotated, [polygon], True, (0, 230, 255), 2)
                x, y, width, height = cv2.boundingRect(polygon)
                detections.append({
                    'kind': 'qr',
                    'text': str(text)[:256],
                    'box': [int(x), int(y), int(width), int(height)],
                })
        result.update(detected=bool(detections), detections=detections)

    def _process_line(self, frame, annotated, result):
        height, width = frame.shape[:2]
        roi_top = height // 2
        roi = frame[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower, upper = self.line_hsv
        mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
        )
        contours = _contours(mask)
        cv2.line(annotated, (0, roi_top), (width, roi_top), (90, 90, 90), 1)
        if not contours:
            result['roi_top_px'] = roi_top
            return
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < 80.0:
            result['roi_top_px'] = roi_top
            return
        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-9:
            result['roi_top_px'] = roi_top
            return
        center_x = moments['m10'] / moments['m00']
        center_y = roi_top + moments['m01'] / moments['m00']
        shifted = contour.copy()
        shifted[:, :, 1] += roi_top
        x, y, box_width, box_height = cv2.boundingRect(shifted)
        radius = max(box_width, box_height) / 2.0
        target = _target_payload(
            center_x,
            center_y,
            radius,
            width,
            height,
        )
        target.update({
            'kind': 'line',
            'area_px': round(area, 2),
            'box': [int(x), int(y), int(box_width), int(box_height)],
        })
        result.update(
            detected=True,
            roi_top_px=roi_top,
            detections=[target],
        )
        cv2.drawContours(annotated, [shifted], -1, (255, 130, 40), 2)
        cv2.circle(
            annotated,
            (int(round(center_x)), int(round(center_y))),
            5,
            (255, 255, 255),
            -1,
        )
