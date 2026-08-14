"""OpenCV processors adapted from Yahboom's DOGZILLA vision lessons."""

import math
import time

import cv2
import numpy as np

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


class VisionProcessor:
    """Process BGR frames without owning a camera or robot controller."""

    def __init__(self, mode='raw', color='red', line_hsv=LINE_DEFAULT):
        self.mode = validate_mode(mode)
        self.color = validate_color(color)
        self.line_hsv = self._validate_hsv_range(line_hsv)
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
        self.mode = requested['mode']
        self.color = requested['color']
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

    def _process_face(self, frame, annotated, result):
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
