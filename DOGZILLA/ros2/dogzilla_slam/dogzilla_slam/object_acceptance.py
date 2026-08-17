"""Repeatable camera-only acceptance check for one object class."""

import argparse
import json
import math
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .object_detector import canonical_label
from .object_detector import validate_detection_payload


LABEL_PATTERN = re.compile(r'^[a-z][a-z0-9 ]{0,39}$')


class AcceptanceError(ValueError):
    """Raised when live evidence cannot satisfy the acceptance contract."""


def validate_arguments(label, duration, confidence, minimum_hits):
    """Normalize and bound acceptance-test arguments."""
    target = canonical_label(label)
    if not LABEL_PATTERN.fullmatch(target):
        raise AcceptanceError(
            'label must start with a letter and use letters, numbers, '
            'underscores, hyphens, or spaces'
        )
    duration = float(duration)
    confidence = float(confidence)
    minimum_hits = int(minimum_hits)
    if not math.isfinite(duration) or not 2.0 <= duration <= 120.0:
        raise AcceptanceError('duration must be from 2 to 120 seconds')
    if not math.isfinite(confidence) or not 0.05 <= confidence <= 0.99:
        raise AcceptanceError('confidence must be from 0.05 to 0.99')
    if not 1 <= minimum_hits <= 30:
        raise AcceptanceError('minimum hits must be from 1 to 30')
    return target, duration, confidence, minimum_hits


class ObjectAcceptanceSession:
    """Accumulate independent frames until one target has repeated evidence."""

    def __init__(
        self,
        label,
        *,
        require_floor=False,
        confidence=0.55,
        minimum_hits=3,
    ):
        target, _, confidence, minimum_hits = validate_arguments(
            label,
            15.0,
            confidence,
            minimum_hits,
        )
        self.label = target
        self.require_floor = bool(require_floor)
        self.confidence = confidence
        self.minimum_hits = minimum_hits
        self.coverage_verified = False
        self.models = []
        self.frames = 0
        self.matching_frames = 0
        self.floor_matching_frames = 0
        self.best_confidence = 0.0
        self._sequences = set()

    @property
    def passed(self):
        return (
            self.coverage_verified
            and self.matching_frames >= self.minimum_hits
        )

    def accept_status(self, payload):
        """Require a ready, non-actuating object detector covering the label."""
        if not isinstance(payload, dict):
            raise AcceptanceError('vision status must be a JSON object')
        if payload.get('state') != 'ready':
            raise AcceptanceError('vision processor is not ready')
        if payload.get('action_output') != 'disabled':
            raise AcceptanceError(
                'vision action output must be disabled for object acceptance'
            )
        object_status = payload.get('object_detection')
        if not isinstance(object_status, dict) or not object_status.get('ready'):
            raise AcceptanceError('object detector is not ready')
        covered = {
            canonical_label(value)
            for value in object_status.get('covered_classes', [])
        }
        if self.label not in covered:
            raise AcceptanceError(
                f'{self.label} is not covered by the loaded models'
            )
        models = object_status.get('models', [])
        if not isinstance(models, list) or not all(
            isinstance(value, str) and value for value in models
        ):
            raise AcceptanceError('vision status has invalid model metadata')
        self.models = list(models)
        self.coverage_verified = True

    def accept_detections(self, payload):
        """Count at most one qualifying hit from each unique camera frame."""
        if not isinstance(payload, dict):
            raise AcceptanceError('vision detections must be a JSON object')
        if payload.get('mode') not in {'objects', 'floor-hazards'}:
            raise AcceptanceError(
                'vision mode must be objects or floor-hazards'
            )
        sequence = payload.get('sequence')
        if not isinstance(sequence, int) or sequence < 1:
            raise AcceptanceError('vision detection sequence is invalid')
        if sequence in self._sequences:
            return False
        self._sequences.add(sequence)
        self.frames += 1

        matches = []
        values = payload.get('detections')
        if not isinstance(values, list):
            raise AcceptanceError('vision detections must be a list')
        for value in values:
            try:
                detection = validate_detection_payload(value)
            except (TypeError, ValueError) as exc:
                raise AcceptanceError(str(exc)) from exc
            if detection['label'] != self.label:
                continue
            if detection['confidence'] < self.confidence:
                continue
            if self.require_floor and not detection['floor_candidate']:
                continue
            matches.append(detection)
        if not matches:
            return False

        best = max(matches, key=lambda value: value['confidence'])
        self.matching_frames += 1
        if best['floor_candidate']:
            self.floor_matching_frames += 1
        self.best_confidence = max(
            self.best_confidence,
            best['confidence'],
        )
        return True

    def report(self, *, duration, failure=None, interrupted=False):
        """Build one stable machine-readable result."""
        reason = None
        if failure:
            reason = str(failure)
        elif interrupted:
            reason = 'test interrupted by operator'
        elif not self.coverage_verified:
            reason = 'no ready vision status was received'
        elif self.matching_frames < self.minimum_hits:
            qualifier = ' floor' if self.require_floor else ''
            reason = (
                f'needed {self.minimum_hits}{qualifier} matching frames; '
                f'received {self.matching_frames}'
            )
        return {
            'schema_version': 1,
            'passed': reason is None,
            'label': self.label,
            'require_floor': self.require_floor,
            'minimum_confidence': self.confidence,
            'minimum_hits': self.minimum_hits,
            'duration_seconds': round(float(duration), 3),
            'coverage_verified': self.coverage_verified,
            'models': self.models,
            'frames': self.frames,
            'matching_frames': self.matching_frames,
            'floor_matching_frames': self.floor_matching_frames,
            'best_confidence': round(self.best_confidence, 4),
            'reason': reason,
        }


class ObjectAcceptanceNode(Node):
    """Subscribe only to vision evidence; publish no commands or motion."""

    def __init__(self, session):
        super().__init__('dogzilla_object_acceptance')
        self.session = session
        self.failure = None
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            '/vision/status',
            self._on_status,
            status_qos,
        )
        self.create_subscription(
            String,
            '/vision/detections',
            self._on_detections,
            10,
        )

    def _on_status(self, message):
        if self.failure is not None:
            return
        try:
            self.session.accept_status(json.loads(message.data))
        except (AcceptanceError, json.JSONDecodeError) as exc:
            self.failure = exc

    def _on_detections(self, message):
        if self.failure is not None:
            return
        try:
            self.session.accept_detections(json.loads(message.data))
        except (AcceptanceError, json.JSONDecodeError) as exc:
            self.failure = exc


def parser():
    """Build the operator command parser."""
    value = argparse.ArgumentParser(
        description='Verify repeated live detection of one object class.',
    )
    value.add_argument('--label', required=True)
    value.add_argument('--duration', type=float, default=15.0)
    value.add_argument('--confidence', type=float, default=0.55)
    value.add_argument('--minimum-hits', type=int, default=3)
    value.add_argument('--require-floor', action='store_true')
    return value


def main(argv=None):
    """Run until repeated evidence passes, fails, or times out."""
    arguments, ros_arguments = parser().parse_known_args(argv)
    try:
        label, duration, confidence, minimum_hits = validate_arguments(
            arguments.label,
            arguments.duration,
            arguments.confidence,
            arguments.minimum_hits,
        )
    except (AcceptanceError, TypeError, ValueError) as exc:
        parser().error(str(exc))
    session = ObjectAcceptanceSession(
        label,
        require_floor=arguments.require_floor,
        confidence=confidence,
        minimum_hits=minimum_hits,
    )

    rclpy.init(args=ros_arguments)
    node = ObjectAcceptanceNode(session)
    started = time.monotonic()
    interrupted = False
    try:
        while (
            rclpy.ok()
            and not session.passed
            and node.failure is None
            and time.monotonic() - started < duration
        ):
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        elapsed = time.monotonic() - started
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    report = session.report(
        duration=elapsed,
        failure=node.failure,
        interrupted=interrupted,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if interrupted:
        return 130
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
