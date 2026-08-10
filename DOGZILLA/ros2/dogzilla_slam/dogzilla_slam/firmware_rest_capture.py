"""Passively capture DOGZILLA's controller-driven low-battery rest motion."""

from collections import deque
from datetime import datetime, timezone
import json
import math
import os
import time


JOINT_COUNT = 12


def _validated_angles(value):
    if not isinstance(value, (list, tuple)) or len(value) != JOINT_COUNT:
        return None
    try:
        angles = tuple(float(angle) for angle in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(angle) for angle in angles):
        return None
    return angles


def save_capture_atomic(payload, output_directory):
    """Durably write one capture without exposing a partial JSON file."""
    os.makedirs(output_directory, exist_ok=True)
    stamp = payload['recorded_utc'].replace('-', '').replace(':', '')
    stamp = stamp.replace('+0000', 'Z')
    filename = f'firmware-rest-{stamp}.json'
    destination = os.path.join(output_directory, filename)
    temporary = destination + '.tmp'

    with open(temporary, 'w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, destination)

    directory_fd = os.open(output_directory, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return destination


class FirmwareRestRecorder:
    """State machine fed only by existing battery and joint readbacks."""

    def __init__(
        self,
        *,
        joint_names,
        save_callback,
        low_battery_percent=25,
        arm_margin_percent=5,
        pre_roll_seconds=2.0,
        stable_seconds=2.0,
        movement_delta_degrees=0.75,
        minimum_travel_degrees=5.0,
        maximum_capture_seconds=20.0,
        clock=time.monotonic,
        wall_clock=None,
    ):
        if len(joint_names) != JOINT_COUNT:
            raise ValueError('joint_names must contain exactly 12 entries')
        if not 1 <= low_battery_percent <= 95:
            raise ValueError('low_battery_percent must be between 1 and 95')
        if not 1 <= arm_margin_percent <= 20:
            raise ValueError('arm_margin_percent must be between 1 and 20')
        if not 0.5 <= pre_roll_seconds <= 10.0:
            raise ValueError('pre_roll_seconds must be between 0.5 and 10')
        if not 0.5 <= stable_seconds <= 10.0:
            raise ValueError('stable_seconds must be between 0.5 and 10')
        if not 5.0 <= maximum_capture_seconds <= 60.0:
            raise ValueError(
                'maximum_capture_seconds must be between 5 and 60'
            )

        self.joint_names = tuple(joint_names)
        self.save_callback = save_callback
        self.low_battery_percent = int(low_battery_percent)
        self.arm_battery_percent = int(
            low_battery_percent + arm_margin_percent
        )
        self.pre_roll_seconds = float(pre_roll_seconds)
        self.stable_seconds = float(stable_seconds)
        self.movement_delta_degrees = float(movement_delta_degrees)
        self.minimum_travel_degrees = float(minimum_travel_degrees)
        self.maximum_capture_seconds = float(maximum_capture_seconds)
        self.clock = clock
        self.wall_clock = wall_clock or (
            lambda: datetime.now(timezone.utc)
        )

        self._battery_percent = None
        self._seen_healthy_battery = False
        self._cycle_blocked = False
        self._capturing = False
        self._pre_roll = deque()
        self._samples = []
        self._trigger_time = None
        self._trigger_battery = None
        self._reference_angles = None
        self._previous_angles = None
        self._movement_started = False
        self._stable_since = None
        self._maximum_travel = 0.0
        self._events = []
        self.last_payload = None

    @property
    def capturing(self):
        return self._capturing

    @property
    def wants_high_joint_rate(self):
        near_low = (
            self._battery_percent is not None
            and self._battery_percent <= self.arm_battery_percent
        )
        return self._capturing or (
            near_low
            and self._seen_healthy_battery
            and not self._cycle_blocked
        )

    def take_events(self):
        events = list(self._events)
        self._events.clear()
        return events

    def observe_battery(self, battery_percent, now=None):
        """Arm and trigger capture from valid controller battery readback."""
        now = self.clock() if now is None else float(now)
        try:
            battery = int(battery_percent)
        except (TypeError, ValueError):
            return
        if not 1 <= battery <= 100:
            return

        previous = self._battery_percent
        self._battery_percent = battery

        reset_percent = self.arm_battery_percent + 3
        if battery > reset_percent:
            if self._cycle_blocked:
                self._events.append((
                    'info',
                    'Firmware-rest capture re-armed after battery recovery.',
                ))
            self._cycle_blocked = False
            self._seen_healthy_battery = True
            self._pre_roll.clear()
            return

        if battery > self.low_battery_percent:
            self._seen_healthy_battery = True

        if previous is None and battery <= self.low_battery_percent:
            self._cycle_blocked = True
            self._events.append((
                'warning',
                'Firmware-rest capture skipped: the manager started after '
                'the battery was already low, so the descent was not observed.',
            ))
            return

        crossed_low = (
            previous is not None
            and previous > self.low_battery_percent
            and battery <= self.low_battery_percent
        )
        if (
            crossed_low
            and self._seen_healthy_battery
            and not self._cycle_blocked
        ):
            self._start_capture(battery, now)

        self._check_timeout(now)

    def observe_joints(self, value, now=None):
        """Record one validated 12-joint readback without commanding motors."""
        now = self.clock() if now is None else float(now)
        angles = _validated_angles(value)
        if angles is None:
            self._check_timeout(now)
            return

        if not self._capturing:
            if self.wants_high_joint_rate:
                self._pre_roll.append((now, angles))
                cutoff = now - self.pre_roll_seconds
                while self._pre_roll and self._pre_roll[0][0] < cutoff:
                    self._pre_roll.popleft()
            return

        self._samples.append((now, angles))
        if self._previous_angles is None:
            self._reference_angles = angles
            self._previous_angles = angles
            self._check_timeout(now)
            return
        change = max(
            abs(current - previous)
            for current, previous in zip(angles, self._previous_angles)
        )
        travel = max(
            abs(current - reference)
            for current, reference in zip(angles, self._reference_angles)
        )
        self._maximum_travel = max(self._maximum_travel, travel)
        self._previous_angles = angles

        if change > self.movement_delta_degrees:
            self._movement_started = True
            self._stable_since = None
        elif self._movement_started:
            if self._stable_since is None:
                self._stable_since = now
            elif now - self._stable_since >= self.stable_seconds:
                if self._maximum_travel >= self.minimum_travel_degrees:
                    self._finish_capture(
                        'captured_unvalidated',
                        'Movement and stationary tail were observed.',
                        now,
                    )
                    return

        self._check_timeout(now)

    def _start_capture(self, battery, now):
        self._capturing = True
        self._cycle_blocked = True
        self._trigger_time = now
        self._trigger_battery = battery
        self._samples = list(self._pre_roll)
        self._pre_roll.clear()
        self._reference_angles = self._samples[0][1] if self._samples else None
        self._previous_angles = self._reference_angles
        self._movement_started = False
        self._stable_since = None
        self._maximum_travel = 0.0

        # The controller can begin its descent between one-second battery
        # reads. Analyse the pre-roll so motion observed just before the exact
        # threshold sample is not lost or misclassified.
        for sample_time, angles in self._samples[1:]:
            change = max(
                abs(current - previous)
                for current, previous in zip(angles, self._previous_angles)
            )
            travel = max(
                abs(current - reference)
                for current, reference in zip(angles, self._reference_angles)
            )
            self._maximum_travel = max(self._maximum_travel, travel)
            self._previous_angles = angles
            if change > self.movement_delta_degrees:
                self._movement_started = True
                self._stable_since = None
            elif self._movement_started and self._stable_since is None:
                self._stable_since = sample_time

        self._events.append((
            'warning',
            f'Low-battery threshold crossed at {battery}%; passively '
            'capturing the controller-driven joint trajectory.',
        ))

        if (
            self._movement_started
            and self._stable_since is not None
            and now - self._stable_since >= self.stable_seconds
            and self._maximum_travel >= self.minimum_travel_degrees
        ):
            self._finish_capture(
                'captured_unvalidated',
                'Movement and stationary tail were observed in the pre-roll.',
                now,
            )

    def _check_timeout(self, now):
        if not self._capturing:
            return
        if now - self._trigger_time < self.maximum_capture_seconds:
            return
        if not self._movement_started:
            reason = 'No joint motion was observed after the battery trigger.'
        else:
            reason = 'The captured motion did not reach a stable tail in time.'
        self._finish_capture('incomplete', reason, now)

    def _finish_capture(self, status, reason, now):
        recorded = self.wall_clock()
        if recorded.tzinfo is None:
            recorded = recorded.replace(tzinfo=timezone.utc)
        samples = [
            {
                't_seconds': round(sample_time - self._trigger_time, 6),
                'angles_degrees': [round(angle, 2) for angle in angles],
            }
            for sample_time, angles in self._samples
        ]
        payload = {
            'schema_version': 1,
            'kind': 'dogzilla_controller_firmware_rest_capture',
            'source': 'controller_low_battery_transition',
            'status': status,
            'replay_enabled': False,
            'recorded_utc': recorded.astimezone(timezone.utc).isoformat(),
            'joint_names': list(self.joint_names),
            'trigger_battery_percent': self._trigger_battery,
            'low_battery_threshold_percent': self.low_battery_percent,
            'pre_roll_seconds': self.pre_roll_seconds,
            'stationary_tail_required_seconds': self.stable_seconds,
            'maximum_joint_travel_degrees': round(
                self._maximum_travel,
                2,
            ),
            'movement_observed': self._movement_started,
            'reason': reason,
            'capture_elapsed_seconds': round(
                now - self._trigger_time,
                6,
            ),
            'samples': samples,
            'final_angles_degrees': (
                list(samples[-1]['angles_degrees']) if samples else None
            ),
        }
        self.last_payload = payload
        self._capturing = False
        self._stable_since = None

        try:
            destination = self.save_callback(payload)
        except Exception as exc:  # Recording must never stop the serial manager.
            self._events.append((
                'error',
                f'Firmware-rest capture could not be saved: {exc}',
            ))
            return

        if status == 'captured_unvalidated':
            self._events.append((
                'warning',
                'Firmware-rest trajectory captured but NOT approved for '
                f'replay: {destination}',
            ))
        else:
            self._events.append((
                'error',
                f'Incomplete firmware-rest diagnostic saved: {destination}',
            ))
