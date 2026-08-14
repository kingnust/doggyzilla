"""Pure validation and debounce policy for explicitly armed vision control."""

import math


FIRMWARE_ACTION_NAMES = {
    1: 'lie-down',
    2: 'stand-up',
    3: 'crawl',
    4: 'turn-around',
    5: 'mark-time',
    6: 'squat',
    7: 'turn-roll',
    8: 'turn-pitch',
    9: 'turn-yaw',
    10: 'three-axis',
    11: 'pee',
    12: 'sit-down',
    13: 'wave-hand',
    14: 'stretch',
    15: 'wave-body',
    16: 'swing',
    17: 'pray',
    18: 'seek',
    19: 'handshake',
}

COLOR_ACTIONS = {
    'red': (14, 'stretch'),
    'green': (13, 'wave-hand'),
    'blue': (16, 'swing'),
    'yellow': (18, 'seek'),
}

QR_ACTIONS = {
    'LIE DOWN': (1, 'lie-down'),
    'STAND UP': (2, 'stand-up'),
    'CRAWL': (3, 'crawl'),
    'TURN AROUND': (4, 'turn-around'),
    'MARK TIME': (5, 'mark-time'),
    'SQUAT': (6, 'squat'),
    'TURN ROLL': (7, 'turn-roll'),
    'TURN PITCH': (8, 'turn-pitch'),
    'TURN YAW': (9, 'turn-yaw'),
    '3 AXIS': (10, 'three-axis'),
    'PEE': (11, 'pee'),
    'SIT DOWN': (12, 'sit-down'),
    'WAVE(HAND)': (13, 'wave-hand'),
    'STRETCH': (14, 'stretch'),
    'WAVE(BODY)': (15, 'wave-body'),
    'SWING': (16, 'swing'),
    'PRAY': (17, 'pray'),
    'SEEK': (18, 'seek'),
    'HANDSHAKE': (19, 'handshake'),
}

FIRMWARE_SOURCES = {
    'yahboom-lesson-8.3',
    'yahboom-lesson-8.6',
    'yahboom-lesson-8.8',
}
LINE_SOURCE = 'yahboom-lessons-8.11-8.12'


class VisionActionPolicyError(ValueError):
    """Raised when a proposal violates the fixed control contract."""


class VisionExecutionSafetyError(RuntimeError):
    """Raised before a firmware command when execution is not safe."""


def _finite_number(value, label):
    if isinstance(value, bool):
        raise VisionActionPolicyError(f'{label} must be a finite number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise VisionActionPolicyError(
            f'{label} must be a finite number'
        ) from exc
    if not math.isfinite(number):
        raise VisionActionPolicyError(f'{label} must be a finite number')
    return number


def validate_action_proposal(proposal):
    """Return one normalized proposal from the fixed Yahboom allowlist."""
    if not isinstance(proposal, dict):
        raise VisionActionPolicyError('action proposal must be an object')
    if proposal.get('requires_explicit_arming') is not True:
        raise VisionActionPolicyError(
            'action proposal must require explicit arming'
        )
    if proposal.get('executed') is not False:
        raise VisionActionPolicyError(
            'vision producer must not mark a proposal as executed'
        )

    kind = proposal.get('kind')
    source = str(proposal.get('source', ''))
    name = str(proposal.get('name', ''))
    if kind == 'firmware-action':
        if source not in FIRMWARE_SOURCES:
            raise VisionActionPolicyError(
                'firmware proposal source is blocked'
            )
        action_id = proposal.get('action_id')
        if isinstance(action_id, bool) or not isinstance(action_id, int):
            raise VisionActionPolicyError(
                'firmware action_id must be an integer'
            )
        if FIRMWARE_ACTION_NAMES.get(action_id) != name:
            raise VisionActionPolicyError(
                'firmware action_id and name do not match the allowlist'
            )
        if source == 'yahboom-lesson-8.3':
            if (action_id, name) not in COLOR_ACTIONS.values():
                raise VisionActionPolicyError(
                    'color action is outside Yahboom lesson 8.3'
                )
        elif source == 'yahboom-lesson-8.6':
            if (action_id, name) != (19, 'handshake'):
                raise VisionActionPolicyError(
                    'Watchdog can only propose handshake action 19'
                )
        else:
            matched_text = str(proposal.get('matched_text', ''))
            if QR_ACTIONS.get(matched_text) != (action_id, name):
                raise VisionActionPolicyError(
                    'QR action must match one exact Yahboom label'
                )
        return {
            'kind': kind,
            'source': source,
            'action_id': action_id,
            'name': name,
        }

    if kind == 'velocity-intent':
        if source != LINE_SOURCE or name != 'line-follow':
            raise VisionActionPolicyError(
                'velocity proposal source is blocked'
            )
        steering_error = _finite_number(
            proposal.get('steering_error'),
            'steering_error',
        )
        if not -1.0 <= steering_error <= 1.0:
            raise VisionActionPolicyError(
                'steering_error must be between -1 and 1'
            )
        if proposal.get('reference_forward_command') != 25:
            raise VisionActionPolicyError(
                'line-follow reference command does not match Yahboom'
            )
        return {
            'kind': kind,
            'source': source,
            'name': name,
            'steering_error': steering_error,
        }

    raise VisionActionPolicyError('unsupported action proposal kind')


def execute_firmware_action(
    controller,
    decision,
    battery_percent,
    low_battery_percent=25,
    movement_inhibited=False,
):
    """Stop, validate, then issue one allowlisted firmware action."""
    if not isinstance(decision, dict):
        raise VisionExecutionSafetyError('firmware decision must be an object')
    action_id = decision.get('action_id')
    name = decision.get('name')
    if (
        decision.get('kind') != 'firmware-action'
        or isinstance(action_id, bool)
        or not isinstance(action_id, int)
        or FIRMWARE_ACTION_NAMES.get(action_id) != name
    ):
        raise VisionExecutionSafetyError(
            'firmware decision is outside the fixed allowlist'
        )
    if movement_inhibited:
        raise VisionExecutionSafetyError(
            'movement is inhibited by the battery safety state'
        )
    try:
        battery = int(battery_percent)
    except (TypeError, ValueError) as exc:
        raise VisionExecutionSafetyError(
            'valid battery telemetry is required'
        ) from exc
    if not 1 <= battery <= 100:
        raise VisionExecutionSafetyError(
            'valid battery telemetry is required'
        )
    if battery <= int(low_battery_percent):
        raise VisionExecutionSafetyError(
            f'battery {battery}% is at or below the '
            f'{int(low_battery_percent)}% safety threshold'
        )
    controller.stop()
    controller.action(action_id)


def line_follow_velocity(decision, forward_speed=0.08, maximum_turn=0.25):
    """Convert normalized Yahboom line error into bounded ROS velocity."""
    if not isinstance(decision, dict) or decision.get('kind') != (
        'velocity-intent'
    ):
        raise VisionExecutionSafetyError('line decision is invalid')
    steering_error = _finite_number(
        decision.get('steering_error'),
        'steering_error',
    )
    if not -1.0 <= steering_error <= 1.0:
        raise VisionExecutionSafetyError(
            'steering_error must be between -1 and 1'
        )
    forward = _finite_number(forward_speed, 'forward_speed')
    turn_limit = _finite_number(maximum_turn, 'maximum_turn')
    if not 0.01 <= forward <= 0.10:
        raise VisionExecutionSafetyError(
            'forward_speed must be between 0.01 and 0.10 m/s'
        )
    if not 0.05 <= turn_limit <= 0.30:
        raise VisionExecutionSafetyError(
            'maximum_turn must be between 0.05 and 0.30 rad/s'
        )
    # Yahboom's PID uses setpoint - measured horizontal error, so a target to
    # the right produces a negative turn command.
    return forward, -steering_error * turn_limit


class VisionControlPolicy:
    """Require stable observations and release before repeated actions."""

    def __init__(
        self,
        required_frames=5,
        release_frames=3,
        action_cooldown_seconds=8.0,
    ):
        if not 2 <= int(required_frames) <= 30:
            raise ValueError('required_frames must be between 2 and 30')
        if not 1 <= int(release_frames) <= 30:
            raise ValueError('release_frames must be between 1 and 30')
        cooldown = _finite_number(
            action_cooldown_seconds,
            'action_cooldown_seconds',
        )
        if not 1.0 <= cooldown <= 120.0:
            raise ValueError(
                'action_cooldown_seconds must be between 1 and 120'
            )
        self.required_frames = int(required_frames)
        self.release_frames = int(release_frames)
        self.action_cooldown_seconds = cooldown
        self._signature = None
        self._stable_frames = 0
        self._empty_frames = 0
        self._fired_signature = None
        self._last_action_time = -math.inf

    @staticmethod
    def _proposal_signature(proposal):
        if proposal['kind'] == 'firmware-action':
            return (
                proposal['kind'],
                proposal['source'],
                proposal['action_id'],
                proposal['name'],
            )
        return (
            proposal['kind'],
            proposal['source'],
            proposal['name'],
        )

    def reset(self):
        """Clear target history and require a new stable observation."""
        self._signature = None
        self._stable_frames = 0
        self._empty_frames = 0
        self._fired_signature = None

    def observe(self, result, now):
        """Return one allowed decision only after all temporal gates pass."""
        current_time = _finite_number(now, 'now')
        if not isinstance(result, dict):
            raise VisionActionPolicyError('vision result must be an object')
        if result.get('schema_version') != 1:
            raise VisionActionPolicyError('unsupported vision result schema')
        if result.get('action_output') != 'disabled':
            raise VisionActionPolicyError(
                'vision producer action output must remain disabled'
            )
        proposals = result.get('action_proposals')
        if not isinstance(proposals, list):
            raise VisionActionPolicyError('action_proposals must be a list')
        if not proposals:
            self._empty_frames += 1
            self._signature = None
            self._stable_frames = 0
            if self._empty_frames >= self.release_frames:
                self._fired_signature = None
            return None

        self._empty_frames = 0
        proposal = validate_action_proposal(proposals[0])
        signature = self._proposal_signature(proposal)
        if signature != self._signature:
            self._signature = signature
            self._stable_frames = 1
        else:
            self._stable_frames += 1
        if self._stable_frames < self.required_frames:
            return None

        if proposal['kind'] == 'velocity-intent':
            return proposal
        if signature == self._fired_signature:
            return None
        if (
            current_time - self._last_action_time
            < self.action_cooldown_seconds
        ):
            return None
        self._fired_signature = signature
        self._last_action_time = current_time
        return proposal
