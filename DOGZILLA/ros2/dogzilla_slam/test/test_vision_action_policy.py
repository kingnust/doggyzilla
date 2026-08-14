import pytest

from dogzilla_slam.vision_action_policy import VisionActionPolicyError
from dogzilla_slam.vision_action_policy import VisionControlPolicy
from dogzilla_slam.vision_action_policy import execute_firmware_action
from dogzilla_slam.vision_action_policy import line_follow_velocity
from dogzilla_slam.vision_action_policy import VisionExecutionSafetyError


def result_with(proposal=None):
    return {
        'schema_version': 1,
        'action_output': 'disabled',
        'action_proposals': [] if proposal is None else [proposal],
    }


def firmware_proposal(
    action_id=14,
    name='stretch',
    source='yahboom-lesson-8.3',
    **extra,
):
    return {
        'kind': 'firmware-action',
        'source': source,
        'action_id': action_id,
        'name': name,
        'requires_explicit_arming': True,
        'executed': False,
        **extra,
    }


def line_proposal(error=0.25):
    return {
        'kind': 'velocity-intent',
        'source': 'yahboom-lessons-8.11-8.12',
        'name': 'line-follow',
        'steering_error': error,
        'reference_forward_command': 25,
        'requires_explicit_arming': True,
        'executed': False,
    }


def test_firmware_action_requires_stable_frames_and_fires_once():
    policy = VisionControlPolicy(required_frames=3, release_frames=2)
    value = result_with(firmware_proposal())

    assert policy.observe(value, 10.0) is None
    assert policy.observe(value, 10.1) is None
    decision = policy.observe(value, 10.2)
    assert decision['action_id'] == 14
    assert policy.observe(value, 30.0) is None


def test_target_must_disappear_before_same_action_can_rearm():
    policy = VisionControlPolicy(
        required_frames=2,
        release_frames=2,
        action_cooldown_seconds=5.0,
    )
    value = result_with(firmware_proposal())

    assert policy.observe(value, 0.0) is None
    assert policy.observe(value, 0.1)['action_id'] == 14
    assert policy.observe(result_with(), 6.0) is None
    assert policy.observe(value, 6.1) is None
    assert policy.observe(value, 6.2) is None
    assert policy.observe(result_with(), 6.3) is None
    assert policy.observe(result_with(), 6.4) is None
    assert policy.observe(value, 6.5) is None
    assert policy.observe(value, 6.6)['action_id'] == 14


def test_global_cooldown_blocks_a_different_immediate_action():
    policy = VisionControlPolicy(
        required_frames=2,
        action_cooldown_seconds=8.0,
    )
    stretch = result_with(firmware_proposal())
    wave = result_with(firmware_proposal(13, 'wave-hand'))

    policy.observe(stretch, 1.0)
    assert policy.observe(stretch, 1.1)['action_id'] == 14
    policy.observe(wave, 2.0)
    assert policy.observe(wave, 2.1) is None
    assert policy.observe(wave, 9.2)['action_id'] == 13


def test_qr_action_requires_exact_text_id_and_name_mapping():
    policy = VisionControlPolicy(required_frames=2)
    allowed = firmware_proposal(
        2,
        'stand-up',
        source='yahboom-lesson-8.8',
        matched_text='STAND UP',
    )
    mismatched = {**allowed, 'matched_text': 'LIE DOWN'}

    assert policy.observe(result_with(allowed), 1.0) is None
    assert policy.observe(result_with(allowed), 1.1)['action_id'] == 2
    policy.reset()
    with pytest.raises(VisionActionPolicyError, match='exact Yahboom'):
        policy.observe(result_with(mismatched), 20.0)


@pytest.mark.parametrize(
    'change',
    [
        {'executed': True},
        {'requires_explicit_arming': False},
        {'action_id': 99},
        {'name': 'not-stretch'},
        {'source': 'untrusted'},
    ],
)
def test_malformed_or_untrusted_firmware_proposals_are_rejected(change):
    proposal = {**firmware_proposal(), **change}
    policy = VisionControlPolicy(required_frames=2)

    with pytest.raises(VisionActionPolicyError):
        policy.observe(result_with(proposal), 1.0)


def test_line_follow_requires_stability_and_bounds_steering():
    policy = VisionControlPolicy(required_frames=3)
    value = result_with(line_proposal(0.4))

    assert policy.observe(value, 1.0) is None
    assert policy.observe(value, 1.1) is None
    decision = policy.observe(value, 1.2)
    assert decision == {
        'kind': 'velocity-intent',
        'source': 'yahboom-lessons-8.11-8.12',
        'name': 'line-follow',
        'steering_error': 0.4,
    }

    with pytest.raises(VisionActionPolicyError, match='between -1 and 1'):
        VisionControlPolicy(required_frames=2).observe(
            result_with(line_proposal(1.1)),
            2.0,
        )


class FakeController:
    def __init__(self):
        self.calls = []

    def stop(self):
        self.calls.append('stop')

    def action(self, action_id):
        self.calls.append(('action', action_id))


def test_firmware_executor_stops_then_sends_one_allowlisted_action():
    controller = FakeController()
    execute_firmware_action(
        controller,
        {
            'kind': 'firmware-action',
            'action_id': 19,
            'name': 'handshake',
        },
        battery_percent=80,
    )

    assert controller.calls == ['stop', ('action', 19)]


@pytest.mark.parametrize('battery', [None, 0, 25])
def test_firmware_executor_sends_nothing_without_safe_battery(battery):
    controller = FakeController()

    with pytest.raises(VisionExecutionSafetyError):
        execute_firmware_action(
            controller,
            {
                'kind': 'firmware-action',
                'action_id': 19,
                'name': 'handshake',
            },
            battery_percent=battery,
        )

    assert controller.calls == []


def test_line_velocity_is_slow_bounded_and_uses_yahboom_turn_sign():
    forward, turn = line_follow_velocity(
        {'kind': 'velocity-intent', 'steering_error': 0.5},
    )

    assert forward == pytest.approx(0.08)
    assert turn == pytest.approx(-0.125)
