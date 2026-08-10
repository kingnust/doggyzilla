from datetime import datetime, timezone
import json

from dogzilla_slam.firmware_rest_capture import FirmwareRestRecorder
from dogzilla_slam.firmware_rest_capture import save_capture_atomic


JOINT_NAMES = tuple(f'joint_{index}' for index in range(12))


def make_recorder(saved, **overrides):
    options = {
        'joint_names': JOINT_NAMES,
        'save_callback': lambda payload: saved.append(payload) or '/capture.json',
        'low_battery_percent': 25,
        'arm_margin_percent': 5,
        'pre_roll_seconds': 2.0,
        'stable_seconds': 2.0,
        'maximum_capture_seconds': 20.0,
        'wall_clock': lambda: datetime(
            2026,
            8,
            7,
            12,
            0,
            tzinfo=timezone.utc,
        ),
    }
    options.update(overrides)
    return FirmwareRestRecorder(**options)


def test_starting_already_low_does_not_create_a_false_capture():
    saved = []
    recorder = make_recorder(saved)

    recorder.observe_battery(24, now=0.0)
    for index in range(30):
        recorder.observe_joints([0.0] * 12, now=index * 0.2)

    assert saved == []
    assert not recorder.capturing
    assert not recorder.wants_high_joint_rate
    assert any('already low' in message for _, message in recorder.take_events())


def test_low_battery_descent_is_captured_with_pre_roll_and_never_enabled():
    saved = []
    recorder = make_recorder(saved)

    recorder.observe_battery(80, now=0.0)
    recorder.observe_battery(30, now=1.0)
    assert recorder.wants_high_joint_rate
    recorder.observe_joints([0.0] * 12, now=1.0)
    recorder.observe_joints([0.0] * 12, now=1.8)

    recorder.observe_battery(25, now=2.0)
    assert recorder.capturing
    for step in range(1, 11):
        recorder.observe_joints([float(step)] * 12, now=2.0 + step * 0.2)
    for step in range(1, 12):
        recorder.observe_joints([10.0] * 12, now=4.0 + step * 0.2)

    assert len(saved) == 1
    payload = saved[0]
    assert payload['status'] == 'captured_unvalidated'
    assert payload['replay_enabled'] is False
    assert payload['movement_observed'] is True
    assert payload['maximum_joint_travel_degrees'] == 10.0
    assert payload['samples'][0]['t_seconds'] < 0.0
    assert payload['final_angles_degrees'] == [10.0] * 12
    assert not recorder.capturing
    assert not recorder.wants_high_joint_rate


def test_missing_motion_is_saved_only_as_incomplete_diagnostic():
    saved = []
    recorder = make_recorder(saved, maximum_capture_seconds=5.0)

    recorder.observe_battery(80, now=0.0)
    recorder.observe_battery(29, now=1.0)
    recorder.observe_joints([0.0] * 12, now=1.0)
    recorder.observe_battery(25, now=2.0)
    recorder.observe_joints([0.0] * 12, now=7.0)

    assert len(saved) == 1
    assert saved[0]['status'] == 'incomplete'
    assert saved[0]['replay_enabled'] is False
    assert saved[0]['movement_observed'] is False


def test_descent_that_begins_before_threshold_is_detected_in_pre_roll():
    saved = []
    recorder = make_recorder(saved)

    recorder.observe_battery(80, now=-1.0)
    recorder.observe_battery(30, now=0.0)
    recorder.observe_joints([0.0] * 12, now=0.0)
    recorder.observe_joints([5.0] * 12, now=0.2)
    recorder.observe_joints([10.0] * 12, now=0.4)
    for step in range(1, 9):
        recorder.observe_joints([10.0] * 12, now=0.4 + step * 0.2)

    recorder.observe_battery(25, now=2.0)
    recorder.observe_joints([10.0] * 12, now=2.2)
    recorder.observe_joints([10.0] * 12, now=2.4)
    recorder.observe_joints([10.0] * 12, now=2.6)

    assert len(saved) == 1
    assert saved[0]['status'] == 'captured_unvalidated'
    assert saved[0]['movement_observed'] is True
    assert saved[0]['maximum_joint_travel_degrees'] == 10.0


def test_invalid_joint_telemetry_never_becomes_a_profile():
    saved = []
    recorder = make_recorder(saved, maximum_capture_seconds=5.0)

    recorder.observe_battery(80, now=0.0)
    recorder.observe_battery(25, now=1.0)
    recorder.observe_joints([], now=6.0)

    assert len(saved) == 1
    assert saved[0]['status'] == 'incomplete'
    assert saved[0]['samples'] == []


def test_atomic_capture_writer_produces_complete_json(tmp_path):
    payload = {
        'recorded_utc': '2026-08-07T12:00:00.123456+00:00',
        'status': 'captured_unvalidated',
        'replay_enabled': False,
    }

    path = save_capture_atomic(payload, str(tmp_path))

    with open(path, encoding='utf-8') as stream:
        assert json.load(stream) == payload
    assert not list(tmp_path.glob('*.tmp'))
