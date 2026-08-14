import json

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

from dogzilla_slam import safe_base


class FakeSerial:
    def __init__(self):
        self.is_open = True

    def close(self):
        self.is_open = False


class FakeController:
    def __init__(self, battery=80):
        self.battery = battery
        self.calls = []
        self.ser = FakeSerial()

    def _DOGZILLA__unpack(self, timeout=1.0):
        return []

    def stop(self):
        self.calls.append('stop')

    def pace(self, value):
        self.calls.append(('pace', value))

    def read_battery(self):
        self.calls.append('read_battery')
        return self.battery

    def read_motor(self):
        self.calls.append('read_motor')
        return [0.0] * 12

    def read_imu_raw(self):
        self.calls.append('read_imu_raw')
        return [0.0] * 6

    def move(self, axis, value):
        self.calls.append(('move', axis, value))

    def turn(self, value):
        self.calls.append(('turn', value))

    def action(self, action_id):
        self.calls.append(('action', action_id))


def detection_message(proposal=None):
    value = {
        'schema_version': 1,
        'action_output': 'disabled',
        'action_proposals': [] if proposal is None else [proposal],
    }
    return String(data=json.dumps(value))


def firmware_proposal():
    return {
        'kind': 'firmware-action',
        'source': 'yahboom-lesson-8.6',
        'action_id': 19,
        'name': 'handshake',
        'requires_explicit_arming': True,
        'executed': False,
    }


def line_proposal(error=0.5):
    return {
        'kind': 'velocity-intent',
        'source': 'yahboom-lessons-8.11-8.12',
        'name': 'line-follow',
        'steering_error': error,
        'reference_forward_command': 25,
        'requires_explicit_arming': True,
        'executed': False,
    }


@pytest.fixture
def base_factory(monkeypatch):
    nodes = []

    def create(controller):
        monkeypatch.setattr(
            safe_base.dog,
            'DOGZILLA',
            lambda: controller,
        )
        rclpy.init()
        node = safe_base.SafeBase(
            parameter_overrides=[
                Parameter('vision_control_enabled', value=True),
                Parameter('accept_velocity_commands', value=False),
                Parameter('vision_required_frames', value=2),
                Parameter('vision_release_frames', value=2),
                Parameter('capture_firmware_rest', value=False),
                Parameter('publish_joint_states', value=False),
                Parameter('battery_rate_hz', value=0.1),
            ],
        )
        nodes.append(node)
        controller.calls.clear()
        return node

    yield create

    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_real_safe_base_callback_executes_after_debounce(base_factory):
    controller = FakeController(battery=80)
    node = base_factory(controller)
    message = detection_message(firmware_proposal())

    node._on_vision_detections(message)
    assert ('action', 19) not in controller.calls
    node._on_vision_detections(message)

    assert controller.calls == ['stop', ('action', 19)]
    assert node._vision_active_action == 'handshake'
    assert node._vision_action_deadline is not None
    assert node._subscription is None


def test_real_safe_base_callback_blocks_low_battery(base_factory):
    controller = FakeController(battery=25)
    node = base_factory(controller)
    controller.calls.clear()
    message = detection_message(firmware_proposal())

    node._on_vision_detections(message)
    node._on_vision_detections(message)

    assert ('action', 19) not in controller.calls
    assert node._movement_inhibited is True


def test_real_safe_base_line_follow_is_bounded_and_stops_on_loss(base_factory):
    controller = FakeController(battery=80)
    node = base_factory(controller)
    message = detection_message(line_proposal(0.5))

    node._on_vision_detections(message)
    node._on_vision_detections(message)

    assert ('move', 'x', 3.2) in controller.calls
    assert ('move', 'y', 0.0) in controller.calls
    assert ('turn', -5.0) in controller.calls
    assert node._vision_line_active is True

    node._on_vision_detections(detection_message())

    assert controller.calls[-1] == 'stop'
    assert node._vision_line_active is False


def test_emergency_stop_cancels_action_and_blocks_new_proposals(base_factory):
    controller = FakeController(battery=80)
    node = base_factory(controller)
    message = detection_message(firmware_proposal())
    node._on_vision_detections(message)
    node._on_vision_detections(message)
    controller.calls.clear()

    node._on_estop(Bool(data=True))
    node._on_vision_detections(message)
    node._on_vision_detections(message)

    assert controller.calls
    assert controller.calls[0] == 'stop'
    assert ('action', 19) not in controller.calls
    assert node._estop_latched is True

    node._on_estop(Bool(data=False))
    assert node._estop_latched is False
