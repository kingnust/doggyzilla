import pytest

from dogzilla_slam.servo_power import apply_servo_mode
from dogzilla_slam.servo_power import ServoPowerSafetyError


class FakeController:
    def __init__(self, battery=80):
        self.calls = []
        self.battery = battery

    def read_battery(self):
        self.calls.append('read_battery')
        return self.battery

    def stop(self):
        self.calls.append('stop')

    def unload_allmotor(self):
        self.calls.append('unload_allmotor')

    def load_allmotor(self):
        self.calls.append('load_allmotor')

    def action(self, action_id):
        self.calls.append(('action', action_id))


class FailingBatteryController(FakeController):
    def read_battery(self):
        self.calls.append('read_battery')
        raise OSError('simulated serial failure')


def test_rest_stops_then_runs_vendor_lie_down_animation():
    controller = FakeController()
    delays = []

    apply_servo_mode('rest', controller, sleep=delays.append)

    assert controller.calls == ['stop', ('action', 1), 'unload_allmotor']
    assert delays == [0.20, 3.00, 0.20]


def test_stand_stops_loads_then_runs_vendor_stand_up_animation():
    controller = FakeController()
    delays = []

    apply_servo_mode('stand', controller, sleep=delays.append)

    assert controller.calls == [
        'read_battery',
        'stop',
        'load_allmotor',
        ('action', 2),
    ]
    assert delays == [0.20, 0.25, 3.00]


@pytest.mark.parametrize('battery', [1, 24, 25])
def test_stand_never_overrides_low_battery_rest(battery):
    controller = FakeController(battery=battery)

    with pytest.raises(ServoPowerSafetyError, match='charge first'):
        apply_servo_mode('stand', controller, sleep=lambda _: None)

    assert controller.calls == ['read_battery']


@pytest.mark.parametrize('battery', [0, None, 'not-a-number'])
def test_stand_is_blocked_when_battery_read_is_invalid(battery):
    controller = FakeController(battery=battery)

    with pytest.raises(ServoPowerSafetyError, match='reading failed'):
        apply_servo_mode('stand', controller, sleep=lambda _: None)

    assert controller.calls == ['read_battery']


def test_stand_is_blocked_when_battery_serial_read_raises():
    controller = FailingBatteryController()

    with pytest.raises(ServoPowerSafetyError, match='reading failed'):
        apply_servo_mode('stand', controller, sleep=lambda _: None)

    assert controller.calls == ['read_battery']


def test_stand_is_allowed_above_threshold():
    controller = FakeController(battery=26)

    battery = apply_servo_mode('stand', controller, sleep=lambda _: None)

    assert battery == 26
    assert controller.calls == [
        'read_battery',
        'stop',
        'load_allmotor',
        ('action', 2),
    ]
