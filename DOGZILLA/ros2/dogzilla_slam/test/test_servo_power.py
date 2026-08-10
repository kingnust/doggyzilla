import pytest

from dogzilla_slam.servo_power import apply_servo_mode
from dogzilla_slam.servo_power import main
from dogzilla_slam.servo_power import ServoPowerSafetyError


class FakeController:
    def __init__(self, battery=80, motor_angles=None):
        self.calls = []
        self.battery = battery
        self.motor_angles = motor_angles or [0.0] * 12

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

    def read_motor(self):
        self.calls.append('read_motor')
        return self.motor_angles


class FailingBatteryController(FakeController):
    def read_battery(self):
        self.calls.append('read_battery')
        raise OSError('simulated serial failure')


def test_rest_is_blocked_without_sending_any_servo_command():
    controller = FakeController()

    with pytest.raises(ServoPowerSafetyError, match='REST is disabled'):
        apply_servo_mode(
            'rest',
            controller,
            sleep=lambda _: None,
        )

    assert controller.calls == []


def test_rest_cli_returns_before_opening_the_serial_port(monkeypatch):
    def unexpected_controller(*args, **kwargs):
        raise AssertionError('REST must not open the controller serial port')

    monkeypatch.setattr('dogzilla_slam.servo_power.dog.DOGZILLA', unexpected_controller)

    assert main(['rest']) == 2


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
    assert delays == [0.20, 0.50, 4.00]


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
