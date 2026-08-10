from datetime import datetime, timezone

from dogzilla_slam.firmware_rest_capture import FirmwareRestRecorder
from dogzilla_slam.firmware_rest_monitor import monitor


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.01, float(seconds))


class ReadOnlyController:
    def __init__(self):
        self.calls = []
        self.battery_reads = 0
        self.joint_reads = 0

    def read_battery(self):
        self.calls.append('read_battery')
        self.battery_reads += 1
        return 30 if self.battery_reads == 1 else 25

    def read_motor(self):
        self.calls.append('read_motor')
        self.joint_reads += 1
        if self.joint_reads <= 5:
            angle = 0.0
        elif self.joint_reads <= 15:
            angle = float(self.joint_reads - 5)
        else:
            angle = 10.0
        return [angle] * 12


def test_monitor_uses_only_read_requests_and_never_enables_replay():
    clock = FakeClock()
    controller = ReadOnlyController()
    saved = []
    recorder = FirmwareRestRecorder(
        joint_names=tuple(f'joint_{index}' for index in range(12)),
        save_callback=lambda payload: saved.append(payload) or '/capture.json',
        clock=clock,
        wall_clock=lambda: datetime(
            2026,
            8,
            7,
            12,
            0,
            tzinfo=timezone.utc,
        ),
    )

    result = monitor(controller, recorder, clock=clock, sleep=clock.sleep)

    assert result == 0
    assert set(controller.calls) == {'read_battery', 'read_motor'}
    assert saved[0]['status'] == 'captured_unvalidated'
    assert saved[0]['replay_enabled'] is False
