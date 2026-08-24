import pytest

from dogzilla_slam.speed_control import AUTONOMY_ANGULAR_LIMITS
from dogzilla_slam.speed_control import AUTONOMY_LINEAR_LIMITS
from dogzilla_slam.speed_control import normalize_speed_level
from dogzilla_slam.speed_control import SPEED_LEVELS
from dogzilla_slam.speed_control import TURN_LEVELS
from dogzilla_slam.teleop import next_posture
from dogzilla_slam.teleop import next_turn_level
from dogzilla_slam.teleop import TURN_KEYS


@pytest.mark.parametrize(
    'key, expected',
    [
        ('r', (110.0, 0.0, 0.0)),
        ('f', (100.0, 0.0, 0.0)),
        ('i', (105.0, -5.0, 0.0)),
        (',', (105.0, 5.0, 0.0)),
        ('j', (105.0, 0.0, 5.0)),
        ('l', (105.0, 0.0, -5.0)),
        ('c', (105.0, 0.0, 0.0)),
    ],
)
def test_posture_keys(key, expected):
    assert next_posture(key, 105.0, 0.0, 0.0) == expected


def test_posture_limits_are_hard_clamped():
    assert next_posture('r', 110.0, 0.0, 0.0)[0] == 110.0
    assert next_posture('f', 75.0, 0.0, 0.0)[0] == 75.0
    assert next_posture('i', 105.0, -15.0, 0.0)[1] == -15.0
    assert next_posture(',', 105.0, 15.0, 0.0)[1] == 15.0
    assert next_posture('j', 105.0, 0.0, 11.0)[2] == 11.0
    assert next_posture('l', 105.0, 0.0, -11.0)[2] == -11.0


def test_unknown_posture_key_is_rejected():
    with pytest.raises(ValueError):
        next_posture('?', 105.0, 0.0, 0.0)


def test_turn_level_steps_and_clamps_between_one_and_nine():
    assert next_turn_level(5, 1) == 6
    assert next_turn_level(5, -1) == 4
    assert next_turn_level(9, 1) == 9
    assert next_turn_level(1, -1) == 1


def test_minus_equals_and_plus_have_explicit_turn_bindings():
    assert TURN_KEYS == {'-': -1, '=': 1, '+': 1}


def test_turn_level_rejects_non_unit_direction():
    with pytest.raises(ValueError, match='direction'):
        next_turn_level(5, 2)


def test_speed_levels_preserve_slow_normal_and_maximum_anchors():
    assert [
        SPEED_LEVELS[level].controller_step for level in range(1, 10)
    ] == [4, 6, 7, 9, 10, 13, 15, 18, 20]
    assert SPEED_LEVELS[1].max_linear == pytest.approx(0.10)
    assert SPEED_LEVELS[1].max_angular == pytest.approx(0.30)
    assert SPEED_LEVELS[5].max_linear == pytest.approx(0.25)
    assert SPEED_LEVELS[5].max_angular == pytest.approx(1.125)
    assert SPEED_LEVELS[9].max_linear == pytest.approx(0.50)
    assert SPEED_LEVELS[9].max_angular == pytest.approx(1.75)


def test_speed_levels_are_monotonic_and_numeric():
    linear = [SPEED_LEVELS[level].max_linear for level in range(1, 10)]
    angular = [SPEED_LEVELS[level].max_angular for level in range(1, 10)]
    assert linear == sorted(linear)
    assert angular == sorted(angular)
    assert len(set(linear)) == 9
    assert len(set(angular)) == 9
    assert normalize_speed_level('7') == 7


def test_turn_levels_are_distinct_and_firmware_valid():
    steps = [TURN_LEVELS[level].controller_step for level in range(1, 10)]
    angular = [TURN_LEVELS[level].max_angular for level in range(1, 10)]
    assert steps == [30, 34, 38, 41, 45, 51, 58, 64, 70]
    assert angular == sorted(angular)
    assert len(set(angular)) == 9
    assert TURN_LEVELS[5].max_angular == pytest.approx(1.125)
    assert TURN_LEVELS[9].max_angular == pytest.approx(1.75)


def test_autonomy_level_four_is_brisk_and_all_levels_are_distinct():
    linear = [AUTONOMY_LINEAR_LIMITS[level] for level in range(1, 10)]
    angular = [AUTONOMY_ANGULAR_LIMITS[level] for level in range(1, 10)]

    assert AUTONOMY_LINEAR_LIMITS[4] == pytest.approx(0.20)
    assert AUTONOMY_ANGULAR_LIMITS[4] == pytest.approx(0.40)
    assert linear == sorted(linear)
    assert angular == sorted(angular)
    assert len(set(linear)) == 9
    assert len(set(angular)) == 9


@pytest.mark.parametrize(
    'value',
    [0, 10, '3.5', '', True, 'slow', 'normal', 'fast', 'quick'],
)
def test_invalid_speed_levels_are_rejected(value):
    with pytest.raises(ValueError, match='1 to 9'):
        normalize_speed_level(value)
