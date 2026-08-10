import pytest

from dogzilla_slam.teleop import next_posture


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
