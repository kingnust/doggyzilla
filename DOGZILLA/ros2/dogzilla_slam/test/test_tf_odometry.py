import math
from types import SimpleNamespace

import pytest

from dogzilla_slam.tf_odometry import normalize_angle, quaternion_yaw


def test_quaternion_yaw_for_quarter_turn():
    half_angle = math.pi / 4.0
    quaternion = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(half_angle),
        w=math.cos(half_angle),
    )

    assert quaternion_yaw(quaternion) == pytest.approx(math.pi / 2.0)


@pytest.mark.parametrize(
    'source, expected',
    [
        (3.0 * math.pi, math.pi),
        (-3.0 * math.pi, -math.pi),
        (0.25, 0.25),
    ],
)
def test_normalize_angle(source, expected):
    assert normalize_angle(source) == pytest.approx(expected)
