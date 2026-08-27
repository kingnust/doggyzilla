import math
from pathlib import Path

import pytest

from dogzilla_slam.steering_guard import SteeringGuardFilter


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def guard():
    return SteeringGuardFilter(
        deadband_rps=0.04,
        reversal_hold_seconds=0.25,
        neutral_reset_seconds=0.50,
        bypass_angular_rps=0.50,
    )


def test_deadband_and_same_direction_commands_pass_predictably():
    filter_ = guard()

    assert filter_.apply(0.03, now=0.0) == 0.0
    assert filter_.apply(0.10, now=0.1) == pytest.approx(0.10)
    assert filter_.apply(0.16, now=0.2) == pytest.approx(0.16)


def test_small_reversal_must_remain_consistent_for_hold_period():
    filter_ = guard()

    assert filter_.apply(0.12, now=1.0) == pytest.approx(0.12)
    assert filter_.apply(-0.12, now=1.1) == 0.0
    assert filter_.apply(-0.12, now=1.3) == 0.0
    assert filter_.apply(-0.12, now=1.36) == pytest.approx(-0.12)


def test_rapid_sign_jitter_never_reaches_the_opposite_direction():
    filter_ = guard()

    assert filter_.apply(0.11, now=2.0) == pytest.approx(0.11)
    assert filter_.apply(-0.11, now=2.1) == 0.0
    assert filter_.apply(0.11, now=2.2) == pytest.approx(0.11)
    assert filter_.apply(-0.11, now=2.3) == 0.0
    assert filter_.apply(0.0, now=2.4) == 0.0
    assert filter_.apply(-0.11, now=2.5) == 0.0


def test_long_neutral_or_input_gap_allows_a_fresh_direction():
    filter_ = guard()

    assert filter_.apply(0.10, now=3.0) == pytest.approx(0.10)
    assert filter_.apply(0.0, now=3.1) == 0.0
    assert filter_.apply(0.0, now=3.7) == 0.0
    assert filter_.apply(-0.10, now=3.8) == pytest.approx(-0.10)

    assert filter_.apply(0.10, now=4.5) == pytest.approx(0.10)


def test_large_reversal_bypass_and_invalid_input_fail_safe():
    filter_ = guard()

    assert filter_.apply(0.10, now=5.0) == pytest.approx(0.10)
    assert filter_.apply(-0.60, now=5.1) == pytest.approx(-0.60)
    assert filter_.apply(math.nan, now=5.2) == 0.0
    assert filter_.apply(-0.10, now=5.3) == pytest.approx(-0.10)


def test_parameters_reject_unsafe_ranges():
    with pytest.raises(ValueError):
        SteeringGuardFilter(deadband_rps=-0.01)
    with pytest.raises(ValueError):
        SteeringGuardFilter(reversal_hold_seconds=1.1)
    with pytest.raises(ValueError):
        SteeringGuardFilter(neutral_reset_seconds=0.01)
    with pytest.raises(ValueError):
        SteeringGuardFilter(deadband_rps=0.1, bypass_angular_rps=0.1)


def test_nav2_launch_wires_guard_between_smoother_and_mux():
    launch = (PACKAGE_ROOT / 'launch' / 'nav2.launch.py').read_text(
        encoding='utf-8'
    )
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')

    assert "('cmd_vel_smoothed', '/cmd_vel_nav_smoothed')" in launch
    assert "executable='steering_guard'" in launch
    assert "'input_topic': '/cmd_vel_nav_smoothed'" in launch
    assert "'output_topic': '/cmd_vel_nav'" in launch
    assert 'steering_guard = dogzilla_slam.steering_guard:main' in setup
