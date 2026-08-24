"""Shared 1-9 DOGZILLA teleop speed-level configuration."""

from dataclasses import dataclass


MINIMUM_SPEED_LEVEL = 1
NORMAL_SPEED_LEVEL = 5
MAXIMUM_SPEED_LEVEL = 9


@dataclass(frozen=True)
class SpeedSetting:
    """One bounded speed level and its Yahboom controller equivalent."""

    level: int
    max_linear: float
    max_angular: float
    controller_pace: str
    controller_step: int
    label: str


@dataclass(frozen=True)
class TurnSetting:
    """One turn level that survives Yahboom's firmware-side clamps."""

    level: int
    max_angular: float
    controller_step: int


_CONTROLLER_STEPS = (4, 6, 7, 9, 10, 13, 15, 18, 20)
_TURN_STEPS = (30, 34, 38, 41, 45, 51, 58, 64, 70)


def _interpolate(level, low, normal, high):
    if level <= NORMAL_SPEED_LEVEL:
        fraction = (level - MINIMUM_SPEED_LEVEL) / (
            NORMAL_SPEED_LEVEL - MINIMUM_SPEED_LEVEL
        )
        return low + fraction * (normal - low)
    fraction = (level - NORMAL_SPEED_LEVEL) / (
        MAXIMUM_SPEED_LEVEL - NORMAL_SPEED_LEVEL
    )
    return normal + fraction * (high - normal)


def _pace(level):
    if level <= 2:
        return 'slow'
    if level >= 8:
        return 'high'
    return 'normal'


def _label(level):
    return {
        MINIMUM_SPEED_LEVEL: 'slow',
        NORMAL_SPEED_LEVEL: 'normal',
        MAXIMUM_SPEED_LEVEL: 'fast',
    }.get(level, f'level {level}')


SPEED_LEVELS = {
    level: SpeedSetting(
        level=level,
        max_linear=step / 40.0,
        max_angular=_interpolate(level, 0.30, 1.125, 1.75),
        controller_pace=_pace(level),
        controller_step=step,
        label=_label(level),
    )
    for level, step in enumerate(_CONTROLLER_STEPS, start=1)
}

TURN_LEVELS = {
    level: TurnSetting(
        level=level,
        max_angular=step / 40.0,
        controller_step=step,
    )
    for level, step in enumerate(_TURN_STEPS, start=1)
}

# Autonomous navigation stays below the teleop/controller envelope. Level 4
# is the default brisk indoor pace; higher levels remain explicit operator
# choices and never exceed the Nav2 smoother ceiling.
AUTONOMY_LINEAR_LIMITS = {
    level: value
    for level, value in enumerate(
        (0.08, 0.12, 0.16, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30),
        start=1,
    )
}

AUTONOMY_ANGULAR_LIMITS = {
    level: value
    for level, value in enumerate(
        (0.22, 0.28, 0.34, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65),
        start=1,
    )
}


def normalize_speed_level(value):
    """Return one integer speed level from 1 to 9."""
    if isinstance(value, bool):
        raise ValueError('speed level must be a whole number from 1 to 9')
    text = str(value).strip().lower()
    try:
        level = int(text, 10)
    except ValueError as exc:
        raise ValueError(
            'speed level must be a whole number from 1 to 9'
        ) from exc
    if text not in {str(level), f'+{level}'}:
        raise ValueError('speed level must be a whole number from 1 to 9')
    if not MINIMUM_SPEED_LEVEL <= level <= MAXIMUM_SPEED_LEVEL:
        raise ValueError('speed level must be a whole number from 1 to 9')
    return level


def speed_setting(value):
    """Return the immutable configuration for one normalized level."""
    return SPEED_LEVELS[normalize_speed_level(value)]
