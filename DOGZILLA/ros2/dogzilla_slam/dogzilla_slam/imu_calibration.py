"""Pure-Python helpers for DOGZILLA IMU calibration and correction."""

from itertools import permutations, product
import json
import math
from pathlib import Path
from statistics import fmean


GRAVITY_M_S2 = 9.80665
SCHEMA_VERSION = 1

POSE_VECTORS = {
    'upright': (0.0, 0.0, 1.0),
    'upside_down': (0.0, 0.0, -1.0),
    'nose_up': (1.0, 0.0, 0.0),
    'nose_down': (-1.0, 0.0, 0.0),
    'left_side_up': (0.0, 1.0, 0.0),
    'right_side_up': (0.0, -1.0, 0.0),
}


def matvec(matrix, vector):
    """Multiply a flattened row-major 3x3 matrix by a 3-vector."""
    return [
        sum(matrix[row * 3 + column] * vector[column]
            for column in range(3))
        for row in range(3)
    ]


def determinant(matrix):
    """Return the determinant of a flattened 3x3 matrix."""
    a, b, c, d, e, f, g, h, i = matrix
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def vector_mean(samples):
    if not samples:
        raise ValueError('At least one sample is required')
    return [fmean(sample[index] for sample in samples) for index in range(3)]


def covariance(samples, minimum_diagonal):
    """Calculate a full sample covariance with a non-zero diagonal floor."""
    if len(samples) < 2:
        raise ValueError('At least two samples are required for covariance')
    mean = vector_mean(samples)
    denominator = len(samples) - 1
    result = []
    for row in range(3):
        for column in range(3):
            value = sum(
                (sample[row] - mean[row]) * (sample[column] - mean[column])
                for sample in samples
            ) / denominator
            if row == column:
                value = max(value, minimum_diagonal)
            result.append(value)
    return result


def _signed_permutation_matrices():
    for column_order in permutations(range(3)):
        for signs in product((-1.0, 1.0), repeat=3):
            matrix = [0.0] * 9
            for row, column in enumerate(column_order):
                matrix[row * 3 + column] = signs[row]
            yield matrix


def _normalized(vector):
    magnitude = math.sqrt(sum(value * value for value in vector))
    if magnitude < 1e-6:
        raise ValueError('IMU acceleration magnitude is zero')
    return [value / magnitude for value in vector]


def fit_axis_transform(pose_means):
    """Fit the best raw-accelerometer to ROS FLU signed permutation."""
    missing = sorted(set(POSE_VECTORS) - set(pose_means))
    if missing:
        raise ValueError(f'Missing calibration poses: {", ".join(missing)}')

    scored = []
    for matrix in _signed_permutation_matrices():
        squared_error = 0.0
        for pose_name, expected in POSE_VECTORS.items():
            measured = _normalized(matvec(matrix, pose_means[pose_name]))
            squared_error += sum(
                (measured[index] - expected[index]) ** 2
                for index in range(3)
            )
        scored.append((math.sqrt(squared_error / len(POSE_VECTORS)), matrix))

    scored.sort(key=lambda item: item[0])
    rms_unit_error, acceleration_matrix = scored[0]
    if rms_unit_error > 0.25:
        raise ValueError(
            'Axis calibration failed: pose error is too large '
            f'({rms_unit_error:.3f} > 0.250). Repeat the guided poses.'
        )

    # A determinant of -1 indicates that the controller reports gravity
    # rather than accelerometer specific force. Negating all axes recovers the
    # proper right-handed rotation used for angular velocity.
    if determinant(acceleration_matrix) > 0:
        gyro_matrix = list(acceleration_matrix)
        acceleration_convention = 'specific_force'
    else:
        gyro_matrix = [-value for value in acceleration_matrix]
        acceleration_convention = 'gravity_vector_inverted_to_specific_force'

    transformed_norms = []
    for pose_name in POSE_VECTORS:
        transformed = matvec(acceleration_matrix, pose_means[pose_name])
        transformed_norms.append(math.sqrt(sum(value * value for value in transformed)))
    acceleration_scale = GRAVITY_M_S2 / fmean(transformed_norms)

    return {
        'acceleration_matrix': acceleration_matrix,
        'gyro_matrix': gyro_matrix,
        'acceleration_scale': acceleration_scale,
        'acceleration_convention': acceleration_convention,
        'axis_rms_unit_error': rms_unit_error,
    }


def create_calibration(pose_samples, monotonic_stamps, created_utc):
    """Build a validated calibration document from six stationary poses."""
    pose_means = {
        name: vector_mean([sample[:3] for sample in samples])
        for name, samples in pose_samples.items()
    }
    fit = fit_axis_transform(pose_means)
    acceleration_matrix = fit['acceleration_matrix']
    gyro_matrix = fit['gyro_matrix']
    acceleration_scale = fit['acceleration_scale']

    upright = pose_samples['upright']
    corrected_acceleration = [
        [acceleration_scale * value for value in matvec(acceleration_matrix, sample[:3])]
        for sample in upright
    ]
    transformed_gyro = [
        matvec(gyro_matrix, [math.radians(value) for value in sample[3:6]])
        for sample in upright
    ]
    gyro_bias = vector_mean(transformed_gyro)
    unbiased_gyro = [
        [sample[index] - gyro_bias[index] for index in range(3)]
        for sample in transformed_gyro
    ]

    deltas = [
        later - earlier
        for earlier, later in zip(monotonic_stamps, monotonic_stamps[1:])
        if later > earlier
    ]
    if not deltas:
        raise ValueError('Could not measure IMU sample timing')

    return {
        'schema_version': SCHEMA_VERSION,
        'created_utc': created_utc,
        'axis_validated': True,
        'frame_id': 'imu_link',
        'frame_convention': 'ROS REP-145, x forward, y left, z up',
        'acceleration': {
            'matrix': acceleration_matrix,
            'scale': acceleration_scale,
            'convention': fit['acceleration_convention'],
            'gravity_m_s2': GRAVITY_M_S2,
            'upright_mean_m_s2': vector_mean(corrected_acceleration),
            'covariance': covariance(corrected_acceleration, 0.0025),
        },
        'angular_velocity': {
            'matrix': gyro_matrix,
            'input_units': 'rad/s',
            'bias_rad_s': gyro_bias,
            'covariance': covariance(unbiased_gyro, 0.000025),
        },
        'quality': {
            'axis_rms_unit_error': fit['axis_rms_unit_error'],
            'samples_per_pose': len(upright),
        },
        'timing': {
            'stamp_source': 'serial_receive_time',
            'sample_rate_hz': 1.0 / fmean(deltas),
            'mean_period_s': fmean(deltas),
            'max_gap_s': max(deltas),
            'monotonic': True,
        },
    }


def validate_calibration(document):
    """Validate calibration fields and return the document unchanged."""
    if document.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('Unsupported IMU calibration schema')
    if document.get('axis_validated') is not True:
        raise ValueError('IMU axes have not been validated')

    for section_name, matrix_name in (
        ('acceleration', 'matrix'),
        ('angular_velocity', 'matrix'),
    ):
        section = document.get(section_name, {})
        matrix = section.get(matrix_name)
        covariance_values = section.get('covariance')
        if not isinstance(matrix, list) or len(matrix) != 9:
            raise ValueError(f'{section_name}.{matrix_name} must contain 9 values')
        if not isinstance(covariance_values, list) or len(covariance_values) != 9:
            raise ValueError(f'{section_name}.covariance must contain 9 values')
        if any(not math.isfinite(float(value)) for value in matrix + covariance_values):
            raise ValueError(f'{section_name} contains non-finite values')
        if any(float(covariance_values[index]) <= 0.0 for index in (0, 4, 8)):
            raise ValueError(f'{section_name} covariance diagonal must be positive')

    bias = document['angular_velocity'].get('bias_rad_s')
    if not isinstance(bias, list) or len(bias) != 3:
        raise ValueError('angular_velocity.bias_rad_s must contain 3 values')
    scale = float(document['acceleration'].get('scale', 0.0))
    if not 0.5 <= scale <= 1.5:
        raise ValueError('Acceleration scale is outside the safe range 0.5..1.5')
    return document


def load_calibration(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return validate_calibration(json.load(stream))
