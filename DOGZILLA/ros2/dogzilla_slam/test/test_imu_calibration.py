import math
import unittest

from dogzilla_slam.imu_calibration import create_calibration
from dogzilla_slam.imu_calibration import determinant
from dogzilla_slam.imu_calibration import POSE_VECTORS
from dogzilla_slam.imu_calibration import validate_calibration


def synthetic_samples():
    poses = {}
    stamps = []
    stamp = 0.0
    for pose_name, expected in POSE_VECTORS.items():
        samples = []
        for index in range(30):
            # Emulate the observed Yahboom gravity-vector convention (-g when
            # upright), plus small repeatable sensor noise and gyro bias.
            noise = (index % 3 - 1) * 0.01
            acceleration = [
                -9.60 * expected[0] + noise,
                -9.60 * expected[1] - noise,
                -9.60 * expected[2] + noise,
            ]
            gyro_degrees = [1.0, -0.5, 0.25]
            samples.append(acceleration + gyro_degrees)
            stamps.append(stamp)
            stamp += 0.05
        poses[pose_name] = samples
    return poses, stamps


class ImuCalibrationTest(unittest.TestCase):
    def test_create_calibration_corrects_gravity_and_gyro_units(self):
        poses, stamps = synthetic_samples()
        document = create_calibration(
            poses,
            stamps,
            '2026-01-01T00:00:00+00:00',
        )

        self.assertTrue(document['axis_validated'])
        self.assertAlmostEqual(
            determinant(document['angular_velocity']['matrix']),
            1.0,
        )
        upright = document['acceleration']['upright_mean_m_s2']
        self.assertAlmostEqual(upright[0], 0.0, delta=0.02)
        self.assertAlmostEqual(upright[1], 0.0, delta=0.02)
        self.assertAlmostEqual(upright[2], 9.80665, delta=0.02)
        for measured, expected in zip(
            document['angular_velocity']['bias_rad_s'],
            map(math.radians, [1.0, -0.5, 0.25]),
        ):
            self.assertAlmostEqual(measured, expected)
        validate_calibration(document)

    def test_validation_rejects_unvalidated_axes(self):
        poses, stamps = synthetic_samples()
        document = create_calibration(
            poses,
            stamps,
            '2026-01-01T00:00:00+00:00',
        )
        document['axis_validated'] = False
        with self.assertRaisesRegex(ValueError, 'axes'):
            validate_calibration(document)


if __name__ == '__main__':
    unittest.main()
