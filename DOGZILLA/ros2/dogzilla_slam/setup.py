from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dogzilla_slam'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.lua'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DOGZILLA maintainer',
    maintainer_email='pi@raspberrypi.local',
    description=(
        'Pi-only Cartographer mapping support for the Yahboom DOGZILLA S2.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_calibrate = dogzilla_slam.imu_calibrate:main',
            'imu_corrector = dogzilla_slam.imu_corrector:main',
            'imu_validate = dogzilla_slam.imu_validate:main',
            'lidar_off = dogzilla_slam.lidar_off:main',
            'safe_base = dogzilla_slam.safe_base:main',
            'save_map = dogzilla_slam.save_map:main',
            'teleop = dogzilla_slam.teleop:main',
        ],
    },
)
