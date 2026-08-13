from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dogzilla_slam'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web_static/*']},
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
            glob('config/*.lua') + glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='DOGZILLA maintainer',
    maintainer_email='pi@raspberrypi.local',
    description=(
        'Pi-only Cartographer mapping support for the Yahboom DOGZILLA S2.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_model = dogzilla_slam.camera_model:main',
            'camera_validate = dogzilla_slam.camera_validate:main',
            'database_validate = dogzilla_slam.database_validate:main',
            'imu_calibrate = dogzilla_slam.imu_calibrate:main',
            'imu_corrector = dogzilla_slam.imu_corrector:main',
            'imu_validate = dogzilla_slam.imu_validate:main',
            'lidar_off = dogzilla_slam.lidar_off:main',
            'firmware_rest_monitor = '
            'dogzilla_slam.firmware_rest_monitor:main',
            'localization_manager = dogzilla_slam.localization_manager:main',
            'safe_base = dogzilla_slam.safe_base:main',
            'save_map = dogzilla_slam.save_map:main',
            'servo_power = dogzilla_slam.servo_power:main',
            'shadow_validate = dogzilla_slam.shadow_validate:main',
            'teleop = dogzilla_slam.teleop:main',
            'tf_odometry = dogzilla_slam.tf_odometry:main',
            'vision_node = dogzilla_slam.vision_node:main',
            'web_gateway = dogzilla_slam.web_gateway:main',
        ],
    },
)
