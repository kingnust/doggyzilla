"""Run the safe DOGZILLA hardware bridge and Cartographer in one launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    hardware_launch = os.path.join(
        package_share,
        'launch',
        'hardware.launch.py',
    )
    mapping_launch = os.path.join(
        package_share,
        'launch',
        'mapping.launch.py',
    )

    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    command_timeout = LaunchConfiguration('command_timeout')
    speed_profile = LaunchConfiguration('speed_profile')
    resolution = LaunchConfiguration('resolution')
    rviz = LaunchConfiguration('rviz')
    use_imu = LaunchConfiguration('use_imu')
    configuration_basename = LaunchConfiguration('configuration_basename')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='0.10'),
        DeclareLaunchArgument('max_angular', default_value='0.30'),
        DeclareLaunchArgument('command_timeout', default_value='0.60'),
        DeclareLaunchArgument('speed_profile', default_value='slow'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='dogzilla_2d.lua',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/calibration/imu.json',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'max_linear': max_linear,
                'max_angular': max_angular,
                'command_timeout': command_timeout,
                'speed_profile': speed_profile,
                'use_imu': use_imu,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'resolution': resolution,
                'correct_imu': use_imu,
                'configuration_basename': configuration_basename,
                'calibration_file': calibration_file,
                'rviz': rviz,
            }.items(),
        ),
    ])
