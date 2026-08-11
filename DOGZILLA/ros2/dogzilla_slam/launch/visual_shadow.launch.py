"""Combine calibrated mono camera, camera TF, and isolated RTAB-Map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    camera_launch = os.path.join(
        package_share,
        'launch',
        'mono_camera.launch.py',
    )
    description_launch = os.path.join(
        package_share,
        'launch',
        'robot_description.launch.py',
    )
    rtabmap_launch = os.path.join(
        package_share,
        'launch',
        'rtabmap_mono_shadow.launch.py',
    )

    enabled = LaunchConfiguration('enabled')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///calibration/camera.yaml',
        ),
        DeclareLaunchArgument('camera_x', default_value='0.150'),
        DeclareLaunchArgument('camera_y', default_value='0.000'),
        DeclareLaunchArgument('camera_z', default_value='0.075'),
        DeclareLaunchArgument('camera_roll', default_value='0.000'),
        DeclareLaunchArgument('camera_pitch', default_value='0.000'),
        DeclareLaunchArgument('camera_yaw', default_value='0.000'),
        DeclareLaunchArgument(
            'database_path',
            default_value='/logs/rtabmap_mono_shadow.db',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'enabled': enabled,
                'rectify': 'true',
                'use_sim_time': use_sim_time,
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'enabled': enabled,
                'use_sim_time': use_sim_time,
                'camera_x': LaunchConfiguration('camera_x'),
                'camera_y': LaunchConfiguration('camera_y'),
                'camera_z': LaunchConfiguration('camera_z'),
                'camera_roll': LaunchConfiguration('camera_roll'),
                'camera_pitch': LaunchConfiguration('camera_pitch'),
                'camera_yaw': LaunchConfiguration('camera_yaw'),
                'include_lidar': 'false',
                'include_imu': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                'enabled': enabled,
                'use_sim_time': use_sim_time,
                'database_path': LaunchConfiguration('database_path'),
            }.items(),
        ),
    ])
