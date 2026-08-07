"""Start only the DOGZILLA hardware nodes required for safe 2D mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lidar_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch',
        'ms200_scan.launch.py',
    )

    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    command_timeout = LaunchConfiguration('command_timeout')
    speed_profile = LaunchConfiguration('speed_profile')
    posture_control_enabled = LaunchConfiguration('posture_control_enabled')
    use_imu = LaunchConfiguration('use_imu')

    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='0.10'),
        DeclareLaunchArgument('max_angular', default_value='0.30'),
        DeclareLaunchArgument('command_timeout', default_value='0.60'),
        DeclareLaunchArgument('speed_profile', default_value='slow'),
        DeclareLaunchArgument(
            'posture_control_enabled',
            default_value='false',
            description='Disabled when LiDAR requires a fixed body transform.',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Read the IMU through the single serial owner.',
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),
        Node(
            package='dogzilla_slam',
            executable='safe_base',
            name='dogzilla_safe_base',
            parameters=[{
                'input_topic': '/cmd_vel',
                'max_linear': ParameterValue(max_linear, value_type=float),
                'max_angular': ParameterValue(max_angular, value_type=float),
                'command_timeout': ParameterValue(
                    command_timeout,
                    value_type=float,
                ),
                'speed_profile': speed_profile,
                'posture_control_enabled': ParameterValue(
                    posture_control_enabled,
                    value_type=bool,
                ),
                'publish_imu': ParameterValue(use_imu, value_type=bool),
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dogzilla_imu_static_transform',
            arguments=[
                '--x', '0.085',
                '--y', '0.0',
                '--z', '0.070',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
            condition=IfCondition(use_imu),
            output='screen',
        ),
    ])
