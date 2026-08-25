"""Run hardware, pure localization, command arbitration, and optional Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    hardware_launch = os.path.join(package_share, 'launch', 'hardware.launch.py')
    localization_launch = os.path.join(
        package_share,
        'launch',
        'localization.launch.py',
    )
    nav2_launch = os.path.join(package_share, 'launch', 'nav2.launch.py')
    twist_mux_config = os.path.join(package_share, 'config', 'twist_mux.yaml')
    nav2_config = os.path.join(package_share, 'config', 'nav2_test1.yaml')

    use_imu = LaunchConfiguration('use_imu')
    use_nav2 = LaunchConfiguration('use_nav2')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value='/maps/test1.yaml'),
        DeclareLaunchArgument(
            'state_file',
            default_value='/maps/test1.pbstream',
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='dogzilla_localization.lua',
        ),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_nav2', default_value='false'),
        DeclareLaunchArgument('start_immediately', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/calibration/imu.json',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'speed_level': '4',
                'turn_level': '1',
                'use_imu': use_imu,
                'posture_control_enabled': 'false',
            }.items(),
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel')],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map_yaml': LaunchConfiguration('map_yaml'),
                'state_file': LaunchConfiguration('state_file'),
                'configuration_basename': LaunchConfiguration(
                    'configuration_basename'
                ),
                'correct_imu': use_imu,
                'start_immediately': LaunchConfiguration(
                    'start_immediately'
                ),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'params_file': nav2_config}.items(),
            condition=IfCondition(use_nav2),
        ),
        Node(
            package='dogzilla_slam',
            executable='navigation_diagnostics',
            name='dogzilla_navigation_diagnostics',
            condition=IfCondition(use_nav2),
            output='screen',
        ),
        Node(
            package='dogzilla_slam',
            executable='navigation_tuning_recorder',
            name='dogzilla_navigation_tuning_recorder',
            parameters=[{'nav2_params_file': nav2_config}],
            condition=IfCondition(use_nav2),
            output='screen',
        ),
    ])
