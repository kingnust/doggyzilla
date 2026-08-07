"""Run pure Cartographer localization against a frozen PBStream map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    configuration_directory = os.path.join(package_share, 'config')
    nav2_rviz = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    state_file = LaunchConfiguration('state_file')
    map_yaml = LaunchConfiguration('map_yaml')
    configuration_basename = LaunchConfiguration('configuration_basename')
    correct_imu = LaunchConfiguration('correct_imu')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('state_file', default_value='/maps/test1.pbstream'),
        DeclareLaunchArgument('map_yaml', default_value='/maps/test1.yaml'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='dogzilla_localization.lua',
        ),
        DeclareLaunchArgument('correct_imu', default_value='false'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/calibration/imu.json',
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='dogzilla_slam',
            executable='imu_corrector',
            name='dogzilla_imu_corrector',
            parameters=[{
                'input_topic': '/imu/data_uncalibrated',
                'output_topic': '/imu/data_corrected',
                'calibration_file': LaunchConfiguration('calibration_file'),
                'output_frame': 'imu_link',
            }],
            condition=IfCondition(correct_imu),
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', state_file,
                '-load_frozen_state=true',
                '-start_trajectory_with_default_topics=false',
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu/data_corrected'),
            ],
            output='screen',
        ),
        Node(
            package='dogzilla_slam',
            executable='localization_manager',
            name='dogzilla_localization_manager',
            parameters=[{
                'configuration_directory': configuration_directory,
                'configuration_basename': configuration_basename,
                'map_frame': 'map',
                'start_immediately': True,
            }],
            output='screen',
        ),
        Node(
            package='dogzilla_slam',
            executable='tf_odometry',
            name='dogzilla_tf_odometry',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 4.0,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='dogzilla_navigation_rviz',
            arguments=['-d', nav2_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
