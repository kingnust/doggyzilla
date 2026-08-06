"""Launch Pi-only LiDAR mapping after DOGZILLA hardware bring-up."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    configuration_directory = os.path.join(package_share, 'config')
    rviz_configuration = os.path.join(
        package_share,
        'rviz',
        'dogzilla_mapping.rviz',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    raw_imu_topic = LaunchConfiguration('raw_imu_topic')
    corrected_imu_topic = LaunchConfiguration('corrected_imu_topic')
    resolution = LaunchConfiguration('resolution')
    configuration_basename = LaunchConfiguration('configuration_basename')
    correct_imu = LaunchConfiguration('correct_imu')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'raw_imu_topic',
            default_value='/imu/data_uncalibrated',
        ),
        DeclareLaunchArgument(
            'corrected_imu_topic',
            default_value='/imu/data_corrected',
        ),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='dogzilla_2d.lua',
        ),
        DeclareLaunchArgument(
            'correct_imu',
            default_value='false',
            description='Publish corrected IMU data if raw IMU is running.',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/calibration/imu.json',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz; requires DISPLAY access from Docker.',
        ),
        Node(
            package='dogzilla_slam',
            executable='imu_corrector',
            name='dogzilla_imu_corrector',
            parameters=[{
                'input_topic': raw_imu_topic,
                'output_topic': corrected_imu_topic,
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
                '-configuration_directory',
                configuration_directory,
                '-configuration_basename',
                configuration_basename,
            ],
            remappings=[
                ('scan', scan_topic),
                ('imu', corrected_imu_topic),
            ],
            output='screen',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': ParameterValue(resolution, value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='dogzilla_mapping_rviz',
            arguments=['-d', rviz_configuration],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
