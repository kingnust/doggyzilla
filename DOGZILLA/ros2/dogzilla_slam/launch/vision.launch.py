"""Launch one shared DOGZILLA camera and a safe vision processor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    camera_launch = os.path.join(
        package_share,
        'launch',
        'mono_camera.launch.py',
    )
    camera_enabled = LaunchConfiguration('camera_enabled')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('mode', default_value='raw'),
        DeclareLaunchArgument('color', default_value='red'),
        DeclareLaunchArgument('process_hz', default_value='10.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'enabled': camera_enabled,
                'rectify': 'false',
                'use_sim_time': use_sim_time,
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items(),
        ),
        Node(
            package='dogzilla_slam',
            executable='vision_node',
            name='dogzilla_vision',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'image_topic': LaunchConfiguration('image_topic'),
                'mode': LaunchConfiguration('mode'),
                'color': LaunchConfiguration('color'),
                'process_hz': ParameterValue(
                    LaunchConfiguration('process_hz'),
                    value_type=float,
                ),
            }],
            output='screen',
        ),
    ])
