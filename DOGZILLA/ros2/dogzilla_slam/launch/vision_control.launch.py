"""Run explicitly armed Yahboom-style vision through the serial manager."""

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
    package_share = get_package_share_directory('dogzilla_slam')
    vision_launch = os.path.join(
        package_share,
        'launch',
        'vision.launch.py',
    )
    description_launch = os.path.join(
        package_share,
        'launch',
        'robot_description.launch.py',
    )
    armed = LaunchConfiguration('armed')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'armed',
            default_value='false',
            description=(
                'Startup-only hardware gate. False opens no robot serial port.'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('mode', default_value='raw'),
        DeclareLaunchArgument('color', default_value='red'),
        DeclareLaunchArgument('process_hz', default_value='10.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'camera_enabled': LaunchConfiguration('camera_enabled'),
                'use_sim_time': use_sim_time,
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'mode': LaunchConfiguration('mode'),
                'color': LaunchConfiguration('color'),
                'process_hz': LaunchConfiguration('process_hz'),
            }.items(),
        ),
        Node(
            package='dogzilla_slam',
            executable='safe_base',
            name='dogzilla_safe_base',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': '/vision_control/no_external_velocity',
                'accept_velocity_commands': False,
                'speed_profile': 'slow',
                'posture_control_enabled': False,
                'publish_imu': False,
                'vision_control_enabled': True,
                'vision_detection_topic': '/vision/detections',
                'vision_action_status_topic': '/vision/action_status',
                'vision_required_frames': 5,
                'vision_release_frames': 3,
                'vision_action_cooldown': 8.0,
                'vision_action_guard_seconds': 8.0,
                'vision_line_forward': 0.08,
                'vision_line_maximum_turn': 0.25,
            }],
            condition=IfCondition(armed),
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'enabled': armed,
                'use_sim_time': use_sim_time,
                'include_lidar': 'false',
                'include_imu': 'false',
            }.items(),
        ),
    ])
