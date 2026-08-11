"""Run guarded DOGZILLA monocular intrinsic calibration on the Pi display."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_launch = os.path.join(
        get_package_share_directory('dogzilla_slam'),
        'launch',
        'mono_camera.launch.py',
    )
    enabled = LaunchConfiguration('enabled')
    calibrator = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='dogzilla_camera_calibrator',
        arguments=[
            '--size',
            LaunchConfiguration('board_size'),
            '--square',
            LaunchConfiguration('square_size'),
            '--camera_name',
            'dogzilla_mono',
            '--max-chessboard-speed',
            LaunchConfiguration('max_chessboard_speed'),
        ],
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera', '/camera'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('board_size'),
        DeclareLaunchArgument('square_size'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///calibration/camera.pending.yaml',
        ),
        DeclareLaunchArgument(
            'max_chessboard_speed',
            default_value='0.5',
        ),
        GroupAction(
            condition=IfCondition(enabled),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(camera_launch),
                    launch_arguments={
                        'enabled': 'true',
                        'rectify': 'false',
                        'camera_info_url': LaunchConfiguration(
                            'camera_info_url'
                        ),
                    }.items(),
                ),
                TimerAction(period=2.0, actions=[calibrator]),
                RegisterEventHandler(OnProcessExit(
                    target_action=calibrator,
                    on_exit=[EmitEvent(event=Shutdown(
                        reason='camera calibration window closed',
                    ))],
                )),
            ],
        ),
    ])
