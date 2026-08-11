"""Calibration-gated DOGZILLA URDF publisher for visual shadow mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
            ' camera_x:=',
            LaunchConfiguration('camera_x'),
            ' camera_y:=',
            LaunchConfiguration('camera_y'),
            ' camera_z:=',
            LaunchConfiguration('camera_z'),
            ' camera_roll:=',
            LaunchConfiguration('camera_roll'),
            ' camera_pitch:=',
            LaunchConfiguration('camera_pitch'),
            ' camera_yaw:=',
            LaunchConfiguration('camera_yaw'),
            ' include_lidar:=',
            LaunchConfiguration('include_lidar'),
            ' include_imu:=',
            LaunchConfiguration('include_imu'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description=(
                'Safety gate. This framework is not part of normal mapping.'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('dogzilla_slam'),
                'urdf',
                'dogzilla_s2.urdf.xacro',
            ]),
        ),
        DeclareLaunchArgument('camera_x', default_value='0.150'),
        DeclareLaunchArgument('camera_y', default_value='0.000'),
        DeclareLaunchArgument('camera_z', default_value='0.075'),
        DeclareLaunchArgument('camera_roll', default_value='0.000'),
        DeclareLaunchArgument('camera_pitch', default_value='0.000'),
        DeclareLaunchArgument('camera_yaw', default_value='0.000'),
        DeclareLaunchArgument('include_lidar', default_value='true'),
        DeclareLaunchArgument('include_imu', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='dogzilla_robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }],
            condition=IfCondition(enabled),
            output='screen',
        ),
    ])
