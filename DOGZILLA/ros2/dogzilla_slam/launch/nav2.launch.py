"""Start a conservative forward-and-turn Nav2 stack for DOGZILLA."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('dogzilla_slam')
    safe_navigation_tree = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_to_pose_safe.xml',
    )
    safe_navigation_through_poses_tree = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_through_poses_safe.xml',
    )
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = [params_file, {'use_sim_time': use_sim_time}]

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=common_parameters,
            remappings=[('cmd_vel', '/cmd_vel_nav_raw')],
            output='screen',
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=common_parameters,
            remappings=[('cmd_vel', '/cmd_vel_nav_raw')],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': safe_navigation_tree,
                    'default_nav_through_poses_bt_xml': (
                        safe_navigation_through_poses_tree
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=common_parameters,
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', '/cmd_vel_nav_smoothed'),
            ],
            output='screen',
        ),
        Node(
            package='dogzilla_slam',
            executable='steering_guard',
            name='dogzilla_steering_guard',
            parameters=[{
                'input_topic': '/cmd_vel_nav_smoothed',
                'output_topic': '/cmd_vel_nav',
                'deadband_rps': 0.04,
                'reversal_hold_seconds': 0.25,
                'neutral_reset_seconds': 0.50,
                'bypass_angular_rps': 0.50,
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
            output='screen',
        ),
    ])
