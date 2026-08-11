"""Start the DOGZILLA monocular USB camera and optional rectification."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enabled = LaunchConfiguration('enabled')
    rectify = LaunchConfiguration('rectify')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('rectify', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dogzilla_slam'),
                'config',
                'mono_camera.yaml',
            ]),
        ),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///calibration/camera.yaml',
        ),
        GroupAction(
            condition=IfCondition(enabled),
            actions=[
                Node(
                    package='usb_cam',
                    executable='usb_cam_node_exe',
                    namespace='camera',
                    name='mono_camera',
                    parameters=[
                        LaunchConfiguration('params_file'),
                        {
                            'video_device': LaunchConfiguration(
                                'video_device'
                            ),
                            'camera_info_url': LaunchConfiguration(
                                'camera_info_url'
                            ),
                            'use_sim_time': ParameterValue(
                                use_sim_time,
                                value_type=bool,
                            ),
                        },
                    ],
                    output='screen',
                ),
                Node(
                    package='image_proc',
                    executable='rectify_node',
                    namespace='camera',
                    name='rectify',
                    remappings=[
                        ('image', 'image_raw'),
                        ('camera_info', 'camera_info'),
                    ],
                    parameters=[{
                        'use_sim_time': ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    }],
                    condition=IfCondition(rectify),
                    output='screen',
                ),
            ],
        ),
    ])
