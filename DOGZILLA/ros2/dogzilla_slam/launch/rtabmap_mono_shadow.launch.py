"""Run isolated monocular RTAB-Map without owning hardware or publishing TF."""

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
    derive_odom_from_tf = LaunchConfiguration('derive_odom_from_tf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description=(
                'Safety gate. False starts no process and preserves mapping.'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dogzilla_slam'),
                'config',
                'rtabmap_mono_shadow.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_rect',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/rtabmap_shadow/odom_input',
        ),
        DeclareLaunchArgument(
            'derive_odom_from_tf',
            default_value='true',
            description=(
                'Convert Cartographer odom -> base_link TF into the isolated '
                'odom input. Disable when another odometry topic is used.'
            ),
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument(
            'database_path',
            default_value='/logs/rtabmap_mono_shadow.db',
            description='Persistent shadow database; never deleted on start.',
        ),
        GroupAction(
            condition=IfCondition(enabled),
            actions=[
                Node(
                    package='dogzilla_slam',
                    executable='tf_odometry',
                    namespace='rtabmap_shadow',
                    name='tf_odometry',
                    parameters=[{
                        'use_sim_time': ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                        'odom_topic': LaunchConfiguration('odom_topic'),
                    }],
                    condition=IfCondition(derive_odom_from_tf),
                    output='screen',
                ),
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    namespace='rtabmap_shadow',
                    name='rtabmap_mono',
                    parameters=[
                        params_file,
                        {
                            'use_sim_time': ParameterValue(
                                use_sim_time,
                                value_type=bool,
                            ),
                            'frame_id': LaunchConfiguration('frame_id'),
                            'database_path': LaunchConfiguration(
                                'database_path'
                            ),
                        },
                    ],
                    remappings=[
                        ('rgb/image', LaunchConfiguration('image_topic')),
                        (
                            'rgb/camera_info',
                            LaunchConfiguration('camera_info_topic'),
                        ),
                        ('odom', LaunchConfiguration('odom_topic')),
                        ('scan', LaunchConfiguration('scan_topic')),
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
