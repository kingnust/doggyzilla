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
        DeclareLaunchArgument('object_process_hz', default_value='2.0'),
        DeclareLaunchArgument(
            'object_model_path',
            default_value='/models/yolox_nano.onnx',
        ),
        DeclareLaunchArgument(
            'open_images_model_path',
            default_value='/models/yolov8n-oiv7.onnx',
        ),
        DeclareLaunchArgument(
            'custom_object_model_path',
            default_value='/models/dogzilla_custom.onnx',
        ),
        DeclareLaunchArgument(
            'custom_object_labels_path',
            default_value='/models/dogzilla_custom.labels',
        ),
        DeclareLaunchArgument(
            'yoloe_object_model_path',
            default_value='/models/yoloe_small_hazards.onnx',
        ),
        DeclareLaunchArgument(
            'yoloe_object_labels_path',
            default_value='/models/yoloe_small_hazards.labels',
        ),
        DeclareLaunchArgument('floor_scan_columns', default_value='2'),
        DeclareLaunchArgument('floor_scan_overlap', default_value='0.18'),
        DeclareLaunchArgument(
            'danger_minimum_confidence', default_value='0.65'
        ),
        DeclareLaunchArgument(
            'danger_minimum_observations', default_value='3'
        ),
        DeclareLaunchArgument(
            'danger_minimum_duration_seconds', default_value='0.8'
        ),
        DeclareLaunchArgument('danger_minimum_iou', default_value='0.35'),
        DeclareLaunchArgument(
            'danger_maximum_gap_seconds', default_value='1.5'
        ),
        DeclareLaunchArgument('danger_cooldown_seconds', default_value='8.0'),
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
                'object_process_hz': ParameterValue(
                    LaunchConfiguration('object_process_hz'),
                    value_type=float,
                ),
                'object_model_path': LaunchConfiguration(
                    'object_model_path'
                ),
                'open_images_model_path': LaunchConfiguration(
                    'open_images_model_path'
                ),
                'custom_object_model_path': LaunchConfiguration(
                    'custom_object_model_path'
                ),
                'custom_object_labels_path': LaunchConfiguration(
                    'custom_object_labels_path'
                ),
                'yoloe_object_model_path': LaunchConfiguration(
                    'yoloe_object_model_path'
                ),
                'yoloe_object_labels_path': LaunchConfiguration(
                    'yoloe_object_labels_path'
                ),
                'floor_scan_columns': ParameterValue(
                    LaunchConfiguration('floor_scan_columns'),
                    value_type=int,
                ),
                'floor_scan_overlap': ParameterValue(
                    LaunchConfiguration('floor_scan_overlap'),
                    value_type=float,
                ),
                'danger_minimum_confidence': ParameterValue(
                    LaunchConfiguration('danger_minimum_confidence'),
                    value_type=float,
                ),
                'danger_minimum_observations': ParameterValue(
                    LaunchConfiguration('danger_minimum_observations'),
                    value_type=int,
                ),
                'danger_minimum_duration_seconds': ParameterValue(
                    LaunchConfiguration('danger_minimum_duration_seconds'),
                    value_type=float,
                ),
                'danger_minimum_iou': ParameterValue(
                    LaunchConfiguration('danger_minimum_iou'),
                    value_type=float,
                ),
                'danger_maximum_gap_seconds': ParameterValue(
                    LaunchConfiguration('danger_maximum_gap_seconds'),
                    value_type=float,
                ),
                'danger_cooldown_seconds': ParameterValue(
                    LaunchConfiguration('danger_cooldown_seconds'),
                    value_type=float,
                ),
            }],
            output='screen',
        ),
    ])
