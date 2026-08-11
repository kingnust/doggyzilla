"""Static checks for the disabled monocular RTAB-Map shadow framework."""

from pathlib import Path
import importlib.util
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'rtabmap_mono_shadow.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'rtabmap_mono_shadow.launch.py'
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _config_value(source, key):
    match = re.search(
        rf"^\s+(?:'{re.escape(key)}'|{re.escape(key)}):\s*(.*?)\s*$",
        source,
        flags=re.MULTILINE,
    )
    assert match is not None, f'configuration key not found: {key}'
    return match.group(1).strip("'\"")


def test_launch_is_valid_python_and_disabled_by_default():
    source = LAUNCH_PATH.read_text()
    compile(source, str(LAUNCH_PATH), 'exec')
    assert re.search(
        r"DeclareLaunchArgument\(\s*'enabled',\s*default_value='false'",
        source,
    )
    assert 'condition=IfCondition(enabled)' in source
    assert "namespace='rtabmap_shadow'" in source
    assert "package='rtabmap_slam'" in source
    assert "executable='rtabmap'" in source
    assert '--delete_db_on_start' not in source


def test_launch_descriptions_construct_when_ros_is_available():
    if importlib.util.find_spec('launch') is None:
        return
    try:
        from launch import LaunchDescription
    except ImportError:
        return
    assert LaunchDescription is not None
    for path in (
        LAUNCH_PATH,
        PACKAGE_ROOT / 'launch' / 'robot_description.launch.py',
    ):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        assert description.entities


def test_monocular_camera_scan_and_external_odom_contract():
    source = LAUNCH_PATH.read_text()
    assert "default_value='/camera/image_rect'" in source
    assert "default_value='/camera/camera_info'" in source
    assert "default_value='/rtabmap_shadow/odom_input'" in source
    assert "default_value='/scan'" in source
    assert "('rgb/image', LaunchConfiguration('image_topic'))" in source
    assert "('odom', LaunchConfiguration('odom_topic'))" in source
    assert "('scan', LaunchConfiguration('scan_topic'))" in source
    assert "executable='tf_odometry'" in source
    assert "'derive_odom_from_tf',\n            default_value='true'" in source
    assert 'condition=IfCondition(derive_odom_from_tf)' in source


def test_shadow_configuration_cannot_compete_for_cartographer_tf():
    source = CONFIG_PATH.read_text()
    assert source.startswith('/rtabmap_shadow/rtabmap_mono:')
    assert _config_value(source, 'publish_tf') == 'false'
    assert _config_value(source, 'map_frame_id') == 'rtabmap_shadow_map'
    assert _config_value(source, 'frame_id') == 'base_link'
    assert _config_value(source, 'subscribe_rgb') == 'true'
    assert _config_value(source, 'subscribe_depth') == 'false'
    assert _config_value(source, 'subscribe_scan') == 'true'
    assert _config_value(source, 'Reg/Strategy') == '2'
    assert _config_value(source, 'Reg/Force3DoF') == 'true'
    assert _config_value(source, 'Grid/FromDepth') == 'false'


def test_parameter_file_has_native_ros_types_and_expected_namespace():
    document = yaml.safe_load(CONFIG_PATH.read_text())
    assert set(document) == {'/rtabmap_shadow/rtabmap_mono'}
    node_config = document['/rtabmap_shadow/rtabmap_mono']
    assert set(node_config) == {'ros__parameters'}
    parameters = node_config['ros__parameters']

    assert parameters['subscribe_rgb'] is True
    assert parameters['subscribe_depth'] is False
    assert parameters['subscribe_rgbd'] is False
    assert parameters['subscribe_stereo'] is False
    assert parameters['subscribe_scan'] is True
    assert parameters['subscribe_scan_cloud'] is False
    assert parameters['publish_tf'] is False
    assert parameters['odom_frame_id'] == ''
    assert parameters['qos_image'] == 2
    assert parameters['qos_scan'] == 2
    assert parameters['qos_odom'] == 2
    assert parameters['topic_queue_size'] >= 10
    assert parameters['sync_queue_size'] >= 10


def test_configuration_is_planar_visual_plus_icp_mapping():
    document = yaml.safe_load(CONFIG_PATH.read_text())
    parameters = document['/rtabmap_shadow/rtabmap_mono'][
        'ros__parameters'
    ]
    assert parameters['Reg/Strategy'] == '2'
    assert parameters['Reg/Force3DoF'] == 'true'
    assert parameters['Optimizer/Slam2D'] == 'true'
    assert parameters['Grid/FromDepth'] == 'false'
    assert parameters['Grid/3D'] == 'false'
    assert float(parameters['Grid/CellSize']) > 0.0
    assert float(parameters['Rtabmap/DetectionRate']) > 0.0
    assert int(parameters['Vis/MinInliers']) >= 10
    assert int(parameters['Kp/MaxFeatures']) >= 100


def test_database_is_persistent_and_framework_is_not_integrated():
    config_source = CONFIG_PATH.read_text()
    launch_source = LAUNCH_PATH.read_text()
    assert '/logs/rtabmap_mono_shadow.db' in config_source
    assert '/logs/rtabmap_mono_shadow.db' in launch_source
    assert 'delete_db' not in config_source
    assert 'delete_db' not in launch_source

    for launch_name in ('full_mapping.launch.py', 'full_navigation.launch.py'):
        operational_source = (
            PACKAGE_ROOT / 'launch' / launch_name
        ).read_text()
        assert 'rtabmap_mono_shadow.launch.py' not in operational_source


def test_framework_has_no_hardware_or_motion_ownership():
    combined_source = LAUNCH_PATH.read_text() + CONFIG_PATH.read_text()
    forbidden_tokens = (
        '/cmd_vel',
        '/dev/ttyAMA0',
        '/dev/ttyAMA1',
        '/dev/video',
        'safe_base',
        'usb_cam',
    )
    for token in forbidden_tokens:
        assert token not in combined_source


def test_framework_is_absent_from_deployment_entry_points():
    deploy_files = (
        REPOSITORY_ROOT / 'deploy' / 'Dockerfile',
        REPOSITORY_ROOT / 'deploy' / 'compose.yaml',
        REPOSITORY_ROOT / 'deploy' / 'dogzilla-map',
        REPOSITORY_ROOT / 'deploy' / 'ros-packages.lock',
    )
    for path in deploy_files:
        source = path.read_text()
        assert 'rtabmap_mono_shadow' not in source
        assert 'robot_description.launch.py' not in source

    package_lock = (
        REPOSITORY_ROOT / 'deploy' / 'ros-packages.lock'
    ).read_text()
    assert 'rtabmap' not in package_lock.lower()


def test_future_runtime_dependencies_are_declared():
    manifest = (PACKAGE_ROOT / 'package.xml').read_text()
    assert '<exec_depend>robot_state_publisher</exec_depend>' in manifest
    assert '<exec_depend>rtabmap_slam</exec_depend>' in manifest
    assert '<exec_depend>xacro</exec_depend>' in manifest
