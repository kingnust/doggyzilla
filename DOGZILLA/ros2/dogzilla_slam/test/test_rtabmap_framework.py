"""Static checks for the gated monocular RTAB-Map shadow deployment."""

from pathlib import Path
import importlib.util
import re

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / 'config' / 'rtabmap_mono_shadow.yaml'
LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'rtabmap_mono_shadow.launch.py'
CAMERA_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'mono_camera.launch.py'
CALIBRATION_LAUNCH_PATH = (
    PACKAGE_ROOT / 'launch' / 'camera_calibration.launch.py'
)
COMBINED_LAUNCH_PATH = PACKAGE_ROOT / 'launch' / 'visual_shadow.launch.py'
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
        CAMERA_LAUNCH_PATH,
        CALIBRATION_LAUNCH_PATH,
        COMBINED_LAUNCH_PATH,
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


def test_database_is_persistent_and_normal_mapping_is_unchanged():
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


def test_camera_and_combined_launches_are_disabled_by_default():
    for path in (
        CAMERA_LAUNCH_PATH,
        CALIBRATION_LAUNCH_PATH,
        COMBINED_LAUNCH_PATH,
    ):
        source = path.read_text()
        compile(source, str(path), 'exec')
        assert re.search(
            r"DeclareLaunchArgument\(\s*'enabled',\s*default_value='false'",
            source,
        )
    camera_source = CAMERA_LAUNCH_PATH.read_text()
    assert 'GroupAction(' in camera_source
    assert 'condition=IfCondition(enabled)' in camera_source


def test_camera_calibration_launch_uses_exact_topics_and_commit_service():
    source = CALIBRATION_LAUNCH_PATH.read_text()
    assert "package='camera_calibration'" in source
    assert "executable='cameracalibrator'" in source
    assert "('image', '/camera/image_raw')" in source
    assert "('camera', '/camera')" in source
    assert "'dogzilla_mono'" in source
    assert "'rectify': 'false'" in source
    assert 'OnProcessExit(' in source
    assert 'camera calibration window closed' in source


def test_shadow_deployment_is_separate_and_has_no_motion_ownership():
    compose_path = REPOSITORY_ROOT / 'deploy' / 'compose.yaml'
    if not compose_path.is_file():
        # The runtime image intentionally copies only this ROS package.
        return
    compose_source = compose_path.read_text()
    command_source = (
        REPOSITORY_ROOT / 'deploy' / 'dogzilla-map'
    ).read_text()
    package_lock = (
        REPOSITORY_ROOT / 'deploy' / 'ros-packages.lock'
    ).read_text()
    dockerfile = (
        REPOSITORY_ROOT / 'deploy' / 'Dockerfile'
    ).read_text()
    camera_profile = yaml.safe_load(CAMERA_LAUNCH_PATH.parent.parent.joinpath(
        'config', 'mono_camera.yaml'
    ).read_text())

    shadow_service = compose_source.split('  shadow:', 1)[1].split(
        '\n  web:', 1
    )[0]
    assert 'visual_shadow.launch.py' in shadow_service
    assert '/dev/video0:/dev/video0' in shadow_service
    assert '/dev/ttyAMA0' not in shadow_service
    assert '/dev/ttyAMA1' not in shadow_service
    assert '/cmd_vel' not in shadow_service
    assert 'privileged: true' not in shadow_service
    assert 'require_valid_camera_model' in command_source
    assert 'camera_extrinsics.yaml' in command_source
    assert 'camera-calibrate' in command_source
    assert '--network none' in command_source
    assert '--device /dev/video0:/dev/video0' in command_source
    assert 'com.dogzilla.role=camera-calibration' in command_source
    assert 'camera-calibration-current' in command_source
    assert 'reconcile_camera_calibration' in command_source
    assert 'timeout --signal=INT --kill-after=30s 1800s' in command_source
    assert '--intrinsics /calibration/camera.yaml' in command_source
    assert 'ros2 run dogzilla_slam shadow_validate' in command_source
    assert 'shadow-health-report.json' in command_source
    assert 'shadow-route-check [SECONDS] [MIN_METRES]' in command_source
    assert 'shadow-db-check [SESSION]' in command_source
    assert '/rtabmap_shadow/info' in shadow_service
    assert 'ros2 topic echo /rtabmap_shadow/info' in shadow_service
    assert '--no-daemon --spin-time 2 --once' in shadow_service
    assert 'm.ref_id > 0 and len(m.wm_state) > 0' in shadow_service
    shadow_start_body = command_source.split(
        'start_visual_shadow() {', 1
    )[1].split('\n}\n', 1)[0]
    assert shadow_start_body.index('require_valid_camera_model') < (
        shadow_start_body.index('start_mapping "$@"')
    )
    assert shadow_start_body.index('start_mapping "$@"') < (
        shadow_start_body.index('compose up')
    )
    shadow_check_body = command_source.split('check_shadow() {', 1)[1].split(
        '\n}\n', 1
    )[0]
    assert 'set -e' in shadow_check_body
    route_check_body = command_source.split(
        'check_shadow_route() {', 1
    )[1].split('\n}\n', 1)[0]
    assert '--require-loop-closure' in route_check_body
    assert '--minimum-travel-metres "$2"' in route_check_body
    assert '--maximum-return-metres 0.75' in route_check_body
    assert 'shadow-route-report.json' in route_check_body
    assert 'route-camera-report.json' in route_check_body
    assert '/cmd_vel' not in route_check_body
    assert 'safe_base' not in route_check_body
    database_check_body = command_source.split(
        'check_shadow_database() {', 1
    )[1].split('\n}\n', 1)[0]
    assert 'database_validate' in database_check_body
    assert '--expected-version 0.23.7' in database_check_body
    assert 'shadow-database-report.json' in database_check_body
    assert 'shadow_is_running' in database_check_body
    assert '/cmd_vel' not in database_check_body
    assert 'rtabmap-slam=' in package_lock
    assert 'rtabmap-msgs=' in package_lock
    assert 'a180538def5056d89563f7275baaa7c3fae01316' in dockerfile
    assert (
        'f91976d9b2091b20465d15f58f81ae57d7fae3b3b69d36803d56821ffce51e9e'
        in dockerfile
    )
    assert '--packages-select usb_cam oradar_lidar dogzilla_slam' in dockerfile
    assert '--allow-overriding usb_cam' in dockerfile
    assert camera_profile['/camera/mono_camera']['ros__parameters'][
        'skip_device_check'
    ] is True


def test_future_runtime_dependencies_are_declared():
    manifest = (PACKAGE_ROOT / 'package.xml').read_text()
    assert '<exec_depend>robot_state_publisher</exec_depend>' in manifest
    assert '<exec_depend>camera_calibration</exec_depend>' in manifest
    assert '<exec_depend>rtabmap_slam</exec_depend>' in manifest
    assert '<exec_depend>rtabmap_msgs</exec_depend>' in manifest
    assert '<exec_depend>image_proc</exec_depend>' in manifest
    assert '<exec_depend>usb_cam</exec_depend>' in manifest
    assert '<exec_depend>xacro</exec_depend>' in manifest

    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert 'shadow_validate = dogzilla_slam.shadow_validate:main' in setup_source
    assert (
        'database_validate = dogzilla_slam.database_validate:main'
        in setup_source
    )
