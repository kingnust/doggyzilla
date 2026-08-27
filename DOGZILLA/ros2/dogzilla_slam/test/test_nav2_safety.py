"""Static regression checks for the conservative indoor Nav2 profile."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIGURATION = PACKAGE_ROOT / 'config' / 'nav2_test1.yaml'
SAFE_TREE = PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_safe.xml'
SAFE_THROUGH_POSES_TREE = (
    PACKAGE_ROOT
    / 'behavior_trees'
    / 'navigate_through_poses_safe.xml'
)


def _configuration():
    return yaml.safe_load(CONFIGURATION.read_text(encoding='utf-8'))


def test_measured_footprint_and_padding_replace_oversized_radius():
    configuration = _configuration()
    for costmap_name in ('local_costmap', 'global_costmap'):
        parameters = configuration[costmap_name][costmap_name][
            'ros__parameters'
        ]
        footprint = ast.literal_eval(parameters['footprint'])
        x_values = [point[0] for point in footprint]
        y_values = [point[1] for point in footprint]

        assert 'robot_radius' not in parameters
        assert max(x_values) - min(x_values) == 0.26
        assert max(y_values) - min(y_values) == 0.145
        assert parameters['footprint_padding'] == 0.03
        assert parameters['always_send_full_costmap'] is False


def test_path_follower_aligns_heading_then_tracks_forward_only():
    controller = _configuration()['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']

    assert controller['controller_frequency'] == 10.0
    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert follow_path['desired_linear_vel'] == 0.20
    assert follow_path['lookahead_dist'] == 0.50
    assert follow_path['min_lookahead_dist'] == 0.30
    assert follow_path['max_lookahead_dist'] == 0.65
    assert follow_path['lookahead_time'] == 2.5
    assert follow_path['use_velocity_scaled_lookahead_dist'] is True
    assert follow_path['rotate_to_heading_angular_vel'] == 0.22
    assert follow_path['use_rotate_to_heading'] is True
    assert follow_path['rotate_to_heading_min_angle'] <= 0.45
    assert follow_path['allow_reversing'] is False
    assert follow_path['use_collision_detection'] is True
    assert follow_path['use_regulated_linear_velocity_scaling'] is True
    assert follow_path['regulated_linear_scaling_min_speed'] <= 0.07

    smoother = _configuration()['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.20, 0.0, 0.22]
    assert smoother['min_velocity'] == [0.0, 0.0, -0.22]
    assert smoother['smoothing_frequency'] == 15.0

    package = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert '<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>' \
        in package


def test_recovery_tree_cannot_command_spin_or_backup():
    for path in (SAFE_TREE, SAFE_THROUGH_POSES_TREE):
        root = ET.parse(path).getroot()
        tags = {element.tag for element in root.iter()}
        recovery = root.find('.//RecoveryNode')

        assert recovery is not None
        assert recovery.attrib['number_of_retries'] == '1'
        assert 'Spin' not in tags
        assert 'BackUp' not in tags
        assert len(root.findall('.//ClearEntireCostmap')) == 2

    configuration = _configuration()
    behaviors = configuration['behavior_server']['ros__parameters']
    assert behaviors['behavior_plugins'] == ['wait']
    assert 'spin' not in behaviors
    assert 'backup' not in behaviors


def test_pi_timeouts_and_waypoint_failure_are_fail_safe():
    configuration = _configuration()
    navigator = configuration['bt_navigator']['ros__parameters']
    follower = configuration['waypoint_follower']['ros__parameters']

    assert navigator['bt_loop_duration'] >= 100
    assert navigator['default_server_timeout'] >= 500
    assert navigator['wait_for_service_timeout'] >= 2000
    assert follower['stop_on_failure'] is True


def test_launch_installs_and_selects_safe_navigation_tree():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    launch = (PACKAGE_ROOT / 'launch' / 'nav2.launch.py').read_text(
        encoding='utf-8'
    )

    assert "glob('behavior_trees/*.xml')" in setup
    assert "'navigate_to_pose_safe.xml'" in launch
    assert "'navigate_through_poses_safe.xml'" in launch
    assert "'default_nav_to_pose_bt_xml': safe_navigation_tree" in launch
    assert "'default_nav_through_poses_bt_xml': (" in launch


def test_localization_limits_background_work_on_the_pi():
    localization = (
        PACKAGE_ROOT / 'config' / 'dogzilla_localization.lua'
    ).read_text(encoding='utf-8')

    assert 'options.pose_publish_period_sec = 0.02' in localization
    assert 'MAP_BUILDER.num_background_threads = 4' in localization
    assert 'POSE_GRAPH.optimize_every_n_nodes = 10' in localization
    assert (
        'POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4'
        in localization
    )


def test_localization_waits_for_initial_pose_unless_match_is_explicit():
    localization_launch = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py'
    ).read_text(encoding='utf-8')
    full_navigation_launch = (
        PACKAGE_ROOT / 'launch' / 'full_navigation.launch.py'
    ).read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('start_immediately', default_value='false')" \
        in localization_launch
    assert "'start_immediately': ParameterValue(" in localization_launch
    assert "DeclareLaunchArgument('start_immediately', default_value='false')" \
        in full_navigation_launch


def test_web_odometry_uses_sensor_qos_and_reports_named_nav_status():
    gateway = (PACKAGE_ROOT / 'dogzilla_slam' / 'web_gateway.py').read_text(
        encoding='utf-8'
    )

    odometry_subscription = gateway.split(
        'self.create_subscription(\n            Odometry,', 1
    )[1].split(')', 1)[0]
    assert 'qos_profile_sensor_data' in odometry_subscription
    assert "GoalStatus.STATUS_ABORTED: 'aborted'" in gateway
    assert 'Nav2 waypoint {status_name} (status {status})' in gateway
