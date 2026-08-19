from pathlib import Path
import os
import unittest

import yaml


REPOSITORY = Path(
    os.environ.get(
        'DOGZILLA_TEST_REPOSITORY',
        Path(__file__).resolve().parents[3],
    )
).resolve()


class VisionDeploymentTest(unittest.TestCase):
    def test_standalone_vision_has_camera_but_no_robot_devices(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        service = compose['services']['vision']
        self.assertEqual(service['devices'], ['/dev/video0:/dev/video0'])
        serialized = yaml.safe_dump(service)
        self.assertNotIn('/dev/ttyAMA0', serialized)
        self.assertNotIn('/dev/ttyAMA1', serialized)
        self.assertIn('vision.launch.py', service['command'])
        self.assertEqual(service['healthcheck']['timeout'], '15s')

    def test_web_gateway_has_no_devices(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        service = compose['services']['web']
        self.assertNotIn('devices', service)
        self.assertEqual(
            service['environment']['DOGZILLA_WEB_KEEPOUT_CLEARANCE'],
            '${DOGZILLA_WEB_KEEPOUT_CLEARANCE:-0.32}',
        )

    def test_mission_perception_is_camera_only_and_detection_only(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        service = compose['services']['perception']
        self.assertEqual(service['devices'], ['/dev/video0:/dev/video0'])
        serialized = yaml.safe_dump(service)
        self.assertNotIn('/dev/ttyAMA0', serialized)
        self.assertNotIn('/dev/ttyAMA1', serialized)
        self.assertNotIn('privileged', service)
        self.assertIn('vision.launch.py', service['command'])
        self.assertIn('mode:=patrol', service['command'])
        healthcheck = service['healthcheck']['test'][-1]
        self.assertIn('person_detection_ready', healthcheck)
        self.assertIn('face_detection', healthcheck)
        self.assertIn('./models:/models:ro', service['volumes'])
        self.assertIn('./datasets:/datasets', service['volumes'])
        self.assertIn(
            'danger_minimum_confidence:=${DOGZILLA_DANGER_CONFIDENCE:-0.65}',
            service['command'],
        )
        self.assertIn(
            'danger_minimum_duration_seconds:=${DOGZILLA_DANGER_DURATION:-0.8}',
            service['command'],
        )
        self.assertIn(
            'danger_maximum_gap_seconds:=${DOGZILLA_DANGER_MAX_GAP:-1.5}',
            service['command'],
        )

        mission = (REPOSITORY / 'deploy' / 'dogzilla-mission').read_text()
        self.assertIn('start_perception', mission)
        self.assertIn("'Floor-hazard perception'", mission)
        stop_body = mission.split('stop_components() {', 1)[1].split('\n}', 1)[0]
        self.assertIn('stop --timeout 20 perception', stop_body)
        self.assertIn('/vision/danger_confirmed', mission)

    def test_nav2_costmaps_enforce_transient_keepout_filter(self):
        configuration = yaml.safe_load(
            (
                REPOSITORY
                / 'ros2'
                / 'dogzilla_slam'
                / 'config'
                / 'nav2_test1.yaml'
            ).read_text()
        )
        for costmap_name in ('local_costmap', 'global_costmap'):
            parameters = configuration[costmap_name][costmap_name][
                'ros__parameters'
            ]
            self.assertEqual(
                parameters['filters'],
                ['keepout_filter', 'keepout_inflation'],
            )
            self.assertEqual(
                parameters['keepout_filter']['plugin'],
                'nav2_costmap_2d::KeepoutFilter',
            )
            self.assertEqual(
                parameters['keepout_filter']['filter_info_topic'],
                '/keepout_filter_info',
            )
            self.assertEqual(
                parameters['keepout_inflation']['plugin'],
                'nav2_costmap_2d::InflationLayer',
            )

        local_parameters = configuration['local_costmap']['local_costmap'][
            'ros__parameters'
        ]
        global_parameters = configuration['global_costmap']['global_costmap'][
            'ros__parameters'
        ]
        self.assertEqual(
            local_parameters['keepout_inflation']['inflation_radius'],
            0.45,
        )
        self.assertEqual(
            global_parameters['keepout_inflation']['inflation_radius'],
            0.50,
        )

    def test_armed_vision_control_has_only_camera_and_base_serial(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        service = compose['services']['vision_control']
        self.assertEqual(
            service['devices'],
            [
                '/dev/ttyAMA0:/dev/ttyAMA0',
                '/dev/video0:/dev/video0',
            ],
        )
        serialized = yaml.safe_dump(service)
        self.assertNotIn('/dev/ttyAMA1', serialized)
        self.assertNotIn('privileged', service)
        self.assertIn('vision_control.launch.py', service['command'])
        self.assertIn('armed:=true', service['command'])
        self.assertEqual(service['restart'], 'no')

    def test_vision_control_launch_is_disarmed_by_default(self):
        launch = (
            REPOSITORY
            / 'ros2'
            / 'dogzilla_slam'
            / 'launch'
            / 'vision_control.launch.py'
        ).read_text()
        self.assertIn("'armed',\n            default_value='false'", launch)
        self.assertIn('condition=IfCondition(armed)', launch)
        self.assertIn("'accept_velocity_commands': False", launch)
        self.assertIn("'vision_control_enabled': True", launch)
        self.assertIn("'speed_level': 1", launch)
        self.assertIn("'turn_level': 1", launch)
        self.assertIn("'include_lidar': 'false'", launch)
        self.assertNotIn('/dev/ttyAMA1', launch)

    def test_visual_shadow_reuses_its_existing_camera(self):
        launch = (
            REPOSITORY
            / 'ros2'
            / 'dogzilla_slam'
            / 'launch'
            / 'visual_shadow.launch.py'
        ).read_text()
        self.assertIn("executable='vision_node'", launch)
        self.assertIn('condition=IfCondition(enabled)', launch)
        self.assertEqual(launch.count("mono_camera.launch.py"), 1)

    def test_operator_surface_offers_disarmed_proposal_modes(self):
        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        for mode in (
            'color-action',
            'watchdog',
            'qr-action',
            'line-follow',
        ):
            self.assertIn(mode, script)
        self.assertIn('Robot action output is disabled', script)

        core = (
            REPOSITORY
            / 'ros2'
            / 'dogzilla_slam'
            / 'dogzilla_slam'
            / 'vision_core.py'
        ).read_text()
        self.assertIn("'action_output': 'disabled'", core)
        self.assertIn("'executed': False", core)
        self.assertNotIn('DOGZILLALib', core)

    def test_operator_requires_typed_arming_and_has_safe_lifecycle(self):
        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        body = script.split('start_vision_control() {', 1)[1].split(
            '\n}\n',
            1,
        )[0]
        self.assertIn("Type ARM VISION to continue", body)
        self.assertIn("!= 'ARM VISION'", body)
        self.assertIn('pause_oled_service', body)
        self.assertIn('assert_serial_ports_free /dev/ttyAMA0', body)
        self.assertIn('assert_camera_free', body)
        self.assertIn('VISION_CONTROL_SERVICE_NAME', body)
        self.assertIn('restore_oled_service', body)
        self.assertNotIn('/dev/ttyAMA1', body)

        stop_body = script.split('stop_dogzilla() {', 1)[1].split(
            '\n}\n',
            1,
        )[0]
        self.assertIn('vision_control_is_running', stop_body)
        self.assertIn('VISION_CONTROL_SERVICE_NAME', stop_body)

    def test_teleop_operator_accepts_levels_one_through_nine(self):
        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        body = script.split('open_teleop() {', 1)[1].split(
            '\n}\n',
            1,
        )[0]

        self.assertIn('local speed_level="${1:-5}"', body)
        self.assertIn('[1-9])', body)
        self.assertIn('initial_level:=$1', body)
        self.assertIn('Press - to turn slower', body)
        self.assertIn('return to level 1 when teleop closes', body)
        self.assertIn('reset_motion_levels "${control_service}"', body)

    def test_mapping_low_posture_is_explicit_and_normal_remains_default(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        mapping_command = compose['services']['mapping']['command']
        self.assertIn(
            'body_height:=${DOGZILLA_BODY_HEIGHT:-105.0}',
            mapping_command,
        )
        self.assertIn(
            'apply_startup_body_height:='
            '${DOGZILLA_APPLY_STARTUP_BODY_HEIGHT:-false}',
            mapping_command,
        )

        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        options = script.split('configure_start_options() {', 1)[1].split(
            '\n}\n',
            1,
        )[0]
        self.assertIn("DOGZILLA_POSTURE_PROFILE='normal'", options)
        self.assertIn("DOGZILLA_BODY_HEIGHT='105.0'", options)
        self.assertIn("DOGZILLA_BODY_HEIGHT='75.0'", options)
        self.assertIn('DOGZILLA_APPLY_STARTUP_BODY_HEIGHT=true', options)
        start = script.split('start_mapping() {', 1)[1].split(
            '\n}\n',
            1,
        )[0]
        self.assertIn('ros2 param get /dogzilla_safe_base body_height', start)
        self.assertIn('Low-posture startup did not complete', start)

    def test_custom_object_workflow_is_camera_only_and_validated(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        vision = compose['services']['vision']
        self.assertIn('./datasets:/datasets', vision['volumes'])
        self.assertEqual(vision['devices'], ['/dev/video0:/dev/video0'])

        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        install = script.split(
            'install_custom_object_model() {',
            1,
        )[1].split('\n}\n', 1)[0]
        self.assertIn('--network none', install)
        self.assertIn('object_model_validate', install)
        self.assertIn('vision_device_is_running', install)
        capture = script.split(
            'capture_object_dataset() {',
            1,
        )[1].split('\n}\n', 1)[0]
        self.assertIn('python3 -m dogzilla_slam.dataset_capture', capture)
        self.assertIn('source /opt/ros/humble/setup.bash', capture)
        self.assertIn('source /root/yahboomcar_ws/install/setup.bash', capture)
        self.assertNotIn('VISION_CONTROL_SERVICE_NAME', capture)
        self.assertIn('local label="${1:-}"', capture)

        acceptance = script.split(
            'check_object_detection() {',
            1,
        )[1].split('\n}\n', 1)[0]
        self.assertIn('python3 -m dogzilla_slam.object_acceptance', acceptance)
        self.assertIn('Object acceptance refuses armed Vision Control', acceptance)
        self.assertIn('set_vision_mode "${mode}" red', acceptance)
        self.assertIn('--minimum-hits 3', acceptance)
        self.assertIn('source /opt/ros/humble/setup.bash', acceptance)
        self.assertIn('report_temporary', acceptance)
        self.assertIn('json.load', acceptance)

    def test_pretrained_yoloe_install_is_offline_validated_and_atomic(self):
        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        install = script.split(
            'install_yoloe_object_model() {',
            1,
        )[1].split('\n}\n', 1)[0]

        self.assertIn('--network none', install)
        self.assertIn('--model-format yoloe', install)
        self.assertIn('vision_device_is_running', install)
        self.assertIn('YOLOE_MODEL_FILE', install)
        self.assertIn('YOLOE_LABELS_FILE', install)
        self.assertIn('mv -f', install)
        self.assertIn('object-model-yoloe-install)', script)


if __name__ == '__main__':
    unittest.main()
