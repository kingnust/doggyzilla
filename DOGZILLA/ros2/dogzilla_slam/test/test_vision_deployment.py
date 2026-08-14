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
        self.assertNotIn('devices', compose['services']['web'])

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
        self.assertIn("'speed_profile': 'slow'", launch)
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


if __name__ == '__main__':
    unittest.main()
