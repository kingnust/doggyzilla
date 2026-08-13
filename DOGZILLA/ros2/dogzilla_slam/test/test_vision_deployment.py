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

    def test_web_gateway_has_no_devices(self):
        compose = yaml.safe_load(
            (REPOSITORY / 'deploy' / 'compose.yaml').read_text()
        )
        self.assertNotIn('devices', compose['services']['web'])

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

    def test_operator_surface_does_not_offer_action_modes(self):
        script = (REPOSITORY / 'deploy' / 'dogzilla-map').read_text()
        for mode in ('color-action', 'qr-action', 'crossing', 'teach'):
            self.assertNotIn(f'raw|{mode}|', script)
        self.assertIn('Robot action output is disabled', script)


if __name__ == '__main__':
    unittest.main()
