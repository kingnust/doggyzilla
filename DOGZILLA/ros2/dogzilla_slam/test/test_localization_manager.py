import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from std_msgs.msg import Bool

from dogzilla_slam.localization_manager import LocalizationManager


class LocalizationManagerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_cancel_pauses_and_a_newer_initial_pose_resumes(self):
        node = LocalizationManager()
        try:
            first = PoseWithCovarianceStamped()
            first.header.frame_id = 'map'
            first.header.stamp = node.get_clock().now().to_msg()
            node._initial_pose_received(first)
            self.assertIsNotNone(node._pending_initial_pose)

            cancel = Bool()
            cancel.data = True
            node._cancel_received(cancel)
            self.assertTrue(node._paused)
            self.assertIsNone(node._pending_initial_pose)

            node._initial_pose_received(first)
            self.assertTrue(node._paused)
            self.assertIsNone(node._pending_initial_pose)

            resume_stamp = node._pause_stamp_ns + 1
            replacement = PoseWithCovarianceStamped()
            replacement.header.frame_id = 'map'
            replacement.header.stamp.sec = resume_stamp // 1_000_000_000
            replacement.header.stamp.nanosec = resume_stamp % 1_000_000_000
            node._initial_pose_received(replacement)
            self.assertFalse(node._paused)
            self.assertIsNotNone(node._pending_initial_pose)

            node._pending_initial_pose = None
            node._paused = True
            node._active_trajectory_id = 7
            node._services_ready = Mock(return_value=True)
            node._finish_active_trajectory = Mock()
            node._tick()
            node._finish_active_trajectory.assert_called_once_with()
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
