"""Manage Cartographer pure-localization trajectories and RViz initial poses."""

from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory
from cartographer_ros_msgs.srv import GetTrajectoryStates
from cartographer_ros_msgs.srv import StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class LocalizationManager(Node):
    """Start global localization and honor RViz's 2D Pose Estimate tool."""

    def __init__(self):
        super().__init__('dogzilla_localization_manager')
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter(
            'configuration_basename',
            'dogzilla_localization.lua',
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('start_immediately', True)

        self._configuration_directory = self.get_parameter(
            'configuration_directory'
        ).value
        self._configuration_basename = self.get_parameter(
            'configuration_basename'
        ).value
        self._map_frame = self.get_parameter('map_frame').value
        self._start_immediately = bool(
            self.get_parameter('start_immediately').value
        )

        self._start_client = self.create_client(
            StartTrajectory,
            '/start_trajectory',
        )
        self._finish_client = self.create_client(
            FinishTrajectory,
            '/finish_trajectory',
        )
        self._states_client = self.create_client(
            GetTrajectoryStates,
            '/get_trajectory_states',
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_received,
            10,
        )

        self._active_trajectory_id = None
        self._frozen_trajectory_id = None
        self._pending_initial_pose = None
        self._busy = False
        self._waiting_logged = False
        self._initial_pose_wait_logged = False
        self._timer = self.create_timer(0.50, self._tick)

    def _services_ready(self):
        return (
            self._start_client.service_is_ready()
            and self._finish_client.service_is_ready()
            and self._states_client.service_is_ready()
        )

    def _tick(self):
        if self._busy or not self._services_ready():
            if not self._waiting_logged:
                self.get_logger().info(
                    'Waiting for Cartographer localization services'
                )
                self._waiting_logged = True
            return
        self._waiting_logged = False

        if (
            self._active_trajectory_id is not None
            and self._pending_initial_pose is not None
        ):
            self._finish_active_trajectory()
            return
        if self._active_trajectory_id is not None:
            return
        if not self._start_immediately and self._pending_initial_pose is None:
            if not self._initial_pose_wait_logged:
                self.get_logger().info(
                    'Waiting for an initial map pose before scan matching'
                )
                self._initial_pose_wait_logged = True
            return

        self._busy = True
        future = self._states_client.call_async(GetTrajectoryStates.Request())
        future.add_done_callback(self._trajectory_states_received)

    def _trajectory_states_received(self, future):
        self._busy = False
        response = future.result()
        if response is None or response.status.code != 0:
            message = response.status.message if response else 'no response'
            self.get_logger().error(
                f'Cannot read Cartographer trajectory states: {message}'
            )
            return

        frozen = [
            trajectory_id
            for trajectory_id, state in zip(
                response.trajectory_states.trajectory_id,
                response.trajectory_states.trajectory_state,
            )
            if state == TrajectoryStates.FROZEN
        ]
        if not frozen:
            self.get_logger().warn(
                'The PBStream has not exposed a frozen trajectory yet'
            )
            return
        self._frozen_trajectory_id = min(frozen)
        self._start_localization_trajectory()

    def _start_localization_trajectory(self):
        initial_pose = self._pending_initial_pose
        self._pending_initial_pose = None

        request = StartTrajectory.Request()
        request.configuration_directory = self._configuration_directory
        request.configuration_basename = self._configuration_basename
        request.use_initial_pose = initial_pose is not None
        request.relative_to_trajectory_id = self._frozen_trajectory_id
        if initial_pose is not None:
            request.initial_pose = initial_pose

        self._busy = True
        future = self._start_client.call_async(request)
        future.add_done_callback(self._trajectory_started)

    def _trajectory_started(self, future):
        self._busy = False
        response = future.result()
        if response is None or response.status.code != 0:
            message = response.status.message if response else 'no response'
            self.get_logger().error(
                f'Cannot start localization trajectory: {message}'
            )
            return
        self._active_trajectory_id = response.trajectory_id
        self.get_logger().info(
            'Cartographer localization trajectory '
            f'{self._active_trajectory_id} is active against frozen '
            f'trajectory {self._frozen_trajectory_id}'
        )

    def _finish_active_trajectory(self):
        request = FinishTrajectory.Request()
        request.trajectory_id = self._active_trajectory_id
        self._busy = True
        future = self._finish_client.call_async(request)
        future.add_done_callback(self._trajectory_finished)

    def _trajectory_finished(self, future):
        self._busy = False
        response = future.result()
        if response is None or response.status.code != 0:
            message = response.status.message if response else 'no response'
            self.get_logger().error(
                f'Cannot restart localization trajectory: {message}'
            )
            return
        self.get_logger().info(
            f'Finished localization trajectory {self._active_trajectory_id}'
        )
        self._active_trajectory_id = None

    def _initial_pose_received(self, message):
        if message.header.frame_id not in ('', self._map_frame):
            self.get_logger().error(
                'Initial pose must be expressed in the map frame; received '
                f'{message.header.frame_id}'
            )
            return
        self._pending_initial_pose = message.pose.pose
        self._initial_pose_wait_logged = False
        self.get_logger().info(
            'RViz initial pose received; localization will restart from it'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
