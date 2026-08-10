"""Safe, single-owner DOGZILLA movement and raw-IMU serial bridge."""

import DOGZILLALib as dog
from geometry_msgs.msg import Twist
import math
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, JointState

from .firmware_rest_capture import FirmwareRestRecorder
from .firmware_rest_capture import save_capture_atomic


class SafeBase(Node):
    """Clamp velocity commands and stop the controller after a short timeout."""

    JOINT_NAMES = tuple(
        f'leg{leg}_motor{motor}_joint'
        for leg in range(1, 5)
        for motor in range(1, 4)
    )

    # These reproduce Yahboom's mobile-app step-width scale. The app maps
    # slow/minimum to controller step 4, normal/default to step 10, and
    # high/maximum to step 20. ROS values are divided by controller_scale.
    SPEED_PROFILES = {
        'slow': (0.10, 0.30, 'slow', 4),
        'normal': (0.25, 1.125, 'normal', 10),
        'high': (0.50, 1.75, 'high', 20),
    }
    POSTURE_LIMITS = {
        'body_height': (75.0, 110.0),
        'head_pitch': (-15.0, 15.0),
        'head_yaw': (-11.0, 11.0),
    }

    def __init__(self):
        super().__init__('dogzilla_safe_base')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.30)
        self.declare_parameter('command_timeout', 0.60)
        self.declare_parameter('controller_scale', 40.0)
        self.declare_parameter('speed_profile', 'slow')
        self.declare_parameter('publish_imu', False)
        self.declare_parameter('raw_imu_topic', '/imu/data_uncalibrated')
        self.declare_parameter('raw_imu_frame', 'imu_link_raw')
        self.declare_parameter('imu_rate_hz', 20.0)
        self.declare_parameter('serial_read_timeout', 0.08)
        self.declare_parameter('publish_battery', True)
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('battery_rate_hz', 1.0)
        self.declare_parameter('low_battery_percent', 25)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_state_rate_hz', 1.0)
        self.declare_parameter('capture_firmware_rest', True)
        self.declare_parameter(
            'firmware_rest_capture_directory',
            '/profiles/captures',
        )
        self.declare_parameter('firmware_rest_arm_margin_percent', 5)
        self.declare_parameter('firmware_rest_capture_joint_rate_hz', 5.0)
        self.declare_parameter('posture_control_enabled', False)
        self.declare_parameter('body_height', 105.0)
        self.declare_parameter('head_pitch', 0.0)
        self.declare_parameter('head_yaw', 0.0)

        input_topic = self.get_parameter('input_topic').value
        self._max_linear = float(self.get_parameter('max_linear').value)
        self._max_angular = float(self.get_parameter('max_angular').value)
        self._command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self._controller_scale = float(
            self.get_parameter('controller_scale').value
        )
        self._publish_imu_enabled = bool(
            self.get_parameter('publish_imu').value
        )
        self._posture_control_enabled = bool(
            self.get_parameter('posture_control_enabled').value
        )
        self._body_height = float(self.get_parameter('body_height').value)
        self._head_pitch = float(self.get_parameter('head_pitch').value)
        self._head_yaw = float(self.get_parameter('head_yaw').value)
        self._low_battery_percent = int(
            self.get_parameter('low_battery_percent').value
        )
        self._movement_inhibited = False
        self._battery_percent = None
        self._battery_failures = 0
        self._joint_failures = 0
        self._joint_rate_hz_normal = float(
            self.get_parameter('joint_state_rate_hz').value
        )
        if not 0.10 <= self._joint_rate_hz_normal <= 5.0:
            raise ValueError('joint_state_rate_hz must be between 0.10 and 5 Hz')
        self._joint_rate_hz_capture = float(
            self.get_parameter('firmware_rest_capture_joint_rate_hz').value
        )
        if not self._joint_rate_hz_normal <= self._joint_rate_hz_capture <= 5.0:
            raise ValueError(
                'firmware_rest_capture_joint_rate_hz must be between the '
                'normal joint rate and 5 Hz'
            )
        self._joint_rate_hz_current = self._joint_rate_hz_normal

        self._firmware_rest_recorder = None
        if bool(self.get_parameter('capture_firmware_rest').value):
            capture_directory = str(
                self.get_parameter(
                    'firmware_rest_capture_directory'
                ).value
            )
            arm_margin = int(
                self.get_parameter(
                    'firmware_rest_arm_margin_percent'
                ).value
            )
            self._firmware_rest_recorder = FirmwareRestRecorder(
                joint_names=self.JOINT_NAMES,
                low_battery_percent=self._low_battery_percent,
                arm_margin_percent=arm_margin,
                save_callback=lambda payload: save_capture_atomic(
                    payload,
                    capture_directory,
                ),
            )

        # This node is the only /dev/ttyAMA0 owner during mapping.
        self._dog = dog.DOGZILLA()
        self._bound_vendor_read_timeout(
            float(self.get_parameter('serial_read_timeout').value)
        )
        self._dog.stop()
        initial_speed_profile = self.get_parameter('speed_profile').value
        self._set_speed_profile(initial_speed_profile, announce=False)
        self.add_on_set_parameters_callback(self._parameters_changed)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._apply_command,
            10,
        )
        self._last_command_time = None
        self._stopped = True
        self._timer = self.create_timer(0.10, self._watchdog)

        self._battery_publisher = None
        self._battery_timer = None
        self._joint_publisher = None
        self._joint_timer = None
        publish_battery = bool(self.get_parameter('publish_battery').value)
        if publish_battery or self._firmware_rest_recorder is not None:
            battery_rate_hz = float(
                self.get_parameter('battery_rate_hz').value
            )
            if not 0.05 <= battery_rate_hz <= 2.0:
                raise ValueError('battery_rate_hz must be between 0.05 and 2 Hz')
            if publish_battery:
                self._battery_publisher = self.create_publisher(
                    BatteryState,
                    self.get_parameter('battery_topic').value,
                    qos_profile_sensor_data,
                )
            self._battery_timer = self.create_timer(
                1.0 / battery_rate_hz,
                self._publish_battery,
            )
            # Check before the executor can accept its first movement command.
            self._publish_battery()

        publish_joint_states = bool(
            self.get_parameter('publish_joint_states').value
        )
        if publish_joint_states or self._firmware_rest_recorder is not None:
            if publish_joint_states:
                self._joint_publisher = self.create_publisher(
                    JointState,
                    self.get_parameter('joint_state_topic').value,
                    qos_profile_sensor_data,
                )
            self._joint_timer = self.create_timer(
                1.0 / self._joint_rate_hz_current,
                self._publish_joint_states,
            )

        self._imu_publisher = None
        self._imu_timer = None
        self._imu_failures = 0
        if self._publish_imu_enabled:
            raw_imu_topic = self.get_parameter('raw_imu_topic').value
            self._raw_imu_frame = self.get_parameter('raw_imu_frame').value
            imu_rate_hz = float(self.get_parameter('imu_rate_hz').value)
            if not 5.0 <= imu_rate_hz <= 50.0:
                raise ValueError('imu_rate_hz must be between 5 and 50 Hz')
            self._imu_publisher = self.create_publisher(
                Imu,
                raw_imu_topic,
                qos_profile_sensor_data,
            )
            self._imu_timer = self.create_timer(
                1.0 / imu_rate_hz,
                self._publish_raw_imu,
            )
            self.get_logger().info(
                f'Uncalibrated IMU enabled at {imu_rate_hz:.1f} Hz on '
                f'{raw_imu_topic}'
            )

        self.get_logger().info(
            'Safe base active: '
            f'profile = {self._speed_profile}, '
            f'linear <= {self._max_linear:.2f}, '
            f'angular <= {self._max_angular:.2f}, '
            f'timeout = {self._command_timeout:.2f}s, '
            f'posture controls = {self._posture_control_enabled}'
        )
        if self._firmware_rest_recorder is not None:
            self.get_logger().info(
                'Passive firmware-rest capture enabled: normal joint rate '
                f'{self._joint_rate_hz_normal:.1f} Hz, near-low rate '
                f'{self._joint_rate_hz_capture:.1f} Hz; replay remains disabled'
            )

    def _set_speed_profile(self, profile, announce=True):
        """Apply a Yahboom-compatible pace and velocity ceiling."""
        if profile not in self.SPEED_PROFILES:
            raise ValueError('speed_profile must be slow, normal, or high')
        max_linear, max_angular, controller_pace, controller_step = (
            self.SPEED_PROFILES[profile]
        )
        self._dog.pace(controller_pace)
        self._speed_profile = profile
        self._max_linear = max_linear
        self._max_angular = max_angular
        if announce:
            self.get_logger().info(
                f'Speed profile changed to {profile}: '
                f'app-equivalent step {controller_step}, '
                f'linear <= {max_linear:.3f}, '
                f'angular <= {max_angular:.3f}'
            )

    def _parameters_changed(self, parameters):
        """Apply bounded speed and posture changes through the serial owner."""
        requested = {}
        for parameter in parameters:
            if parameter.name == 'speed_profile':
                profile = str(parameter.value)
                if profile not in self.SPEED_PROFILES:
                    return SetParametersResult(
                        successful=False,
                        reason='speed_profile must be slow, normal, or high',
                    )
                requested[parameter.name] = profile
            elif parameter.name in self.POSTURE_LIMITS:
                if not self._posture_control_enabled:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'posture control is disabled while LiDAR '
                            'mapping/localization is active'
                        ),
                    )
                if self._movement_inhibited:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'posture control is blocked by the low-battery '
                            'movement lockout; charge the robot first'
                        ),
                    )
                value = float(parameter.value)
                lower, upper = self.POSTURE_LIMITS[parameter.name]
                if not lower <= value <= upper:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f'{parameter.name} must be between '
                            f'{lower:.0f} and {upper:.0f}'
                        ),
                    )
                requested[parameter.name] = value

        if not requested:
            return SetParametersResult(successful=True)

        self.stop()
        try:
            if 'speed_profile' in requested:
                self._set_speed_profile(requested['speed_profile'])
            if 'body_height' in requested:
                value = requested['body_height']
                self._dog.translation('z', value)
                self._body_height = value
            if 'head_pitch' in requested:
                value = requested['head_pitch']
                self._dog.attitude('p', value)
                self._head_pitch = value
            if 'head_yaw' in requested:
                value = requested['head_yaw']
                self._dog.attitude('y', value)
                self._head_yaw = value
        except Exception as exc:
            self.stop()
            return SetParametersResult(
                successful=False,
                reason=f'controller rejected parameter change: {exc}',
            )

        if any(name in requested for name in self.POSTURE_LIMITS):
            self.get_logger().info(
                'Posture changed: '
                f'height={self._body_height:.0f}, '
                f'pitch={self._head_pitch:.0f}, '
                f'yaw={self._head_yaw:.0f}'
            )
        return SetParametersResult(successful=True)

    def _bound_vendor_read_timeout(self, timeout_s):
        """Limit Yahboom's private one-second serial busy wait."""
        if not 0.01 <= timeout_s <= 0.20:
            raise ValueError('serial_read_timeout must be between 0.01 and 0.20s')
        original_unpack = getattr(self._dog, '_DOGZILLA__unpack')

        def bounded_unpack(timeout=timeout_s):
            return original_unpack(timeout=min(float(timeout), timeout_s))

        setattr(self._dog, '_DOGZILLA__unpack', bounded_unpack)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def stop(self):
        """Stop movement directly through the serial controller."""
        try:
            self._dog.stop()
        except Exception as exc:  # Hardware errors must not block shutdown.
            if rclpy.ok():
                self.get_logger().error(f'Failed to stop DOGZILLA: {exc}')
        self._stopped = True

    def _apply_command(self, source):
        if self._movement_inhibited:
            return

        linear_x = self._clamp(source.linear.x, self._max_linear)
        linear_y = self._clamp(source.linear.y, self._max_linear)
        angular_z = self._clamp(source.angular.z, self._max_angular)

        self._last_command_time = self.get_clock().now()
        if (
            abs(linear_x) < 0.01
            and abs(linear_y) < 0.01
            and abs(angular_z) < 0.01
        ):
            self.stop()
            return

        self._dog.move('x', linear_x * self._controller_scale)
        self._dog.move('y', linear_y * self._controller_scale)
        self._dog.turn(angular_z * self._controller_scale)
        self._stopped = False

    def _watchdog(self):
        if self._last_command_time is None or self._stopped:
            return

        elapsed = (
            self.get_clock().now() - self._last_command_time
        ).nanoseconds / 1e9
        if elapsed <= self._command_timeout:
            return

        self.get_logger().warn('Command timeout: stopping DOGZILLA')
        self.stop()

    def _publish_battery(self):
        """Publish battery percentage and cooperate with firmware low power."""
        try:
            battery = int(self._dog.read_battery())
        except Exception as exc:
            battery = 0
            if self._battery_failures == 0:
                self.get_logger().error(f'Battery serial read error: {exc}')
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.voltage = math.nan
        message.temperature = math.nan
        message.current = math.nan
        message.charge = math.nan
        message.capacity = math.nan
        message.design_capacity = math.nan
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        message.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )

        if not 1 <= battery <= 100:
            self._battery_failures += 1
            message.percentage = math.nan
            message.present = False
            if self._battery_failures == 1 or self._battery_failures % 12 == 0:
                self.get_logger().warn(
                    'Battery telemetry read failed '
                    f'({self._battery_failures} failures)'
                )
            if self._battery_publisher is not None:
                self._battery_publisher.publish(message)
            return

        self._battery_failures = 0
        self._battery_percent = battery
        message.percentage = battery / 100.0
        message.present = True
        if self._battery_publisher is not None:
            self._battery_publisher.publish(message)

        if self._firmware_rest_recorder is not None:
            self._firmware_rest_recorder.observe_battery(battery)
            self._report_firmware_rest_capture_events()
            self._update_joint_capture_rate()

        if battery <= self._low_battery_percent:
            if not self._movement_inhibited:
                self.stop()
                self._movement_inhibited = True
                self.get_logger().error(
                    f'Battery {battery}% <= {self._low_battery_percent}%: '
                    'ROS movement inhibited; Yahboom low-battery rest wins'
                )
        elif self._movement_inhibited and battery >= (
            self._low_battery_percent + 3
        ):
            self._movement_inhibited = False
            self.get_logger().info(
                f'Battery recovered to {battery}%; ROS movement enabled'
            )

    def _publish_joint_states(self):
        """Publish all motor angles reported by the controller in radians."""
        try:
            angles = self._dog.read_motor()
        except Exception as exc:
            angles = []
            if self._joint_failures == 0:
                self.get_logger().error(f'Motor serial read error: {exc}')
        if len(angles) != len(self.JOINT_NAMES):
            self._joint_failures += 1
            if self._joint_failures == 1 or self._joint_failures % 10 == 0:
                self.get_logger().warn(
                    'Motor-angle telemetry read failed '
                    f'({self._joint_failures} failures)'
                )
            return

        self._joint_failures = 0
        if self._firmware_rest_recorder is not None:
            self._firmware_rest_recorder.observe_joints(angles)
            self._report_firmware_rest_capture_events()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.JOINT_NAMES)
        message.position = [math.radians(float(angle)) for angle in angles]
        if self._joint_publisher is not None:
            self._joint_publisher.publish(message)

    def _report_firmware_rest_capture_events(self):
        if self._firmware_rest_recorder is None:
            return
        logger = self.get_logger()
        for level, message in self._firmware_rest_recorder.take_events():
            if level == 'error':
                logger.error(message)
            elif level == 'warning':
                logger.warn(message)
            else:
                logger.info(message)

    def _update_joint_capture_rate(self):
        if self._firmware_rest_recorder is None:
            return
        target_rate = (
            self._joint_rate_hz_capture
            if self._firmware_rest_recorder.wants_high_joint_rate
            else self._joint_rate_hz_normal
        )
        if abs(target_rate - self._joint_rate_hz_current) < 1e-9:
            return
        self._joint_rate_hz_current = target_rate
        if self._joint_timer is not None:
            self._joint_timer.cancel()
            self._joint_timer = self.create_timer(
                1.0 / target_rate,
                self._publish_joint_states,
            )
        self.get_logger().info(
            f'Joint telemetry rate changed to {target_rate:.1f} Hz for '
            'passive firmware-rest capture.'
        )

    def _publish_raw_imu(self):
        """Read the controller IMU without opening a second serial owner."""
        raw = self._dog.read_imu_raw()
        if len(raw) < 6 or not any(abs(value) > 1e-9 for value in raw[:6]):
            self._imu_failures += 1
            if self._imu_failures == 1 or self._imu_failures % 20 == 0:
                self.get_logger().warn(
                    f'IMU serial read failed ({self._imu_failures} failures)'
                )
            if self._imu_failures == 10:
                self._imu_timer.cancel()
                self.get_logger().error(
                    'Disabling IMU reads after 10 consecutive failures; '
                    'movement watchdog remains active'
                )
            return

        self._imu_failures = 0
        message = Imu()
        # The packet has no hardware clock, so stamp it immediately after the
        # complete packet is received. imu_validate checks monotonicity/gaps.
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._raw_imu_frame

        # Yahboom reports gyro values in degrees/s. Convert at the hardware
        # boundary so every ROS Imu message uses the required radians/s.
        message.angular_velocity.x = math.radians(float(raw[3]))
        message.angular_velocity.y = math.radians(float(raw[4]))
        message.angular_velocity.z = math.radians(float(raw[5]))
        message.linear_acceleration.x = float(raw[0])
        message.linear_acceleration.y = float(raw[1])
        message.linear_acceleration.z = float(raw[2])

        # The controller's fused RPY world convention is undocumented. Mark
        # orientation unavailable rather than publishing a misleading value.
        message.orientation_covariance[0] = -1.0
        self._imu_publisher.publish(message)

    def close(self):
        """Stop motion and release the single serial handle."""
        self.stop()
        if self._dog.ser.is_open:
            self._dog.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = SafeBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # This is a direct serial command, so it still works after the ROS
        # context has begun shutting down.
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
