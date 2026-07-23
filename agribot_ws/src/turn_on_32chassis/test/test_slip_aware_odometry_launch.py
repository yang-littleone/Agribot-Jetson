import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry


@pytest.mark.launch_test
def generate_test_description():
    manager = launch_ros.actions.Node(
        package='turn_on_32chassis',
        executable='slip_aware_odometry_node',
        name='slip_aware_odometry_test',
        output='screen',
        parameters=[{
            'wheel_topic': '/test/wheel',
            'lio_input_topic': '/test/lio_raw',
            'lio_output_topic': '/test/lio',
            'validated_wheel_topic': '/test/wheel_validated',
            'fused_input_topic': '/test/fused',
            'final_odom_topic': '/test/final',
            'comparison_window': 0.1,
            'maximum_sync_delta': 0.15,
            'wheel_lio_max_age': 0.2,
            'lio_timeout': 0.3,
            'output_frequency': 50.0,
            'maximum_prediction_horizon': 0.15,
            'maximum_wheel_prediction_distance': 0.02,
            'maximum_lio_latency': 0.3,
            'stationary_hold_time': 0.15,
            'stationary_lio_speed_threshold': 0.05,
            'stationary_lio_release_speed': 0.10,
            'stationary_wheel_linear_threshold': 0.01,
            'stationary_wheel_angular_threshold': 0.02,
            'stationary_release_distance': 0.03,
            'instant_linear_residual_threshold': 0.1,
            'instant_linear_recovery_threshold': 0.05,
            'translation_residual_threshold': 0.015,
            'recovery_translation_threshold': 0.008,
            'minimum_observable_motion': 0.005,
            'slip_confirmation_time': 0.15,
            'wheel_recovery_time': 0.15,
            'wheel_recovery_ramp_time': 0.15,
        }],
    )
    return (
        launch.LaunchDescription([
            manager,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'manager': manager},
    )


class TestSlipAwareOdometry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('slip_aware_odometry_test_client')
        self.wheel_pub = self.node.create_publisher(
            Odometry, '/test/wheel', 20)
        self.lio_pub = self.node.create_publisher(
            Odometry, '/test/lio_raw', 20)
        self.fused_pub = self.node.create_publisher(
            Odometry, '/test/fused', 20)
        self.validated = []
        self.standardized_lio = []
        self.final = []
        self.node.create_subscription(
            Odometry, '/test/wheel_validated',
            lambda message: self.validated.append((time.monotonic(), message)),
            20)
        self.node.create_subscription(
            Odometry, '/test/lio',
            lambda message: self.standardized_lio.append(message), 20)
        self.node.create_subscription(
            Odometry, '/test/final',
            lambda message: self.final.append((time.monotonic(), message)), 20)
        self._wait_for_connections()

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _wait_for_connections(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (
                self.wheel_pub.get_subscription_count() > 0 and
                self.lio_pub.get_subscription_count() > 0 and
                self.fused_pub.get_subscription_count() > 0
            ):
                return
            self._spin_for(0.05)
        self.fail('slip-aware odometry subscriptions were not discovered')

    def _odom(self, stamp, x, vx=0.0):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'source_odom'
        message.child_frame_id = 'source_body'
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.001
        message.pose.covariance[7] = 0.001
        message.pose.covariance[35] = 0.01
        message.twist.twist.linear.x = vx
        message.twist.covariance[0] = 0.01
        message.twist.covariance[7] = 0.01
        message.twist.covariance[35] = 0.02
        return message

    def _publish_triplet(self, wheel_x, lio_x, fused_x, vx=0.0):
        stamp = self.node.get_clock().now().to_msg()
        self.wheel_pub.publish(self._odom(stamp, wheel_x, vx))
        self.lio_pub.publish(self._odom(stamp, lio_x, vx))
        self.fused_pub.publish(self._odom(stamp, fused_x, vx))
        self._spin_for(0.05)

    def test_synthetic_slip_timeout_and_bad_lio(self):
        # Normal motion proves agreement, then the wheel is gradually admitted.
        position = 0.0
        for _ in range(24):
            position += 0.01
            self._publish_triplet(position, position, position, 0.2)
        self.assertGreater(
            len(self.validated), 0,
            'wheel odometry never recovered after sustained LIO agreement')

        # A fresh LIO position must drive final x/y immediately. The EKF
        # message deliberately retains its old x position to catch the former
        # "integrate first, pull back later" behavior.
        stale_fused_position = position
        position += 0.08
        self._publish_triplet(
            position, position, stale_fused_position, 0.2)
        expected_lio_x = self.standardized_lio[-1].pose.pose.position.x
        final_x = self.final[-1][1].pose.pose.position.x
        self.assertAlmostEqual(final_x, expected_lio_x, delta=0.02)
        self.assertGreater(
            abs(final_x - stale_fused_position), 0.04,
            'final odometry still waited for the EKF position correction')

        # A sudden wheel-only speed/pose jump must be blocked immediately by
        # the latest LIO velocity residual, before the displacement window has
        # enough samples to enter the persistent-slip state.
        for _ in range(5):
            self._publish_triplet(position, position, position, 0.0)

        stationary_outputs = []
        for index in range(8):
            lio_noise = 0.004 if index % 2 == 0 else -0.004
            self._publish_triplet(
                position, position + lio_noise, position, 0.0)
            stationary_outputs.append(
                self.final[-1][1].pose.pose.position.x)
        self.assertLessEqual(
            max(stationary_outputs[-4:]) -
            min(stationary_outputs[-4:]),
            0.002,
            'stationary FAST-LIO noise leaked into final odometry')

        final_before_instant_slip = self.final[-1][1].pose.pose.position.x
        wheel_position = position + 0.08
        instant_slip_stamp = self.node.get_clock().now().to_msg()
        self.wheel_pub.publish(
            self._odom(instant_slip_stamp, wheel_position, 0.5))
        self._spin_for(0.04)
        self.assertLessEqual(
            abs(
                self.final[-1][1].pose.pose.position.x -
                final_before_instant_slip),
            0.02,
            'wheel-only slip entered the final inter-frame prediction')

        # Out-of-order and NaN LIO samples must not reach the standardized topic.
        accepted_count = len(self.standardized_lio)
        old_stamp = self.standardized_lio[-1].header.stamp
        self.lio_pub.publish(self._odom(old_stamp, position))
        self._spin_for(0.08)
        self.assertEqual(len(self.standardized_lio), accepted_count)

        nan_message = self._odom(
            self.node.get_clock().now().to_msg(), position)
        nan_message.pose.pose.position.x = math.nan
        self.lio_pub.publish(nan_message)
        self._spin_for(0.08)
        self.assertEqual(len(self.standardized_lio), accepted_count)

        # The chassis reports forward motion while LIO and fused odometry stay
        # fixed. The validated wheel stream must be cut and final pose must not
        # integrate the wheel motion.
        fixed_position = position
        final_before_slip = self.final[-1][1].pose.pose.position.x
        for _ in range(12):
            wheel_position += 0.025
            self._publish_triplet(
                wheel_position, fixed_position, fixed_position, 0.0)
        validated_at_rejection = len(self.validated)
        for _ in range(5):
            wheel_position += 0.025
            self._publish_triplet(
                wheel_position, fixed_position, fixed_position, 0.0)
        self.assertLessEqual(
            len(self.validated) - validated_at_rejection, 1,
            'wheel samples continued after persistent slip was rejected')
        self.assertLessEqual(
            abs(self.final[-1][1].pose.pose.position.x - final_before_slip),
            0.02)

        # With LIO absent, short prediction is allowed, then pose freezes,
        # twist becomes zero, and covariance marks the output untrusted.
        predicted_position = fixed_position
        for _ in range(12):
            predicted_position += 0.02
            stamp = self.node.get_clock().now().to_msg()
            self.fused_pub.publish(
                self._odom(stamp, predicted_position, 0.4))
            self._spin_for(0.05)
        frozen = self.final[-1][1]
        self.assertAlmostEqual(frozen.twist.twist.linear.x, 0.0, places=6)
        self.assertGreaterEqual(frozen.pose.covariance[0], 1000.0)
        frozen_x = frozen.pose.pose.position.x
        self._spin_for(0.15)
        self.assertAlmostEqual(
            self.final[-1][1].pose.pose.position.x, frozen_x, places=6)

        # A recovered LIO stream is re-aligned to the frozen odom pose even
        # when its own local frame restarts far away.
        recovery_stamp = self.node.get_clock().now().to_msg()
        self.lio_pub.publish(self._odom(recovery_stamp, 10.0, 0.0))
        self.fused_pub.publish(
            self._odom(recovery_stamp, predicted_position, 0.0))
        self._spin_for(0.08)
        self.assertAlmostEqual(
            self.standardized_lio[-1].pose.pose.position.x,
            frozen_x,
            delta=0.01)
