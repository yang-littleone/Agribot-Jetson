#!/usr/bin/env python3
"""Publish two synthetic crop-row centerlines around one real PID U-turn."""

import math
from enum import Enum

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, MultiArrayDimension, String


class Stage(Enum):
    WAITING_FOR_ODOM = 'WAITING_FOR_ODOM'
    START_DELAY = 'START_DELAY'
    FIRST_ROW = 'FIRST_ROW'
    HEADLAND = 'HEADLAND'
    U_TURN = 'U_TURN'
    SECOND_ROW = 'SECOND_ROW'
    FINISHED = 'FINISHED'


class IndoorHeadlandSimulator(Node):
    DIAGNOSTIC_NAMES = (
        'valid', 'left_points', 'right_points', 'corridor_width_m',
        'safety_margin_m', 'published_confidence', 'row_yaw_rad',
        'center_offset_m', 'left_slope', 'left_intercept', 'right_slope',
        'right_intercept', 'line_lost_count', 'path_points', 'support_score',
        'observation_score', 'width_score', 'residual_score', 'safety_score',
        'observed_valid_ratio', 'raw_confidence', 'error_budget_m',
        'support_weight', 'observation_weight', 'width_weight',
        'residual_weight', 'safety_weight', 'innermost_enabled',
        'robust_enabled', 'temporal_enabled', 'parallel_enabled',
        'left_longitudinal_coverage', 'right_longitudinal_coverage',
    )

    def __init__(self):
        # This name lets the existing trial logger obtain a detector snapshot.
        super().__init__('corn_row_detector_projection')
        self.first_length = max(
            0.30, float(self.declare_parameter('first_line_length', 1.0).value))
        self.second_length = max(
            0.30, float(self.declare_parameter('second_line_length', 1.0).value))
        self.row_spacing = max(
            0.30, float(self.declare_parameter('row_spacing', 0.60).value))
        self.path_step = max(
            0.02, float(self.declare_parameter('path_step', 0.05).value))
        self.publish_period = max(
            0.02, float(self.declare_parameter('publish_period', 0.10).value))
        self.start_delay = max(
            0.0, float(self.declare_parameter('start_delay', 5.0).value))
        direction = str(self.declare_parameter('turn_direction', 'left').value)
        self.turn_sign = -1.0 if direction == 'right' else 1.0
        self.valid_confidence = min(1.0, max(
            0.75, float(self.declare_parameter('valid_confidence', 0.95).value)))
        self.invalid_confidence = min(0.25, max(
            0.0, float(self.declare_parameter('invalid_confidence', 0.10).value)))
        self.safety_margin = max(
            0.16, float(self.declare_parameter('safety_margin', 0.20).value))

        # Parameters read by field_trial_logger as the detector snapshot.
        self.declare_parameter('point_cloud_topic', 'indoor_simulation')
        self.declare_parameter('platform_width', 0.22)
        self.declare_parameter('desired_row_separation', self.row_spacing)
        self.declare_parameter('plant_safety_clearance', 0.05)
        self.declare_parameter('min_row_separation', 0.30)
        self.declare_parameter('max_row_separation', 1.50)
        self.declare_parameter('enable_innermost_row_extraction', True)
        self.declare_parameter('enable_robust_refinement', True)
        self.declare_parameter('use_parallel_row_model', True)
        self.declare_parameter('enable_temporal_tracking', True)
        self.declare_parameter('enable_quality_evaluation', True)

        self.path_pub = self.create_publisher(Path, '/corn_row_center_line', 10)
        self.path_viz_pub = self.create_publisher(
            Path, '/corn_row_center_line_viz', 10)
        self.left_pub = self.create_publisher(
            Path, '/under_canopy_left_boundary', 10)
        self.right_pub = self.create_publisher(
            Path, '/under_canopy_right_boundary', 10)
        self.width_pub = self.create_publisher(Float32, '/corridor_width', 10)
        self.margin_pub = self.create_publisher(
            Float32, '/corridor_safety_margin', 10)
        self.budget_pub = self.create_publisher(
            Float32, '/corridor_error_budget', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/corridor_confidence', 10)
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray, '/centerline_detection_diagnostics', 10)
        self.headland_pub = self.create_publisher(
            Bool, '/headland_detected', 10)
        self.stage_pub = self.create_publisher(
            String, '/indoor_test_stage', 10)
        self.finished_pub = self.create_publisher(
            Bool, '/indoor_test_finished', 10)

        self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 20)
        self.create_subscription(
            String, '/navigation_mode', self.mode_callback, 20)
        self.timer = self.create_timer(self.publish_period, self.publish)

        self.stage = Stage.WAITING_FOR_ODOM
        self.mode = ''
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self.anchor_time = None
        self.second_start_s = None
        self.first_path = Path()
        self.second_path = Path()
        self.saw_uturn = False
        self.saw_reacquire = False
        self.last_logged_stage = None

        self.get_logger().info(
            '室内地头模拟器等待 /odometry/filtered；不要同时启动真实玉米行检测器')

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def quaternion_from_yaw(yaw):
        half = 0.5 * yaw
        from geometry_msgs.msg import Quaternion
        return Quaternion(z=math.sin(half), w=math.cos(half))

    def odom_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        if self.have_odom:
            return
        self.have_odom = True
        self.origin_x = self.x
        self.origin_y = self.y
        self.origin_yaw = self.yaw
        self.anchor_time = self.get_clock().now()
        self.set_stage(Stage.START_DELAY)
        self.get_logger().info(
            f'已收到初始位姿；{self.start_delay:.1f}s后按最新静止位姿固定路径并开始，'
            f'第一行={self.first_length:.2f}m，第二行={self.second_length:.2f}m，'
            f'行距={self.row_spacing:.2f}m')

    def mode_callback(self, msg):
        self.mode = msg.data.strip()
        if self.mode == 'U_TURN':
            self.saw_uturn = True
            if self.stage in {Stage.HEADLAND, Stage.FIRST_ROW}:
                self.set_stage(Stage.U_TURN)
        elif self.mode == 'NEXT_ROW_REACQUIRE' and self.saw_uturn:
            self.saw_reacquire = True
            if self.second_start_s is None:
                self.second_start_s = self.longitudinal_station()
                self.second_path = self.build_line(
                    start_s=self.second_start_s,
                    lateral=self.turn_sign * self.row_spacing,
                    length=self.second_length,
                    travel_sign=-1.0)
            self.set_stage(Stage.SECOND_ROW)

    def set_stage(self, stage):
        if self.stage == stage:
            return
        self.stage = stage
        self.get_logger().warn(f'室内模拟阶段切换: {stage.value}')

    def longitudinal_station(self):
        dx = self.x - self.origin_x
        dy = self.y - self.origin_y
        return dx * math.cos(self.origin_yaw) + dy * math.sin(self.origin_yaw)

    def build_line(self, start_s, lateral, length, travel_sign):
        path = Path()
        path.header.frame_id = 'odom'
        samples = max(1, int(math.ceil(length / self.path_step)))
        tx = math.cos(self.origin_yaw)
        ty = math.sin(self.origin_yaw)
        lx = -math.sin(self.origin_yaw)
        ly = math.cos(self.origin_yaw)
        heading = self.origin_yaw if travel_sign > 0.0 else self.origin_yaw + math.pi
        for index in range(samples + 1):
            distance = min(length, index * self.path_step)
            station = start_s + travel_sign * distance
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.origin_x + tx * station + lx * lateral
            pose.pose.position.y = self.origin_y + ty * station + ly * lateral
            pose.pose.orientation = self.quaternion_from_yaw(heading)
            path.poses.append(pose)
        return path

    def shifted_boundary(self, path, offset):
        boundary = Path()
        boundary.header.frame_id = path.header.frame_id
        if not path.poses:
            return boundary
        heading = self.yaw_from_quaternion(path.poses[0].pose.orientation)
        nx = -math.sin(heading)
        ny = math.cos(heading)
        for source in path.poses:
            pose = PoseStamped()
            pose.header.frame_id = source.header.frame_id
            pose.pose.position.x = source.pose.position.x + nx * offset
            pose.pose.position.y = source.pose.position.y + ny * offset
            pose.pose.position.z = source.pose.position.z
            pose.pose.orientation.x = source.pose.orientation.x
            pose.pose.orientation.y = source.pose.orientation.y
            pose.pose.orientation.z = source.pose.orientation.z
            pose.pose.orientation.w = source.pose.orientation.w
            boundary.poses.append(pose)
        return boundary

    def stamped(self, path):
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header.stamp = stamp
        return path

    def publish_quality(self, valid, path, confidence):
        width = Float32(data=float(self.row_spacing))
        margin = Float32(data=float(self.safety_margin))
        budget = Float32(data=float(self.safety_margin))
        confidence_msg = Float32(data=float(confidence))
        self.width_pub.publish(width)
        self.margin_pub.publish(margin)
        self.budget_pub.publish(budget)
        self.confidence_pub.publish(confidence_msg)

        values = {name: 0.0 for name in self.DIAGNOSTIC_NAMES}
        values.update({
            'valid': 1.0 if valid else 0.0,
            'left_points': 120.0 if valid else 0.0,
            'right_points': 120.0 if valid else 0.0,
            'corridor_width_m': self.row_spacing,
            'safety_margin_m': self.safety_margin,
            'published_confidence': confidence,
            'row_yaw_rad': self.yaw,
            'path_points': float(len(path.poses)),
            'support_score': 1.0 if valid else 0.0,
            'observation_score': 1.0 if valid else 0.0,
            'width_score': 1.0 if valid else 0.0,
            'residual_score': 1.0 if valid else 0.0,
            'safety_score': 1.0,
            'observed_valid_ratio': 1.0 if valid else 0.0,
            'raw_confidence': confidence,
            'error_budget_m': self.safety_margin,
            'innermost_enabled': 1.0,
            'robust_enabled': 1.0,
            'temporal_enabled': 1.0,
            'parallel_enabled': 1.0,
            'left_longitudinal_coverage': 1.0 if valid else 0.0,
            'right_longitudinal_coverage': 1.0 if valid else 0.0,
        })
        diagnostics = Float32MultiArray()
        diagnostics.layout.dim = [MultiArrayDimension(
            label=','.join(self.DIAGNOSTIC_NAMES),
            size=len(self.DIAGNOSTIC_NAMES),
            stride=len(self.DIAGNOSTIC_NAMES))]
        diagnostics.data = [float(values[name]) for name in self.DIAGNOSTIC_NAMES]
        self.diagnostics_pub.publish(diagnostics)

    def publish_path_set(self, path):
        path = self.stamped(path)
        self.path_pub.publish(path)
        self.path_viz_pub.publish(path)
        self.left_pub.publish(self.stamped(
            self.shifted_boundary(path, 0.5 * self.row_spacing)))
        self.right_pub.publish(self.stamped(
            self.shifted_boundary(path, -0.5 * self.row_spacing)))

    def publish_empty(self):
        empty = Path()
        empty.header.frame_id = 'odom'
        empty = self.stamped(empty)
        self.path_pub.publish(empty)
        self.path_viz_pub.publish(empty)
        self.left_pub.publish(empty)
        self.right_pub.publish(empty)
        return empty

    def publish(self):
        if not self.have_odom:
            return

        if self.stage == Stage.START_DELAY:
            elapsed = (self.get_clock().now() - self.anchor_time).nanoseconds / 1e9
            empty = self.publish_empty()
            self.publish_quality(False, empty, self.invalid_confidence)
            self.headland_pub.publish(Bool(data=False))
            if elapsed >= self.start_delay:
                # Anchor after the stationary delay so localization startup
                # drift does not shift the synthetic rows.
                self.origin_x = self.x
                self.origin_y = self.y
                self.origin_yaw = self.yaw
                self.first_path = self.build_line(
                    start_s=0.0, lateral=0.0, length=self.first_length,
                    travel_sign=1.0)
                self.set_stage(Stage.FIRST_ROW)

        elif self.stage == Stage.FIRST_ROW:
            self.publish_path_set(self.first_path)
            self.publish_quality(True, self.first_path, self.valid_confidence)
            self.headland_pub.publish(Bool(data=False))
            if self.longitudinal_station() >= self.first_length:
                self.set_stage(Stage.HEADLAND)

        elif self.stage == Stage.HEADLAND:
            empty = self.publish_empty()
            self.publish_quality(False, empty, self.invalid_confidence)
            self.headland_pub.publish(Bool(data=True))

        elif self.stage == Stage.U_TURN:
            empty = self.publish_empty()
            self.publish_quality(False, empty, self.invalid_confidence)
            self.headland_pub.publish(Bool(data=False))

        elif self.stage == Stage.SECOND_ROW:
            self.publish_path_set(self.second_path)
            self.publish_quality(True, self.second_path, self.valid_confidence)
            self.headland_pub.publish(Bool(data=False))
            progress = (
                self.second_start_s - self.longitudinal_station()
                if self.second_start_s is not None else 0.0)
            if self.saw_reacquire and self.mode == 'ROW_FOLLOW' and progress >= self.second_length:
                self.set_stage(Stage.FINISHED)

        elif self.stage == Stage.FINISHED:
            empty = self.publish_empty()
            self.publish_quality(False, empty, self.invalid_confidence)
            self.headland_pub.publish(Bool(data=False))

        self.stage_pub.publish(String(data=self.stage.value))
        self.finished_pub.publish(Bool(data=self.stage == Stage.FINISHED))


def main(args=None):
    rclpy.init(args=args)
    node = IndoorHeadlandSimulator()
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
