#!/usr/bin/env python3
"""以固定字段连续记录田间试验数据，并保存感知/控制参数快照."""

import csv
import json
import math
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as PathMsg
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def parameter_value(value):
    if value.type == 1:
        return value.bool_value
    if value.type == 2:
        return value.integer_value
    if value.type == 3:
        return value.double_value
    if value.type == 4:
        return value.string_value
    if value.type == 5:
        return list(value.byte_array_value)
    if value.type == 6:
        return list(value.bool_array_value)
    if value.type == 7:
        return list(value.integer_array_value)
    if value.type == 8:
        return list(value.double_array_value)
    if value.type == 9:
        return list(value.string_array_value)
    return None


class FieldTrialLogger(Node):
    DIAGNOSTIC_NAMES = (
        'valid', 'left_points', 'right_points', 'corridor_width_m',
        'safety_margin_m', 'published_confidence', 'row_yaw_rad',
        'center_offset_m', 'left_slope', 'left_intercept', 'right_slope',
        'right_intercept', 'line_lost_count', 'path_points', 'support_score',
        'observation_score', 'width_score', 'residual_score', 'safety_score',
        'observed_valid_ratio', 'raw_confidence', 'error_budget_m',
        'support_weight', 'observation_weight', 'width_weight',
        'residual_weight', 'safety_weight', 'innermost_enabled', 'robust_enabled',
        'temporal_enabled', 'parallel_enabled',
    )
    CONTROLLER_PARAMETERS = (
        'target_distance', 'max_linear_speed', 'min_linear_speed',
        'max_angular_speed', 'lateral_kp', 'lateral_ki', 'lateral_kd',
        'heading_kp', 'heading_ki', 'heading_kd',
        'use_quality_aware_control', 'require_quality_metrics',
        'confidence_high_threshold', 'confidence_low_threshold',
        'confidence_stop_threshold', 'confidence_min_speed_factor',
        'max_low_confidence_duration', 'centerline_timeout', 'quality_timeout',
        'safety_margin_high', 'safety_margin_mid', 'safety_margin_stop',
        'max_row_follow_distance', 'max_row_follow_time',
        'enable_headland_turn',
    )
    DETECTOR_PARAMETERS = (
        'point_cloud_topic', 'z_min', 'z_max', 'voxel_size', 'x_min', 'x_max',
        'y_min', 'y_max', 'path_length', 'path_step', 'platform_width',
        'desired_row_separation', 'plant_safety_clearance',
        'min_row_separation', 'max_row_separation', 'section_width',
        'min_section_points', 'robust_fit_residual_threshold',
        'enable_innermost_row_extraction', 'enable_robust_refinement',
        'use_parallel_row_model', 'enable_temporal_tracking',
        'enable_quality_evaluation', 'quality_support_weight',
        'quality_observation_weight', 'quality_width_weight',
        'quality_residual_weight', 'quality_safety_weight',
        'quality_observation_fallback_score',
    )

    def __init__(self):
        super().__init__('field_trial_logger')
        trial_id = str(self.declare_parameter('trial_id', 'commissioning').value)
        experiment_type = str(self.declare_parameter(
            'experiment_type', 'closed_loop').value)
        scenario = str(self.declare_parameter('scenario', 'normal').value)
        perception_method = str(self.declare_parameter(
            'perception_method', 'full').value)
        repeat_index = int(self.declare_parameter('repeat_index', 1).value)
        nominal_speed = float(self.declare_parameter('nominal_speed', 0.10).value)
        quality_aware = bool(self.declare_parameter('quality_aware', True).value)
        output_dir = Path(str(self.declare_parameter(
            'output_dir', 'field_trial_results').value)).expanduser()
        self.sample_period = max(
            0.01, float(self.declare_parameter('sample_period', 0.05).value))
        controller_node = str(self.declare_parameter(
            'controller_node', '/pid_controller').value)
        detector_node = str(self.declare_parameter(
            'detector_node', '/corn_row_detector_projection').value)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_trial_id = ''.join(
            character if character.isalnum() or character in '-_' else '_'
            for character in trial_id)
        self.run_dir = output_dir / f'{stamp}_{safe_trial_id}'
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.csv_file = (self.run_dir / 'timeseries.csv').open(
            'w', newline='', encoding='utf-8')

        self.fieldnames = [
            'trial_id', 'experiment_type', 'scenario', 'perception_method',
            'repeat_index', 'nominal_speed_mps', 'quality_aware',
            'time_s', 'ros_time_s',
            'x_m', 'y_m', 'yaw_rad',
            'odom_linear_mps', 'odom_angular_rps', 'travel_distance_m',
            'cmd_linear_mps', 'cmd_angular_rps', 'path_points',
            'path_first_x_m', 'path_first_y_m', 'path_mid_x_m', 'path_mid_y_m',
            'path_last_x_m', 'path_last_y_m', 'corridor_width_m',
            'safety_margin_m', 'error_budget_m', 'published_confidence',
            'control_quality_factor', 'headland_detected', 'navigation_mode',
            'navigation_safety_state',
        ] + [
            name for name in self.DIAGNOSTIC_NAMES
            if name not in {
                'path_points', 'corridor_width_m', 'safety_margin_m',
                'error_budget_m', 'published_confidence',
            }
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.identity = {
            'trial_id': trial_id,
            'experiment_type': experiment_type,
            'scenario': scenario,
            'perception_method': perception_method,
            'repeat_index': repeat_index,
            'nominal_speed_mps': nominal_speed,
            'quality_aware': quality_aware,
        }
        self.metadata = {
            **self.identity,
            'started_at_local': datetime.now().isoformat(timespec='seconds'),
            'risk_threshold_definition': (
                'row_width/2 - platform_width/2 - plant_safety_clearance'),
            'data_file': 'timeseries.csv',
            'parameter_snapshots': {},
        }
        self.metadata_path = self.run_dir / 'metadata.json'
        self.write_metadata()

        self.start_time = self.get_clock().now()
        self.latest_odom = None
        self.latest_cmd = Twist()
        self.latest_path = None
        self.metrics = {
            'corridor_width_m': math.nan,
            'safety_margin_m': math.nan,
            'error_budget_m': math.nan,
            'published_confidence': math.nan,
            'control_quality_factor': math.nan,
        }
        self.diagnostics = {}
        self.headland_detected = False
        self.navigation_mode = ''
        self.navigation_safety_state = ''
        self.travel_distance = 0.0
        self.previous_xy = None

        self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 20)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 20)
        self.create_subscription(
            PathMsg, '/corn_row_center_line', self.path_callback, 10)
        self.create_subscription(
            Float32, '/corridor_width',
            lambda msg: self.set_metric('corridor_width_m', msg.data), 10)
        self.create_subscription(
            Float32, '/corridor_safety_margin',
            lambda msg: self.set_metric('safety_margin_m', msg.data), 10)
        self.create_subscription(
            Float32, '/corridor_error_budget',
            lambda msg: self.set_metric('error_budget_m', msg.data), 10)
        self.create_subscription(
            Float32, '/corridor_confidence',
            lambda msg: self.set_metric('published_confidence', msg.data), 10)
        self.create_subscription(
            Float32, '/control_quality_factor',
            lambda msg: self.set_metric('control_quality_factor', msg.data), 10)
        self.create_subscription(
            Float32MultiArray, '/centerline_detection_diagnostics',
            self.diagnostics_callback, 10)
        self.create_subscription(
            Bool, '/headland_detected',
            lambda msg: setattr(self, 'headland_detected', msg.data), 10)
        self.create_subscription(
            String, '/navigation_mode',
            lambda msg: setattr(self, 'navigation_mode', msg.data), 10)
        self.create_subscription(
            String, '/navigation_safety_state',
            lambda msg: setattr(self, 'navigation_safety_state', msg.data), 10)
        self.create_timer(self.sample_period, self.sample)

        self.parameter_clients = []
        self.request_parameter_snapshot(
            controller_node, self.CONTROLLER_PARAMETERS, 'controller')
        self.request_parameter_snapshot(
            detector_node, self.DETECTOR_PARAMETERS, 'detector')
        self.get_logger().info(f'田间试验记录目录: {self.run_dir}')

    def write_metadata(self):
        self.metadata_path.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2),
            encoding='utf-8')

    def request_parameter_snapshot(self, node_name, names, key):
        client = self.create_client(GetParameters, f'{node_name}/get_parameters')
        entry = {
            'client': client, 'names': names, 'key': key, 'future': None,
        }
        self.parameter_clients.append(entry)
        timer = self.create_timer(
            0.5, lambda entry=entry: self.try_parameter_request(entry))
        entry['timer'] = timer

    def try_parameter_request(self, entry):
        if entry['future'] is not None or not entry['client'].service_is_ready():
            return
        request = GetParameters.Request()
        request.names = list(entry['names'])
        entry['future'] = entry['client'].call_async(request)
        entry['future'].add_done_callback(
            lambda future: self.parameter_response(entry, future))

    def parameter_response(self, entry, future):
        try:
            response = future.result()
            self.metadata['parameter_snapshots'][entry['key']] = {
                name: parameter_value(value)
                for name, value in zip(entry['names'], response.values)
            }
            self.write_metadata()
            entry['timer'].cancel()
        except Exception as error:
            self.get_logger().warn(f"读取{entry['key']}参数失败，将重试: {error}")
            entry['future'] = None

    def set_metric(self, name, value):
        self.metrics[name] = float(value)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def path_callback(self, msg):
        self.latest_path = msg

    def diagnostics_callback(self, msg):
        names = self.DIAGNOSTIC_NAMES
        if msg.layout.dim and msg.layout.dim[0].label:
            parsed = tuple(msg.layout.dim[0].label.split(','))
            if len(parsed) == len(msg.data):
                names = parsed
        self.diagnostics = {
            name: float(value) for name, value in zip(names, msg.data)
        }

    def odom_callback(self, msg):
        self.latest_odom = msg
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.previous_xy is not None:
            step = math.hypot(xy[0] - self.previous_xy[0], xy[1] - self.previous_xy[1])
            if step < 1.0:
                self.travel_distance += step
        self.previous_xy = xy

    def path_values(self):
        values = {
            'path_points': 0,
            'path_first_x_m': math.nan, 'path_first_y_m': math.nan,
            'path_mid_x_m': math.nan, 'path_mid_y_m': math.nan,
            'path_last_x_m': math.nan, 'path_last_y_m': math.nan,
        }
        if self.latest_path is None or not self.latest_path.poses:
            return values
        poses = self.latest_path.poses
        selected = {
            'first': poses[0].pose.position,
            'mid': poses[len(poses) // 2].pose.position,
            'last': poses[-1].pose.position,
        }
        values['path_points'] = len(poses)
        for label, point in selected.items():
            values[f'path_{label}_x_m'] = point.x
            values[f'path_{label}_y_m'] = point.y
        return values

    def sample(self):
        if self.latest_odom is None:
            return
        now = self.get_clock().now()
        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        row = {
            **self.identity,
            'time_s': (now - self.start_time).nanoseconds / 1e9,
            'ros_time_s': now.nanoseconds / 1e9,
            'x_m': pose.position.x,
            'y_m': pose.position.y,
            'yaw_rad': yaw_from_quaternion(pose.orientation),
            'odom_linear_mps': twist.linear.x,
            'odom_angular_rps': twist.angular.z,
            'travel_distance_m': self.travel_distance,
            'cmd_linear_mps': self.latest_cmd.linear.x,
            'cmd_angular_rps': self.latest_cmd.angular.z,
            **self.path_values(),
            **self.metrics,
            'headland_detected': int(self.headland_detected),
            'navigation_mode': self.navigation_mode,
            'navigation_safety_state': self.navigation_safety_state,
            **self.diagnostics,
        }
        self.writer.writerow({
            field: row.get(field, math.nan) for field in self.fieldnames
        })
        self.csv_file.flush()

    def destroy_node(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        self.metadata['ended_at_local'] = datetime.now().isoformat(timespec='seconds')
        self.metadata['logged_distance_m'] = self.travel_distance
        self.write_metadata()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FieldTrialLogger()
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
