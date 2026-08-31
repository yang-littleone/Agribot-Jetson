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
        'temporal_enabled', 'parallel_enabled', 'left_longitudinal_coverage',
        'right_longitudinal_coverage',
    )
    CONTROLLER_PARAMETERS = (
        'target_distance', 'max_linear_speed', 'min_linear_speed',
        'max_angular_speed', 'max_angular_acceleration',
        'angular_command_deadband',
        'lateral_kp', 'lateral_ki', 'lateral_kd',
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
        fallback_output_dir = Path(str(self.declare_parameter(
            'fallback_output_dir', 'field_trial_results').value)).expanduser()
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
        self.run_basename = f'{stamp}_{safe_trial_id}'
        self.requested_output_dir = output_dir
        self.fallback_output_dir = fallback_output_dir
        self.storage_fallback_active = False
        self.csv_file = None
        self.writer = None

        self.fieldnames = [
            'trial_id', 'experiment_type', 'scenario', 'perception_method',
            'repeat_index', 'nominal_speed_mps', 'quality_aware',
            'time_s', 'ros_time_s',
            'odom_stamp_s', 'odom_age_s', 'path_stamp_s', 'path_age_s',
            'x_m', 'y_m', 'yaw_rad',
            'odom_linear_mps', 'odom_angular_rps', 'travel_distance_m',
            'cmd_linear_mps', 'cmd_angular_rps', 'path_points',
            'path_first_x_m', 'path_first_y_m', 'path_mid_x_m', 'path_mid_y_m',
            'path_last_x_m', 'path_last_y_m', 'corridor_width_m',
            'local_path_points',
            'local_path_first_x_m', 'local_path_first_y_m',
            'local_path_mid_x_m', 'local_path_mid_y_m',
            'local_path_last_x_m', 'local_path_last_y_m',
            'safety_margin_m', 'error_budget_m', 'published_confidence',
            'control_quality_factor', 'headland_detected', 'navigation_mode',
            'navigation_safety_state',
            'controller_lateral_error_m', 'controller_heading_error_rad',
        ] + [
            name for name in self.DIAGNOSTIC_NAMES
            if name not in {
                'path_points', 'corridor_width_m', 'safety_margin_m',
                'error_budget_m', 'published_confidence',
            }
        ]
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
            'requested_output_dir': str(output_dir),
            'fallback_output_dir': str(fallback_output_dir),
            'storage_failovers': [],
        }
        try:
            self.activate_storage(output_dir)
        except OSError as error:
            if output_dir.resolve() == fallback_output_dir.resolve():
                raise
            self.storage_fallback_active = True
            self.metadata['storage_failovers'].append({
                'at_local': datetime.now().isoformat(timespec='seconds'),
                'from': str(output_dir),
                'to': str(fallback_output_dir),
                'reason': f'initial_open_failed: {error}',
            })
            self.get_logger().warn(
                f'U盘记录目录不可用，改用工作空间: {error}')
            self.activate_storage(
                fallback_output_dir, suffix='_workspace_fallback')
        self.write_metadata()

        self.start_time = self.get_clock().now()
        self.latest_odom = None
        self.latest_cmd = Twist()
        self.latest_path = None
        self.latest_local_path = None
        self.metrics = {
            'corridor_width_m': math.nan,
            'safety_margin_m': math.nan,
            'error_budget_m': math.nan,
            'published_confidence': math.nan,
            'control_quality_factor': math.nan,
            'controller_lateral_error_m': math.nan,
            'controller_heading_error_rad': math.nan,
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
            PathMsg, '/corn_row_center_line_viz',
            self.local_path_callback, 10)
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
            Float32, '/controller_lateral_error',
            lambda msg: self.set_metric(
                'controller_lateral_error_m', msg.data), 10)
        self.create_subscription(
            Float32, '/controller_heading_error',
            lambda msg: self.set_metric(
                'controller_heading_error_rad', msg.data), 10)
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

    def activate_storage(self, root, suffix=''):
        root = Path(root).expanduser()
        candidate = root / f'{self.run_basename}{suffix}'
        duplicate_index = 1
        while candidate.exists():
            candidate = root / (
                f'{self.run_basename}{suffix}_{duplicate_index}')
            duplicate_index += 1
        candidate.mkdir(parents=True, exist_ok=False)
        csv_file = (candidate / 'timeseries.csv').open(
            'w', newline='', encoding='utf-8')
        writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
        writer.writeheader()
        csv_file.flush()
        self.run_dir = candidate
        self.csv_file = csv_file
        self.writer = writer
        self.metadata_path = candidate / 'metadata.json'
        self.metadata['active_run_dir'] = str(candidate)

    def switch_to_workspace_fallback(self, error):
        if self.storage_fallback_active:
            return False
        if (self.run_dir.parent.resolve() ==
                self.fallback_output_dir.resolve()):
            self.storage_fallback_active = True
            return False

        previous_run_dir = self.run_dir
        try:
            self.csv_file.close()
        except OSError:
            pass
        self.metadata['storage_failovers'].append({
            'at_local': datetime.now().isoformat(timespec='seconds'),
            'from': str(previous_run_dir),
            'to': str(self.fallback_output_dir),
            'reason': f'runtime_write_failed: {error}',
        })
        try:
            self.activate_storage(
                self.fallback_output_dir, suffix='_workspace_continued')
        except OSError as fallback_error:
            self.get_logger().error(
                f'U盘写入失败且工作空间续写也失败: {fallback_error}')
            return False
        self.storage_fallback_active = True
        self.get_logger().error(
            f'U盘写入失败，CSV已转到工作空间续写: {self.run_dir}')
        return True

    def write_metadata(self):
        payload = json.dumps(self.metadata, ensure_ascii=False, indent=2)
        try:
            self.metadata_path.write_text(payload, encoding='utf-8')
        except OSError as error:
            if not self.switch_to_workspace_fallback(error):
                raise
            payload = json.dumps(self.metadata, ensure_ascii=False, indent=2)
            self.metadata_path.write_text(payload, encoding='utf-8')

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

    def local_path_callback(self, msg):
        self.latest_local_path = msg

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

    @staticmethod
    def selected_path_values(path, prefix):
        values = {
            f'{prefix}_points': 0,
            f'{prefix}_first_x_m': math.nan,
            f'{prefix}_first_y_m': math.nan,
            f'{prefix}_mid_x_m': math.nan,
            f'{prefix}_mid_y_m': math.nan,
            f'{prefix}_last_x_m': math.nan,
            f'{prefix}_last_y_m': math.nan,
        }
        if path is None or not path.poses:
            return values
        poses = path.poses
        selected = {
            'first': poses[0].pose.position,
            'mid': poses[len(poses) // 2].pose.position,
            'last': poses[-1].pose.position,
        }
        values[f'{prefix}_points'] = len(poses)
        for label, point in selected.items():
            values[f'{prefix}_{label}_x_m'] = point.x
            values[f'{prefix}_{label}_y_m'] = point.y
        return values

    def path_values(self):
        return {
            **self.selected_path_values(self.latest_path, 'path'),
            **self.selected_path_values(self.latest_local_path, 'local_path'),
        }

    def sample(self):
        if self.latest_odom is None:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        odom_stamp_s = (
            self.latest_odom.header.stamp.sec +
            self.latest_odom.header.stamp.nanosec * 1e-9)
        path_stamp_s = math.nan
        if self.latest_path is not None:
            path_stamp_s = (
                self.latest_path.header.stamp.sec +
                self.latest_path.header.stamp.nanosec * 1e-9)
        row = {
            **self.identity,
            'time_s': (now - self.start_time).nanoseconds / 1e9,
            'ros_time_s': now_s,
            'odom_stamp_s': odom_stamp_s,
            'odom_age_s': (
                max(0.0, now_s - odom_stamp_s)
                if odom_stamp_s > 0.0 else math.nan),
            'path_stamp_s': path_stamp_s,
            'path_age_s': (
                max(0.0, now_s - path_stamp_s)
                if math.isfinite(path_stamp_s) and path_stamp_s > 0.0
                else math.nan),
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
        output_row = {
            field: row.get(field, math.nan) for field in self.fieldnames
        }
        try:
            self.writer.writerow(output_row)
            self.csv_file.flush()
        except OSError as error:
            if not self.switch_to_workspace_fallback(error):
                raise
            self.writer.writerow(output_row)
            self.csv_file.flush()
            self.write_metadata()

    def destroy_node(self):
        try:
            if self.csv_file is not None and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        except OSError as error:
            self.switch_to_workspace_fallback(error)
        self.metadata['ended_at_local'] = datetime.now().isoformat(timespec='seconds')
        self.metadata['logged_distance_m'] = self.travel_distance
        try:
            self.write_metadata()
        except OSError as error:
            self.get_logger().error(f'最终参数快照写入失败: {error}')
        finally:
            try:
                if self.csv_file is not None and not self.csv_file.closed:
                    self.csv_file.flush()
                    self.csv_file.close()
            except OSError as error:
                self.get_logger().error(f'最终CSV关闭失败: {error}')
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
