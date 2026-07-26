#!/usr/bin/env python3
"""记录固定路径 PID 跟踪误差，并给出下一轮安全的调参建议。"""

import csv
import json
import math
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as PathMsg
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


class PIDTrackingEvaluator(Node):
    """以路径线段投影为基准，而非简单取最近离散点，计算横向误差。"""

    gain_names = (
        'lateral_kp', 'lateral_ki', 'lateral_kd',
        'heading_kp', 'heading_ki', 'heading_kd',
        'max_linear_speed', 'max_angular_speed',
        'max_angular_acceleration', 'angular_command_deadband',
    )

    def __init__(self):
        super().__init__('pid_tracking_evaluator')
        path_topic = self.declare_parameter('path_topic', '/corn_row_center_line').value
        odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.controller_node = self.declare_parameter(
            'controller_node', '/pid_controller').value
        self.max_angular_speed = float(self.declare_parameter('max_angular_speed', 0.70).value)
        self.sample_period = float(self.declare_parameter('sample_period', 0.05).value)
        self.finish_tolerance = float(self.declare_parameter('finish_tolerance', 0.12).value)
        self.finish_hold_time = float(self.declare_parameter('finish_hold_time', 0.60).value)
        self.auto_finish = bool(self.declare_parameter('auto_finish', True).value)
        self.output_dir = self.declare_parameter(
            'output_dir', 'pid_tracking_results').value

        self.points = []
        self.path_length = 0.0
        self.records = []
        self.latest_cmd = Twist()
        self.start_time = None
        self.last_sample_time = None
        self.finish_started = None
        self.finished = False
        self.results_written = False
        self.gains = None
        self.parameter_future = None

        self.create_subscription(PathMsg, path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_callback, 20)
        self.parameter_client = self.create_client(
            GetParameters, f'{self.controller_node}/get_parameters')
        self.create_timer(0.25, self.timer_callback)

        self.get_logger().info(
            f'评估器已启动：等待 {path_topic} 和 {odom_topic}；结束后写入 {self.output_dir}')

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        self.points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.path_length = sum(
            math.hypot(self.points[index + 1][0] - self.points[index][0],
                       self.points[index + 1][1] - self.points[index][1])
            for index in range(len(self.points) - 1))

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def project_to_path(self, x, y):
        best = None
        accumulated = 0.0
        for index in range(len(self.points) - 1):
            x0, y0 = self.points[index]
            x1, y1 = self.points[index + 1]
            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length < 1e-9:
                continue
            fraction = clamp(((x - x0) * dx + (y - y0) * dy) / (length * length), 0.0, 1.0)
            projected_x = x0 + fraction * dx
            projected_y = y0 + fraction * dy
            distance = math.hypot(x - projected_x, y - projected_y)
            if best is None or distance < best['distance']:
                # 路径左侧为正，右侧为负，符号便于判断是否持续偏在一侧。
                signed_error = (dx * (y - y0) - dy * (x - x0)) / length
                best = {
                    'distance': distance,
                    'signed_error': signed_error,
                    'path_heading': math.atan2(dy, dx),
                    'progress': accumulated + fraction * length,
                }
            accumulated += length
        return best

    def odom_callback(self, msg):
        if len(self.points) < 2 or self.finished:
            return
        now = self.get_clock().now()
        if self.last_sample_time and (now - self.last_sample_time).nanoseconds < self.sample_period * 1e9:
            return
        self.last_sample_time = now
        if self.start_time is None:
            self.start_time = now

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        projection = self.project_to_path(x, y)
        if projection is None:
            return

        end_x, end_y = self.points[-1]
        end_distance = math.hypot(x - end_x, y - end_y)
        self.records.append({
            'time_s': round((now - self.start_time).nanoseconds / 1e9, 4),
            'x_m': x,
            'y_m': y,
            'yaw_rad': yaw,
            'cross_track_error_m': projection['signed_error'],
            'abs_cross_track_error_m': projection['distance'],
            'heading_error_rad': normalize_angle(projection['path_heading'] - yaw),
            'path_progress_m': projection['progress'],
            'endpoint_distance_m': end_distance,
            'linear_cmd_mps': self.latest_cmd.linear.x,
            'angular_cmd_rps': self.latest_cmd.angular.z,
        })

        stopped = abs(self.latest_cmd.linear.x) < 0.01 and abs(self.latest_cmd.angular.z) < 0.03
        if end_distance <= self.finish_tolerance and stopped:
            if self.finish_started is None:
                self.finish_started = now
        else:
            self.finish_started = None

    def timer_callback(self):
        if self.gains is None and self.parameter_future is None and self.parameter_client.service_is_ready():
            request = GetParameters.Request()
            request.names = list(self.gain_names)
            self.parameter_future = self.parameter_client.call_async(request)
            self.parameter_future.add_done_callback(self.gains_callback)

        if (self.auto_finish and not self.finished and self.finish_started is not None and
                (self.get_clock().now() - self.finish_started).nanoseconds >= self.finish_hold_time * 1e9):
            self.finished = True
            self.write_results()
            rclpy.shutdown()

    def gains_callback(self, future):
        try:
            response = future.result()
            self.gains = {
                name: value.double_value
                for name, value in zip(self.gain_names, response.values)
            }
            self.get_logger().info('已读取当前 PID 参数，报告会给出下一轮建议。')
        except Exception as error:  # 参数服务不可用不应妨碍记录误差。
            self.get_logger().warn(f'无法读取控制器参数：{error}')
        finally:
            self.parameter_future = None

    @staticmethod
    def percentile(values, fraction):
        ordered = sorted(values)
        index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
        return ordered[index]

    @staticmethod
    def count_oscillations(records):
        # 只统计横向误差大于 1.5 cm 的有效换侧，过滤里程计微小噪声。
        sign = 0
        crossings = 0
        for record in records:
            error = record['cross_track_error_m']
            next_sign = 1 if error > 0.015 else -1 if error < -0.015 else 0
            if next_sign and sign and next_sign != sign:
                crossings += 1
            if next_sign:
                sign = next_sign
        return crossings

    def recommendation(self, stats):
        if not stats['experiment_valid']:
            return {
                'status': (
                    '本次试验无效，不据此调 PID：测试路径未完整跑完，'
                    '或机器人未产生足够的路径进度。'),
                'apply_one_change_only': True,
                'proposed_parameters': {},
            }

        if self.gains is None:
            return {
                'status': '无法读取当前控制器参数，仅输出误差数据；请确认控制器节点名称。',
                'apply_one_change_only': True,
                'proposed_parameters': {},
            }

        proposed = dict(self.gains)
        if stats['rmse_m'] <= 0.04 and stats['p95_m'] <= 0.07 and stats['saturation_ratio'] < 0.05:
            status = '通过：横向误差已适合此室内测试，下一轮保持 PID 不变，仅可小幅提高速度。'
        elif stats['oscillation_crossings'] >= 4 or stats['saturation_ratio'] >= 0.20:
            # 振荡或长期饱和时先降 P，避免把噪声/延迟当成需更大增益的误差。
            proposed['lateral_kp'] = round(max(0.05, proposed['lateral_kp'] * 0.85), 4)
            status = '检测到振荡或转向饱和：下一轮只降低 lateral_kp，速度保持不变。'
        elif abs(stats['mean_signed_error_m']) >= 0.025 and stats['rmse_m'] <= 0.08:
            # 只有误差稳定偏向一侧才建议很小的 I，防止曲线阶段导致积分累积。
            proposed['lateral_ki'] = round(max(proposed['lateral_ki'], 0.01), 4)
            status = '存在稳定单侧偏差：下一轮只尝试很小的 lateral_ki。'
        else:
            proposed['lateral_kp'] = round(proposed['lateral_kp'] * 1.15, 4)
            status = '误差偏大但未见明显振荡：下一轮小幅提高 lateral_kp。'

        return {
            'status': status,
            'apply_one_change_only': True,
            'proposed_parameters': proposed,
        }

    def write_results(self):
        if self.results_written:
            return
        if not self.records:
            return

        # 轨迹第一次进入终点容差后，后续行驶不属于路径跟踪本身。
        # 即使控制器或人工停车有延迟，也不能用这些越界样本误调 PID。
        analysis_records = self.records
        endpoint_index = next(
            (index for index, record in enumerate(self.records)
             if record['endpoint_distance_m'] <= self.finish_tolerance),
            None)
        if endpoint_index is not None:
            analysis_records = self.records[:endpoint_index + 1]

        absolute_errors = [record['abs_cross_track_error_m'] for record in analysis_records]
        signed_errors = [record['cross_track_error_m'] for record in analysis_records]
        heading_errors = [abs(record['heading_error_rad']) for record in analysis_records]
        saturation_ratio = sum(
            abs(record['angular_cmd_rps']) >= 0.95 * self.max_angular_speed
            for record in analysis_records) / len(analysis_records)
        max_progress = max(record['path_progress_m'] for record in analysis_records)
        # 只有确实抵达终点，且已经覆盖绝大部分路径，才允许把结果用于 PID 调参。
        # 这样“起点被误判为终点”或人工刚启动就停止时，不会出现 RMSE=0 的假通过。
        completion_ratio = max_progress / self.path_length if self.path_length > 1e-9 else 0.0
        experiment_valid = endpoint_index is not None and completion_ratio >= 0.70
        stats = {
            'samples': len(analysis_records),
            'raw_samples': len(self.records),
            'duration_s': round(analysis_records[-1]['time_s'], 3),
            'path_length_m': round(self.path_length, 3),
            'max_progress_m': round(max_progress, 3),
            'completion_ratio': round(completion_ratio, 4),
            'endpoint_reached': endpoint_index is not None,
            'experiment_valid': experiment_valid,
            'rmse_m': round(math.sqrt(sum(error * error for error in absolute_errors) / len(absolute_errors)), 4),
            'mean_abs_error_m': round(sum(absolute_errors) / len(absolute_errors), 4),
            'mean_signed_error_m': round(sum(signed_errors) / len(signed_errors), 4),
            'p95_m': round(self.percentile(absolute_errors, 0.95), 4),
            'max_error_m': round(max(absolute_errors), 4),
            'mean_abs_heading_error_deg': round(math.degrees(sum(heading_errors) / len(heading_errors)), 3),
            'final_endpoint_error_m': round(analysis_records[-1]['endpoint_distance_m'], 4),
            'saturation_ratio': round(saturation_ratio, 4),
            'oscillation_crossings': self.count_oscillations(analysis_records),
        }
        report = {
            'generated_at': datetime.now().isoformat(timespec='seconds'),
            'metrics': stats,
            'current_parameters': self.gains,
            'recommendation': self.recommendation(stats),
        }

        run_dir = Path(self.output_dir).expanduser() / datetime.now().strftime('run_%Y%m%d_%H%M%S')
        run_dir.mkdir(parents=True, exist_ok=False)
        with (run_dir / 'tracking_samples.csv').open('w', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)
        with (run_dir / 'report.json').open('w', encoding='utf-8') as file:
            json.dump(report, file, ensure_ascii=False, indent=2)

        proposed = report['recommendation']['proposed_parameters']
        if proposed:
            with (run_dir / 'recommended_pid.yaml').open('w', encoding='utf-8') as file:
                file.write('pid_controller:\n  ros__parameters:\n')
                for name in self.gain_names:
                    file.write(f'    {name}: {proposed[name]:.4f}\n')

        self.results_written = True
        validity = '有效' if experiment_valid else '无效'
        self.get_logger().info(
            f"评估完成（{validity}）：RMSE={stats['rmse_m']:.3f} m, P95={stats['p95_m']:.3f} m, "
            f"最大误差={stats['max_error_m']:.3f} m, "
            f"饱和率={100.0 * stats['saturation_ratio']:.1f}%。结果：{run_dir}")
        self.get_logger().info(f"建议：{report['recommendation']['status']}")


def main():
    rclpy.init()
    node = PIDTrackingEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch 的 SIGINT 会使 spin() 正常返回而非抛出 KeyboardInterrupt。
        # 因此在 finally 中统一落盘，确保人工停止的试验也保留已有数据。
        if not node.results_written:
            node.write_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
