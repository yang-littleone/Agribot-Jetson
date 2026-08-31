#!/usr/bin/env python3
"""
Collect per-frame centerline latency and Jetson process resource usage.

The detector publishes /centerline_performance only when the measurement
parameter is enabled. This collector never subscribes to the large input cloud,
so its measurement overhead stays small. Raw bags are read by rosbag2 only.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import statistics
import time

from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def percentile(values, percent):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def summarize(values):
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {
            'count': 0, 'mean': math.nan, 'sd': math.nan,
            'median': math.nan, 'p95': math.nan, 'p99': math.nan,
            'maximum': math.nan,
        }
    return {
        'count': len(clean),
        'mean': statistics.fmean(clean),
        'sd': statistics.stdev(clean) if len(clean) > 1 else 0.0,
        'median': statistics.median(clean),
        'p95': percentile(clean, 95.0),
        'p99': percentile(clean, 99.0),
        'maximum': max(clean),
    }


def stamp_ns_from_path(message):
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def find_detector_pid():
    own_pid = os.getpid()
    candidates = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(
                errors='replace')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if 'corn_row_detector_projection' in command:
            try:
                process_name = (entry / 'comm').read_text(
                    encoding='utf-8').strip()
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                process_name = ''
            is_detector_binary = process_name.startswith('corn_row_detect')
            candidates.append((
                0 if is_detector_binary else 1,
                int(entry.name),
                command.strip()))
    if not candidates:
        return None, ''
    _, pid, command = min(candidates)
    return pid, command


def read_process_ticks(pid):
    raw = Path(f'/proc/{pid}/stat').read_text(encoding='utf-8')
    fields = raw[raw.rfind(')') + 2:].split()
    return int(fields[11]) + int(fields[12])


def read_rss_mib(pid):
    for line in Path(f'/proc/{pid}/status').read_text(encoding='utf-8').splitlines():
        if line.startswith('VmRSS:'):
            return float(line.split()[1]) / 1024.0
    return math.nan


def read_system_cpu_ticks():
    fields = Path('/proc/stat').read_text(encoding='utf-8').splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_memory_mib():
    values = {}
    for line in Path('/proc/meminfo').read_text(encoding='utf-8').splitlines():
        key, rest = line.split(':', 1)
        values[key] = float(rest.split()[0]) / 1024.0
    return values.get('MemTotal', math.nan), values.get('MemAvailable', math.nan)


def read_temperature_c():
    values = []
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            value = float(path.read_text(encoding='utf-8').strip())
        except (OSError, ValueError):
            continue
        values.append(value / 1000.0 if value > 1000.0 else value)
    return max(values) if values else math.nan


def read_cpu_frequency_mhz():
    values = []
    for path in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'):
        try:
            values.append(float(path.read_text(encoding='utf-8').strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return statistics.fmean(values) if values else math.nan


class RuntimeCollector(Node):
    def __init__(self, output_dir, run_id, sample_period, warmup_frames):
        super().__init__('centerline_runtime_benchmark')
        self.output_dir = output_dir
        self.run_id = run_id
        self.warmup_frames = warmup_frames
        self.frames = []
        self.paths = {}
        self.resources = []
        self.detector_pid = None
        self.detector_command = ''
        self.measured_detector_pid = None
        self.measured_detector_command = ''
        self.previous_resource_time = None
        self.previous_process_ticks = None
        self.previous_system_ticks = None
        self.clock_ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        self.cpu_count = os.cpu_count() or 1

        self.create_subscription(
            Float64MultiArray,
            '/centerline_performance',
            self.performance_callback,
            100)
        self.create_subscription(
            PathMsg,
            '/corn_row_center_line',
            self.path_callback,
            100)
        self.create_timer(sample_period, self.resource_callback)

    def path_callback(self, message):
        stamp_ns = stamp_ns_from_path(message)
        self.paths[stamp_ns] = {
            'path_arrival_steady_s': time.monotonic(),
            'path_points': len(message.poses),
        }

    def performance_callback(self, message):
        if len(message.data) != 5:
            self.get_logger().warning(
                f'Unexpected performance field count: {len(message.data)}')
            return
        stamp_sec, stamp_nanosec, start_s, end_s, duration_ms = message.data
        stamp_ns = int(round(stamp_sec)) * 1_000_000_000 + int(round(stamp_nanosec))
        self.frames.append({
            'frame_index': len(self.frames),
            'input_stamp_ns': stamp_ns,
            'input_stamp_s': stamp_ns / 1e9,
            'callback_start_steady_s': float(start_s),
            'callback_end_steady_s': float(end_s),
            'callback_duration_ms': float(duration_ms),
            'performance_arrival_steady_s': time.monotonic(),
        })

    def resource_callback(self):
        now = time.monotonic()
        if self.detector_pid is None:
            self.detector_pid, self.detector_command = find_detector_pid()
            if self.detector_pid is None:
                return
            self.measured_detector_pid = self.detector_pid
            self.measured_detector_command = self.detector_command
            self.get_logger().info(f'Detector PID: {self.detector_pid}')

        try:
            process_ticks = read_process_ticks(self.detector_pid)
            rss_mib = read_rss_mib(self.detector_pid)
            system_ticks = read_system_cpu_ticks()
            memory_total_mib, memory_available_mib = read_memory_mib()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            self.detector_pid = None
            self.previous_resource_time = None
            self.previous_process_ticks = None
            self.previous_system_ticks = None
            return

        process_cpu_one_core = math.nan
        process_cpu_total = math.nan
        system_cpu = math.nan
        if self.previous_resource_time is not None:
            elapsed = now - self.previous_resource_time
            if elapsed > 0.0:
                process_seconds = (
                    process_ticks - self.previous_process_ticks) / self.clock_ticks
                process_cpu_one_core = 100.0 * process_seconds / elapsed
                process_cpu_total = process_cpu_one_core / self.cpu_count
                total_delta = system_ticks[0] - self.previous_system_ticks[0]
                idle_delta = system_ticks[1] - self.previous_system_ticks[1]
                if total_delta > 0:
                    system_cpu = 100.0 * (total_delta - idle_delta) / total_delta

        self.resources.append({
            'sample_index': len(self.resources),
            'steady_s': now,
            'detector_pid': self.detector_pid,
            'process_cpu_percent_one_core': process_cpu_one_core,
            'process_cpu_percent_total_capacity': process_cpu_total,
            'process_rss_mib': rss_mib,
            'system_cpu_percent': system_cpu,
            'system_memory_used_mib': memory_total_mib - memory_available_mib,
            'system_memory_available_mib': memory_available_mib,
            'max_thermal_zone_c': read_temperature_c(),
            'mean_online_cpu_frequency_mhz': read_cpu_frequency_mhz(),
        })
        self.previous_resource_time = now
        self.previous_process_ticks = process_ticks
        self.previous_system_ticks = system_ticks

    def write_results(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        merged = []
        for row in self.frames:
            path = self.paths.get(row['input_stamp_ns'], {})
            path_arrival = path.get('path_arrival_steady_s', math.nan)
            path_points = path.get('path_points', -1)
            row = dict(row)
            row['path_arrival_steady_s'] = path_arrival
            row['end_to_end_path_ms'] = (
                (path_arrival - row['callback_start_steady_s']) * 1000.0
                if math.isfinite(path_arrival) else math.nan)
            row['performance_transport_ms'] = (
                (row['performance_arrival_steady_s']
                 - row['callback_end_steady_s']) * 1000.0)
            row['path_points'] = path_points
            row['valid_path'] = 1 if path_points > 0 else 0
            row['warmup'] = 1 if row['frame_index'] < self.warmup_frames else 0
            merged.append(row)

        frame_path = self.output_dir / 'frame_metrics.csv'
        if merged:
            with frame_path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(merged[0]))
                writer.writeheader()
                writer.writerows(merged)

        analysis_rows = [row for row in merged if not row['warmup']]
        callback_values = [row['callback_duration_ms'] for row in analysis_rows]
        e2e_values = [row['end_to_end_path_ms'] for row in analysis_rows]
        transport_values = [row['performance_transport_ms'] for row in analysis_rows]
        source_stamps = [row['input_stamp_s'] for row in analysis_rows]
        starts = [row['callback_start_steady_s'] for row in analysis_rows]
        valid_count = sum(row['valid_path'] for row in analysis_rows)
        path_match_count = sum(
            math.isfinite(row['end_to_end_path_ms']) for row in analysis_rows)

        def frequency(values):
            if len(values) < 2 or values[-1] <= values[0]:
                return math.nan
            return (len(values) - 1) / (values[-1] - values[0])

        active_start = (
            analysis_rows[0]['callback_start_steady_s']
            if analysis_rows else math.nan)
        active_end = (
            analysis_rows[-1]['callback_end_steady_s']
            if analysis_rows else math.nan)
        for row in self.resources:
            row['active_measurement'] = int(
                math.isfinite(active_start)
                and active_start <= row['steady_s'] <= active_end)

        resource_path = self.output_dir / 'resource_usage.csv'
        if self.resources:
            with resource_path.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=list(self.resources[0]))
                writer.writeheader()
                writer.writerows(self.resources)

        valid_resources = [
            row for row in self.resources
            if row['active_measurement']
            and math.isfinite(row['process_cpu_percent_one_core'])]
        summary = {
            'run_id': self.run_id,
            'measurement_definition': {
                'callback_duration': (
                    'steady-clock time from point-cloud callback entry through '
                    'normal/empty path and diagnostic publication, excluding only '
                    'the performance-message publication'),
                'end_to_end_path': (
                    'steady-clock time from detector callback entry to receipt of '
                    '/corn_row_center_line by the external collector'),
                'cpu': 'detector process CPU; 100% equals one fully occupied CPU core',
            },
            'warmup_frames_excluded': self.warmup_frames,
            'frames_total': len(merged),
            'frames_analyzed': len(analysis_rows),
            'path_stamp_matches': path_match_count,
            'valid_path_frames': valid_count,
            'valid_path_percent': (
                100.0 * valid_count / len(analysis_rows)
                if analysis_rows else math.nan),
            'input_frequency_hz_from_source_stamps': frequency(source_stamps),
            'processed_output_frequency_hz': frequency(starts),
            'callback_duration_ms': summarize(callback_values),
            'end_to_end_path_ms': summarize(e2e_values),
            'performance_transport_ms': summarize(transport_values),
            'resource_samples': len(valid_resources),
            'process_cpu_percent_one_core': summarize([
                row['process_cpu_percent_one_core'] for row in valid_resources]),
            'process_cpu_percent_total_capacity': summarize([
                row['process_cpu_percent_total_capacity'] for row in valid_resources]),
            'process_rss_mib': summarize([
                row['process_rss_mib'] for row in valid_resources]),
            'system_cpu_percent': summarize([
                row['system_cpu_percent'] for row in valid_resources]),
            'max_thermal_zone_c': summarize([
                row['max_thermal_zone_c'] for row in valid_resources]),
            'mean_online_cpu_frequency_mhz': summarize([
                row['mean_online_cpu_frequency_mhz'] for row in valid_resources]),
            'detector_pid': self.measured_detector_pid,
            'detector_command': self.measured_detector_command,
            'cpu_logical_count': self.cpu_count,
        }
        (self.output_dir / 'run_summary.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True),
            encoding='utf-8')
        return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--sample-period', type=float, default=0.5)
    parser.add_argument('--warmup-frames', type=int, default=30)
    args = parser.parse_args()

    rclpy.init()
    node = RuntimeCollector(
        args.output_dir.resolve(), args.run_id,
        max(args.sample_period, 0.1), max(args.warmup_frames, 0))

    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        summary = node.write_results()
        print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
