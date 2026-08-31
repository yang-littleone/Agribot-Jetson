#!/usr/bin/env python3
"""在田间rosbag正常关闭后自动验收bag、CSV和参数快照."""

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import sys

import yaml


REQUIRED_BAG_TOPICS = (
    '/livox/lidar',
    '/imu/selected',
    '/wheel/odom_validated',
    '/odometry/fused_internal',
    '/odometry/filtered',
    '/corn_row_center_line',
    '/corn_row_center_line_viz',
    '/centerline_detection_diagnostics',
    '/corridor_confidence',
)

CLOSED_LOOP_REQUIRED_TOPICS = (
    '/cmd_vel',
    '/controller_lateral_error',
    '/controller_heading_error',
    '/navigation_mode',
    '/navigation_safety_state',
)

HEADLAND_CONTINUOUS_TOPICS = (
    '/headland_detected',
)

# These topics are event-driven. Zero messages can be a valid record of a
# failed detection/turn, so absence is a warning rather than data corruption.
HEADLAND_EVENT_TOPICS = (
    '/headland_turn_path',
    '/reacquire_reference_path',
)


class Validation:
    def __init__(self):
        self.checks = []

    def add(self, name, status, detail):
        self.checks.append({
            'name': name,
            'status': status,
            'detail': str(detail),
        })

    @property
    def verdict(self):
        statuses = {item['status'] for item in self.checks}
        if 'FAIL' in statuses:
            return 'FAIL'
        if 'WARN' in statuses:
            return 'WARN'
        return 'PASS'


def load_bag_metadata(bag_dir):
    metadata_path = bag_dir / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'缺少 {metadata_path}')
    with metadata_path.open(encoding='utf-8') as stream:
        payload = yaml.safe_load(stream)
    information = payload.get('rosbag2_bagfile_information', {})
    if not information:
        raise ValueError('metadata.yaml缺少rosbag2_bagfile_information')
    topics = {}
    for entry in information.get('topics_with_message_count', []):
        topic_metadata = entry.get('topic_metadata', {})
        name = topic_metadata.get('name')
        if name:
            topics[name] = int(entry.get('message_count', 0))
    duration_ns = int(
        information.get('duration', {}).get('nanoseconds', 0))
    starting_time_ns = int(
        information.get(
            'starting_time', {}).get('nanoseconds_since_epoch', 0))
    return (
        information, topics, duration_ns / 1e9, starting_time_ns / 1e9)


def candidate_result_roots(workspace_root, usb_mount):
    roots = [workspace_root / 'field_trial_results']
    if usb_mount:
        roots.append(
            usb_mount / 'agribot_field_data' / 'field_trial_results')
    media_root = Path('/media') / os.environ.get('USER', 'wheeltec')
    if media_root.is_dir():
        try:
            mounts = list(media_root.iterdir())
        except OSError:
            mounts = []
        for mount in mounts:
            roots.append(
                mount / 'agribot_field_data' / 'field_trial_results')
    unique = []
    seen = set()
    for root in roots:
        try:
            key = str(root.resolve())
        except OSError:
            key = str(root)
        if key not in seen:
            seen.add(key)
            unique.append(root)
    return unique


def load_json(path):
    with path.open(encoding='utf-8') as stream:
        return json.load(stream)


def result_start_epoch(candidate, metadata):
    started_at = str(metadata.get('started_at_local', '')).strip()
    if started_at:
        try:
            return datetime.fromisoformat(started_at).timestamp()
        except ValueError:
            pass
    try:
        return datetime.strptime(
            candidate.name[:15], '%Y%m%d_%H%M%S').timestamp()
    except ValueError:
        return None


def find_result_dir(
        workspace_root, usb_mount, trial_id, bag_start_s, bag_duration_s):
    candidates = []
    for root in candidate_result_roots(workspace_root, usb_mount):
        if not root.is_dir():
            continue
        try:
            paths = root.glob(f'*_{trial_id}*')
            for candidate in paths:
                metadata_path = candidate / 'metadata.json'
                if not candidate.is_dir() or not metadata_path.is_file():
                    continue
                try:
                    metadata = load_json(metadata_path)
                    if str(metadata.get('trial_id', '')) != trial_id:
                        continue
                    start_s = result_start_epoch(candidate, metadata)
                    candidates.append((
                        start_s,
                        candidate,
                        metadata,
                    ))
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
        except OSError:
            continue
    if not candidates:
        return None, None, '没有同名trial_id结果目录'

    bag_end_s = bag_start_s + bag_duration_s
    matched = [
        item for item in candidates
        if item[0] is not None
        and bag_start_s - 10.0 <= item[0] <= bag_end_s + 10.0
    ]
    if not matched:
        timed = [item for item in candidates if item[0] is not None]
        if timed:
            nearest = min(
                timed, key=lambda item: abs(item[0] - bag_start_s))
            offset = nearest[0] - bag_start_s
            return (
                None, None,
                f'发现同名目录，但最近CSV启动时间与bag相差{offset:.1f}s；'
                '拒绝自动配对')
        return None, None, '同名目录缺少可解析的启动时间'

    _, result_dir, metadata = min(
        matched, key=lambda item: abs(item[0] - bag_start_s))
    return result_dir, metadata, 'trial_id和录制时间窗均匹配'


def parse_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def inspect_csv(csv_path, trial_id):
    row_count = 0
    wrong_trial_rows = 0
    times = []
    odom_ages = []
    local_path_valid_rows = 0
    with csv_path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or [])
        required = {'trial_id', 'time_s', 'odom_age_s'}
        missing = sorted(required - fieldnames)
        if missing:
            raise ValueError(f'缺少字段: {", ".join(missing)}')
        for row in reader:
            row_count += 1
            if str(row.get('trial_id', '')).strip() != trial_id:
                wrong_trial_rows += 1
            time_s = parse_float(row.get('time_s'))
            if time_s is not None:
                times.append(time_s)
            odom_age = parse_float(row.get('odom_age_s'))
            if odom_age is not None:
                odom_ages.append(odom_age)
            local_points = parse_float(row.get('local_path_points'))
            if local_points is not None and local_points > 0:
                local_path_valid_rows += 1
    duration = max(times) - min(times) if len(times) >= 2 else 0.0
    return {
        'row_count': row_count,
        'wrong_trial_rows': wrong_trial_rows,
        'duration_s': duration,
        'odom_age_count': len(odom_ages),
        'max_odom_age_s': max(odom_ages) if odom_ages else None,
        'local_path_valid_rows': local_path_valid_rows,
    }


def closed_loop_expected(trial_id, result_metadata, topic_counts):
    if result_metadata is not None:
        return result_metadata.get('experiment_type') == 'closed_loop'
    lowered = trial_id.lower()
    static_markers = ('static', 'qv', 'perception_only')
    if any(marker in lowered for marker in static_markers):
        return False
    has_speed_tag = re.search(
        r'(^|_)v[0-9]{2,3}($|_)', lowered) is not None
    return (
        topic_counts.get('/cmd_vel', 0) > 0
        or has_speed_tag
        or any(marker in lowered for marker in ('qaware', 'fixed'))
    )


def run_validation(args):
    validation = Validation()
    bag_dir = Path(args.bag).expanduser().resolve()
    workspace_root = Path(args.workspace_root).expanduser().resolve()
    usb_mount = (
        Path(args.usb_mount).expanduser().resolve()
        if args.usb_mount else None)

    try:
        (
            information, topic_counts, bag_duration, bag_start_s,
        ) = load_bag_metadata(bag_dir)
        validation.add(
            'rosbag_metadata', 'PASS',
            f'可读取，消息总数={information.get("message_count", 0)}，'
            f'时长={bag_duration:.3f}s')
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        validation.add('rosbag_metadata', 'FAIL', error)
        return validation, bag_dir, None, {}

    if bag_duration > 0.0:
        validation.add('rosbag_duration', 'PASS', f'{bag_duration:.3f}s')
    else:
        validation.add('rosbag_duration', 'FAIL', '持续时间为0')

    for topic in REQUIRED_BAG_TOPICS:
        count = topic_counts.get(topic, 0)
        validation.add(
            f'topic:{topic}',
            'PASS' if count > 0 else 'FAIL',
            f'消息数={count}')

    odom_count = topic_counts.get('/odometry/filtered', 0)
    odom_rate = odom_count / bag_duration if bag_duration > 0 else 0.0
    validation.add(
        'bag_odometry_effective_rate',
        'PASS' if odom_rate >= 5.0 else 'FAIL',
        f'{odom_rate:.2f} Hz（要求至少5 Hz）')

    result_dir, result_metadata, result_match_detail = find_result_dir(
        workspace_root, usb_mount, args.trial_id, bag_start_s, bag_duration)
    is_closed_loop = closed_loop_expected(
        args.trial_id, result_metadata, topic_counts)
    validation.add(
        'trial_type', 'PASS',
        '闭环路径跟踪' if is_closed_loop else '静态/仅感知记录')

    cmd_count = topic_counts.get('/cmd_vel', 0)
    if is_closed_loop:
        for topic in CLOSED_LOOP_REQUIRED_TOPICS:
            count = topic_counts.get(topic, 0)
            validation.add(
                f'topic:{topic}',
                'PASS' if count > 0 else 'FAIL',
                f'消息数={count}（闭环试验必须大于0）')
    else:
        validation.add(
            'topic:/cmd_vel', 'PASS',
            f'消息数={cmd_count}（静态/仅感知记录不强制）')

    if result_dir is None:
        status = 'FAIL' if is_closed_loop else 'WARN'
        validation.add(
            'trial_result_directory', status,
            f'未找到对应CSV目录：{result_match_detail}')
        return validation, bag_dir, None, topic_counts

    validation.add(
        'trial_result_directory', 'PASS',
        f'{result_dir}（{result_match_detail}）')
    metadata_path = result_dir / 'metadata.json'
    validation.add(
        'result_metadata_json', 'PASS',
        f'JSON有效，trial_id={result_metadata.get("trial_id", "")}')

    identity_fields = (
        'trial_id', 'scenario', 'quality_aware', 'repeat_index')
    missing_identity = [
        name for name in identity_fields if name not in result_metadata]
    validation.add(
        'result_identity_fields',
        'FAIL' if missing_identity else 'PASS',
        ('缺少: ' + ', '.join(missing_identity))
        if missing_identity else
        ', '.join(
            f'{name}={result_metadata.get(name)}'
            for name in identity_fields))

    snapshots = result_metadata.get('parameter_snapshots', {})
    for component in ('controller', 'detector'):
        snapshot = snapshots.get(component)
        validation.add(
            f'parameter_snapshot:{component}',
            'PASS' if isinstance(snapshot, dict) and snapshot else 'FAIL',
            f'参数数={len(snapshot) if isinstance(snapshot, dict) else 0}')

    controller_snapshot = snapshots.get('controller', {})
    headland_turn_enabled = (
        isinstance(controller_snapshot, dict)
        and controller_snapshot.get('enable_headland_turn') is True)
    if headland_turn_enabled:
        for topic in HEADLAND_CONTINUOUS_TOPICS:
            count = topic_counts.get(topic, 0)
            validation.add(
                f'topic:{topic}',
                'PASS' if count > 0 else 'FAIL',
                f'消息数={count}（掉头试验必须持续记录）')
        for topic in HEADLAND_EVENT_TOPICS:
            count = topic_counts.get(topic, 0)
            validation.add(
                f'topic:{topic}',
                'PASS' if count > 0 else 'WARN',
                (f'消息数={count}；该话题仅在对应状态触发后发布，'
                 '若本次掉头失败则允许为0，但必须保留该次试验'))

    if result_metadata.get('perception_method') == 'indoor_synthetic_rows':
        for topic in ('/indoor_test_stage', '/indoor_test_finished'):
            count = topic_counts.get(topic, 0)
            validation.add(
                f'topic:{topic}',
                'PASS' if count > 0 else 'FAIL',
                f'消息数={count}（室内模拟试验必须记录）')

    csv_path = result_dir / 'timeseries.csv'
    if not csv_path.is_file():
        validation.add('timeseries_csv', 'FAIL', f'缺少 {csv_path}')
        return validation, bag_dir, result_dir, topic_counts
    try:
        csv_summary = inspect_csv(csv_path, args.trial_id)
    except (OSError, ValueError) as error:
        validation.add('timeseries_csv', 'FAIL', error)
        return validation, bag_dir, result_dir, topic_counts

    row_count = csv_summary['row_count']
    validation.add(
        'timeseries_csv_rows',
        'PASS' if row_count >= 2 else 'FAIL',
        f'数据行数={row_count}')
    validation.add(
        'timeseries_trial_id',
        'PASS' if csv_summary['wrong_trial_rows'] == 0 else 'FAIL',
        f'不匹配行数={csv_summary["wrong_trial_rows"]}')

    max_odom_age = csv_summary['max_odom_age_s']
    if max_odom_age is None:
        validation.add(
            'odometry_continuity', 'FAIL',
            'CSV没有有效odom_age_s，无法确认连续性')
    else:
        validation.add(
            'odometry_continuity',
            'PASS' if max_odom_age <= 0.5 else 'FAIL',
            f'最大里程计数据年龄={max_odom_age:.3f}s（要求不超过0.5s）')

    csv_duration = csv_summary['duration_s']
    coverage_ok = bag_duration + 0.5 >= csv_duration
    validation.add(
        'bag_covers_csv_duration',
        'PASS' if coverage_ok else 'FAIL',
        f'bag={bag_duration:.3f}s，CSV={csv_duration:.3f}s，允许0.5s误差')

    local_valid = csv_summary['local_path_valid_rows']
    validation.add(
        'local_path_samples',
        'PASS' if local_valid > 0 else 'FAIL',
        f'具有局部中心线的CSV行数={local_valid}')
    return validation, bag_dir, result_dir, topic_counts


def write_report(validation, bag_dir, result_dir, args, topic_counts):
    report = {
        'trial_id': args.trial_id,
        'verdict': validation.verdict,
        'bag_dir': str(bag_dir),
        'result_dir': str(result_dir) if result_dir else None,
        'checks': validation.checks,
        'topic_counts': topic_counts,
    }
    report_path = bag_dir / 'field_validation_report.json'
    try:
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding='utf-8')
    except OSError as error:
        print(f'警告: 无法写入验收报告: {error}', file=sys.stderr)
        report_path = None
    return report_path


def print_report(validation, bag_dir, result_dir, report_path):
    print()
    print('========== 田间试验自动验收 ==========')
    for item in validation.checks:
        print(
            f'[{item["status"]}] {item["name"]}: {item["detail"]}')
    print('--------------------------------------')
    print(f'rosbag目录: {bag_dir}')
    print(f'CSV目录: {result_dir if result_dir else "未找到"}')
    if report_path:
        print(f'验收报告: {report_path}')
    labels = {
        'PASS': '通过',
        'WARN': '有警告，需人工确认',
        'FAIL': '不通过；保留数据并标记原因，不能删除',
    }
    print(f'最终结论: {validation.verdict}（{labels[validation.verdict]}）')
    print('======================================')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, help='rosbag目录')
    parser.add_argument('--trial-id', required=True)
    parser.add_argument('--workspace-root', required=True)
    parser.add_argument('--usb-mount', default='')
    args = parser.parse_args()

    validation, bag_dir, result_dir, topic_counts = run_validation(args)
    report_path = write_report(
        validation, bag_dir, result_dir, args, topic_counts)
    print_report(validation, bag_dir, result_dir, report_path)
    return 1 if validation.verdict == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
