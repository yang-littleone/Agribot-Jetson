#!/usr/bin/env python3
"""Summarize the runtime benchmark over the paper's 30 formal field bags."""

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_csv(path):
    with path.open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def clean(values):
    return np.asarray([float(v) for v in values if math.isfinite(float(v))])


def metrics(values):
    data = clean(values)
    if not len(data):
        return {name: math.nan for name in ('n', 'mean', 'median', 'p95', 'p99', 'max')}
    return {'n': int(len(data)), 'mean': float(np.mean(data)), 'median': float(np.median(data)),
            'p95': float(np.percentile(data, 95)), 'p99': float(np.percentile(data, 99)),
            'max': float(np.max(data))}


def parse_tegrastats(path):
    # tegrastats reports instantaneous/averaged VDD_IN power as e.g. 7450mW/7221mW.
    power, temp, throttle_lines = [], [], []
    if not path.is_file():
        return power, temp, throttle_lines
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        found_power = re.search(r'VDD_IN\s+(\d+)mW/', line)
        if found_power:
            power.append(float(found_power.group(1)) / 1000.0)
        temp.extend(float(v) for v in re.findall(r'\b\w+@([0-9.]+)C', line))
        if re.search(r'thrott|thermal.{0,12}(limit|thrott)', line, re.I):
            throttle_lines.append(line)
    return power, temp, throttle_lines


def fmt(value, digits=2):
    return '—' if not math.isfinite(float(value)) else f'{float(value):.{digits}f}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--tegrastats-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    manifest = read_csv(args.manifest)
    if len(manifest) != 30:
        raise RuntimeError(f'需要30条正式试验清单，实际为{len(manifest)}')
    rows_by_bag = {row['bag_dir']: row for row in manifest}
    run_rows, per_frame = [], defaultdict(list)
    missing = []
    for index, bag in enumerate(rows_by_bag, 1):
        run_dir = args.input_root / f'{index}_{bag}'
        summary_path, frames_path = run_dir / 'run_summary.json', run_dir / 'frame_metrics.csv'
        if not summary_path.is_file() or not frames_path.is_file():
            missing.append(bag)
            continue
        summary = json.loads(summary_path.read_text(encoding='utf-8'))
        source = rows_by_bag[bag]
        formal = [r for r in read_csv(frames_path) if r['warmup'] == '0']
        for r in formal:
            r['bag_dir'] = bag
            r['group_id'] = source['group_id']
            per_frame[source['group_id']].append(r)
        run_rows.append({
            **source,
            'frames_analyzed': summary['frames_analyzed'],
            'valid_path_percent': summary['valid_path_percent'],
            'input_hz': summary['input_frequency_hz_from_source_stamps'],
            'output_hz': summary['processed_output_frequency_hz'],
            'callback_mean_ms': summary['callback_duration_ms']['mean'],
            'callback_p95_ms': summary['callback_duration_ms']['p95'],
            'callback_max_ms': summary['callback_duration_ms']['maximum'],
            'e2e_p95_ms': summary['end_to_end_path_ms']['p95'],
            'cpu_one_core_percent': summary['process_cpu_percent_one_core']['mean'],
            'rss_mib': summary['process_rss_mib']['mean'],
        })
    if missing:
        raise RuntimeError('以下正式包缺少完整结果：' + ', '.join(missing))

    with (args.output_dir / 'per_bag_runtime_summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(run_rows[0]))
        writer.writeheader(); writer.writerows(run_rows)

    group_table = []
    all_frames = [f for values in per_frame.values() for f in values]
    for group_id, values in per_frame.items():
        group_table.append({
            'group_id': group_id,
            'trial_count': sum(r['group_id'] == group_id for r in run_rows),
            'frames': len(values),
            'callback_mean_ms': metrics(r['callback_duration_ms'] for r in values)['mean'],
            'callback_p95_ms': metrics(r['callback_duration_ms'] for r in values)['p95'],
            'callback_max_ms': metrics(r['callback_duration_ms'] for r in values)['max'],
            'e2e_p95_ms': metrics(r['end_to_end_path_ms'] for r in values)['p95'],
            'valid_path_percent': 100 * sum(r['valid_path'] == '1' for r in values) / len(values),
        })
    group_table.sort(key=lambda row: row['group_id'])
    with (args.output_dir / 'per_group_runtime_summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(group_table[0]))
        writer.writeheader(); writer.writerows(group_table)

    callback = metrics(r['callback_duration_ms'] for r in all_frames)
    e2e = metrics(r['end_to_end_path_ms'] for r in all_frames)
    valid_pct = 100 * sum(r['valid_path'] == '1' for r in all_frames) / len(all_frames)
    power, temperature, throttle = parse_tegrastats(args.tegrastats_log)
    overview = {
        'formal_trial_count': len(run_rows), 'frames_analyzed': len(all_frames),
        'callback_duration_ms': callback, 'end_to_end_path_ms': e2e,
        'valid_path_percent': valid_pct,
        'callback_over_100ms_count': sum(float(r['callback_duration_ms']) > 100 for r in all_frames),
        'end_to_end_over_100ms_count': sum(float(r['end_to_end_path_ms']) > 100 for r in all_frames),
        'tegrastats_vdd_in_power_w': metrics(power),
        'tegrastats_all_reported_temperatures_c': metrics(temperature),
        'thermal_throttle_log_line_count': len(throttle),
        'thermal_throttle_log_lines': throttle[:20],
    }
    (args.output_dir / 'formal_30_overview.json').write_text(
        json.dumps(overview, ensure_ascii=False, indent=2, allow_nan=True), encoding='utf-8')

    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    labels = [row['group_id'] for row in group_table]
    values = [[float(r['callback_duration_ms']) for r in per_frame[label]] for label in labels]
    ax.boxplot(values, labels=labels, showfliers=False)
    ax.axhline(100, color='tab:red', linestyle='--', label='10 Hz周期：100 ms')
    ax.set_ylabel('逐帧回调耗时 / ms'); ax.set_title('30个正式田间包的逐帧算法耗时')
    ax.tick_params(axis='x', rotation=25); ax.legend(); fig.tight_layout()
    fig.savefig(args.output_dir / 'fig1_formal30_callback_by_group.png', dpi=220); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    axes[0].hist([float(r['callback_duration_ms']) for r in all_frames], bins=55, alpha=.8)
    axes[0].axvline(callback['p95'], color='tab:red', linestyle='--', label=f'P95={callback["p95"]:.1f} ms')
    axes[0].set_xlabel('回调耗时 / ms'); axes[0].set_ylabel('帧数'); axes[0].legend()
    if power:
        axes[1].plot(power, linewidth=.65, label='VDD_IN瞬时功率')
        axes[1].set_ylabel('功率 / W'); axes[1].set_xlabel('tegrastats采样序号（0.5 s）')
        axes[1].set_title('全30包连续回放期间的输入功率')
    else:
        axes[1].text(.5, .5, '未采集到tegrastats功率字段', ha='center')
    fig.tight_layout(); fig.savefig(args.output_dir / 'fig2_latency_and_power.png', dpi=220); plt.close(fig)

    lines = [
        '# Jetson车载端：30个正式田间试验包实时性验证', '',
        '## 结论', '',
        f'按论文测量清单逐一回放30个正式20 m田间rosbag（6组×5次），去除每包前30帧预热后共统计{len(all_frames)}帧。',
        f'当前中心线感知节点逐帧回调均值为{fmt(callback["mean"])} ms，P95为{fmt(callback["p95"])} ms，最大值为{fmt(callback["max"])} ms；',
        f'回调入口至外部节点接收中心线路径的端到端P95为{fmt(e2e["p95"])} ms。',
        f'回调/端到端超过100 ms（10 Hz输入周期）的帧数分别为{overview["callback_over_100ms_count"]}/{overview["end_to_end_over_100ms_count"]}。',
        f'有效中心线输出比例为{valid_pct:.2f}%。', '',
        '## 覆盖范围与边界', '',
        '- 数据严格限于小论文的30个正式田间20 m试验包；输入来自U盘，原始包未被写入。',
        '- 六组为p01/p02、0.18/0.40 m/s和原试验控制策略的组合。控制策略是原始车载试验属性；此回放仅评估当前中心线感知节点计算负载，不重新执行底盘闭环控制。',
        '- 连续运行过程中通过tegrastats采集VDD_IN功率及温度字段。没有出现包含“thrott”或热限制关键词的日志行，并不能等同于硬件厂商级的节流诊断；它是本次可重复测试条件下的运行监测证据。', '',
        '## 分组结果', '',
        '|组别|包数|正式帧数|回调均值/ms|回调P95/ms|最大/ms|端到端P95/ms|有效中心线/%|',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for row in group_table:
        lines.append('|{group_id}|{trial_count}|{frames}|{mean}|{p95}|{max}|{e2e}|{valid:.2f}|'.format(
            group_id=row['group_id'], trial_count=row['trial_count'], frames=row['frames'],
            mean=fmt(row['callback_mean_ms']), p95=fmt(row['callback_p95_ms']),
            max=fmt(row['callback_max_ms']), e2e=fmt(row['e2e_p95_ms']), valid=row['valid_path_percent']))
    lines.extend(['', '## 连续功耗与温度', '',
                  f'- VDD_IN功率：均值 {fmt(overview["tegrastats_vdd_in_power_w"]["mean"])} W，P95 {fmt(overview["tegrastats_vdd_in_power_w"]["p95"])} W，最大 {fmt(overview["tegrastats_vdd_in_power_w"]["max"])} W。',
                  f'- tegrastats报告的全部温度字段：最大 {fmt(overview["tegrastats_all_reported_temperatures_c"]["max"])} °C。',
                  f'- 日志中热节流关键词命中：{len(throttle)} 行。', '',
                  '原始逐包帧级数据在`raw_runs/`，逐包/分组汇总分别见`per_bag_runtime_summary.csv`和`per_group_runtime_summary.csv`。'])
    (args.output_dir / 'Jetson_30正式田间包实时性测试报告.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
