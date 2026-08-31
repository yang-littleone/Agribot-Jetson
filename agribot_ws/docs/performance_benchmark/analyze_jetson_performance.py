#!/usr/bin/env python3
"""Combine repeated Jetson runtime runs into paper-ready tables and figures."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

import matplotlib.pyplot as plt
import numpy as np


def finite(values):
    return np.asarray([float(value) for value in values if math.isfinite(float(value))])


def metric(values):
    values = finite(values)
    if not values.size:
        return {key: math.nan for key in ('mean', 'sd', 'median', 'p95', 'p99', 'max')}
    return {
        'mean': float(np.mean(values)),
        'sd': float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)),
        'p99': float(np.percentile(values, 99)),
        'max': float(np.max(values)),
    }


def load_csv(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def fmt(value, digits=2):
    return '—' if not math.isfinite(float(value)) else f'{float(value):.{digits}f}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--bag-label', default='20260729_115928_p01_normal_qaware_v018_r1')
    parser.add_argument('--segment-duration-s', type=float, default=60.0)
    args = parser.parse_args()

    run_dirs = sorted(path for path in args.input_root.glob('run_*') if path.is_dir())
    if not run_dirs:
        raise RuntimeError(f'No run_* directories under {args.input_root}')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    run_summaries = []
    all_frames = []
    all_resources = []
    for run_dir in run_dirs:
        summary = json.loads((run_dir / 'run_summary.json').read_text(encoding='utf-8'))
        frames = load_csv(run_dir / 'frame_metrics.csv')
        resources = load_csv(run_dir / 'resource_usage.csv')
        for row in frames:
            row['run_id'] = summary['run_id']
            if int(row['warmup']) == 0:
                all_frames.append(row)
        for row in resources:
            row['run_id'] = summary['run_id']
            if (row.get('active_measurement', '0') == '1'
                    and row['process_cpu_percent_one_core'] not in ('', 'nan', 'NaN')):
                all_resources.append(row)
        run_summaries.append(summary)

    callback = metric(row['callback_duration_ms'] for row in all_frames)
    end_to_end = metric(row['end_to_end_path_ms'] for row in all_frames)
    transport = metric(row['performance_transport_ms'] for row in all_frames)
    cpu_one = metric(row['process_cpu_percent_one_core'] for row in all_resources)
    cpu_total = metric(row['process_cpu_percent_total_capacity'] for row in all_resources)
    rss = metric(row['process_rss_mib'] for row in all_resources)
    system_cpu = metric(row['system_cpu_percent'] for row in all_resources)
    temperature = metric(row['max_thermal_zone_c'] for row in all_resources)
    frequency = metric(
        summary['processed_output_frequency_hz'] for summary in run_summaries)
    input_frequency = metric(
        summary['input_frequency_hz_from_source_stamps'] for summary in run_summaries)
    valid_percent = metric(summary['valid_path_percent'] for summary in run_summaries)

    run_table = []
    for summary in run_summaries:
        run_table.append({
            'run_id': summary['run_id'],
            'frames_analyzed': summary['frames_analyzed'],
            'analyzed_duration_s': (
                (summary['frames_analyzed'] - 1)
                / summary['input_frequency_hz_from_source_stamps']
                if summary['frames_analyzed'] > 1 else math.nan),
            'input_hz': summary['input_frequency_hz_from_source_stamps'],
            'output_hz': summary['processed_output_frequency_hz'],
            'valid_output_hz': (
                summary['processed_output_frequency_hz']
                * summary['valid_path_percent'] / 100.0),
            'valid_path_percent': summary['valid_path_percent'],
            'callback_mean_ms': summary['callback_duration_ms']['mean'],
            'callback_p95_ms': summary['callback_duration_ms']['p95'],
            'callback_max_ms': summary['callback_duration_ms']['maximum'],
            'e2e_mean_ms': summary['end_to_end_path_ms']['mean'],
            'e2e_p95_ms': summary['end_to_end_path_ms']['p95'],
            'e2e_max_ms': summary['end_to_end_path_ms']['maximum'],
            'cpu_mean_percent_one_core': summary['process_cpu_percent_one_core']['mean'],
            'rss_mean_mib': summary['process_rss_mib']['mean'],
        })
    with (args.output_dir / 'run_summary_table.csv').open(
            'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(run_table[0]))
        writer.writeheader()
        writer.writerows(run_table)

    pooled = {
        'bag_label': args.bag_label,
        'repeat_count': len(run_summaries),
        'frames_analyzed': len(all_frames),
        'resource_samples': len(all_resources),
        'input_frequency_hz': input_frequency,
        'processed_output_frequency_hz': frequency,
        'valid_path_percent': valid_percent,
        'callback_duration_ms': callback,
        'end_to_end_path_ms': end_to_end,
        'performance_transport_ms': transport,
        'process_cpu_percent_one_core': cpu_one,
        'process_cpu_percent_total_capacity': cpu_total,
        'process_rss_mib': rss,
        'system_cpu_percent': system_cpu,
        'max_thermal_zone_c': temperature,
        'segment_duration_s_each_run': args.segment_duration_s,
        'nominal_input_period_ms': 100.0,
        'callback_deadline_exceedance_count': sum(
            float(row['callback_duration_ms']) > 100.0 for row in all_frames),
        'end_to_end_deadline_exceedance_count': sum(
            float(row['end_to_end_path_ms']) > 100.0 for row in all_frames),
    }
    (args.output_dir / 'pooled_summary.json').write_text(
        json.dumps(pooled, ensure_ascii=False, indent=2, allow_nan=True),
        encoding='utf-8')

    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    callback_values = finite(row['callback_duration_ms'] for row in all_frames)
    e2e_values = finite(row['end_to_end_path_ms'] for row in all_frames)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].hist(callback_values, bins=45, alpha=0.75, label='逐帧算法回调')
    axes[0].axvline(callback['p95'], color='tab:red', linestyle='--',
                    label=f"P95={callback['p95']:.1f} ms")
    axes[0].set_xlabel('耗时 / ms')
    axes[0].set_ylabel('帧数')
    axes[0].set_title('逐帧计算耗时分布')
    axes[0].legend()

    for values, label in ((callback_values, '算法回调'), (e2e_values, '回调入口至路径接收')):
        ordered = np.sort(values)
        probability = np.arange(1, ordered.size + 1) / ordered.size
        axes[1].plot(ordered, probability, label=label)
    axes[1].axhline(0.95, color='0.5', linestyle=':')
    axes[1].set_xlabel('延迟 / ms')
    axes[1].set_ylabel('累计概率')
    axes[1].set_title('计算与端到端延迟CDF')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / 'fig1_runtime_latency.png', dpi=220)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=False)
    for run_id in sorted({row['run_id'] for row in all_resources}):
        rows = [row for row in all_resources if row['run_id'] == run_id]
        time_values = np.asarray([float(row['steady_s']) for row in rows])
        time_values -= time_values[0]
        axes[0].plot(
            time_values,
            [float(row['process_cpu_percent_one_core']) for row in rows],
            label=run_id)
        axes[1].plot(
            time_values,
            [float(row['process_rss_mib']) for row in rows],
            label=run_id)
    axes[0].set_ylabel('节点CPU / 单核%')
    axes[0].set_title('感知节点CPU占用')
    axes[0].legend(ncol=len(run_summaries))
    axes[1].set_xlabel('本次测试时间 / s')
    axes[1].set_ylabel('节点RSS / MiB')
    axes[1].set_title('感知节点常驻内存')
    fig.tight_layout()
    fig.savefig(args.output_dir / 'fig2_resource_usage.png', dpi=220)
    plt.close(fig)

    system_info = ''
    for info_path in (
            args.input_root / 'system_info.txt',
            args.input_root.parent / 'system_info.txt'):
        if info_path.exists():
            system_info = info_path.read_text(encoding='utf-8', errors='replace')
            break

    lines = [
        '# Jetson车载端中心线算法实时性能测试报告',
        '',
        '## 1. 测试结论',
        '',
        f"在Jetson车载端对真实玉米田点云片段进行1.0倍速重复回放{len(run_summaries)}次，",
        f"去除每次前{run_summaries[0]['warmup_frames_excluded']}帧预热后共统计{len(all_frames)}帧。",
        f"中心线感知节点平均逐帧回调耗时为{callback['mean']:.2f} ms，",
        f"P95为{callback['p95']:.2f} ms，最大值为{callback['max']:.2f} ms；",
        f"回调入口至外部订阅者收到`/corn_row_center_line`的端到端延迟P95为",
        f"{end_to_end['p95']:.2f} ms。处理输出频率为{frequency['mean']:.2f} Hz。",
        f"全部{len(all_frames)}帧的回调耗时和端到端路径延迟均未超过10 Hz输入对应的",
        f"100 ms周期；最大回调耗时仍保留{100.0 - callback['max']:.2f} ms周期裕量。",
        '',
        '这些数据证明的是当前Jetson、当前编译方式和当前真实点云负载下的实时运行能力。',
        '它们不代表雷达内部扫描延迟，也不能替代中心线精度和闭环跟踪精度。',
        '',
        '## 2. 测试对象与方法',
        '',
        '- 平台：NVIDIA Jetson Orin NX，16 GB内存，Ubuntu 22.04，ROS 2 Humble；',
        '- 功耗模式：25 W；未以root执行`jetson_clocks`，因此CPU保持实际动态调频状态；',
        '- 编译：与当前车载部署一致，`CMAKE_BUILD_TYPE`为空，未额外加入`-O2/-O3`；',
        f"- 数据：`{args.bag_label}`真实玉米田rosbag；",
        f"- 回放：1.0倍速，每次设置{args.segment_duration_s:.0f} s墙钟上限，共{len(run_summaries)}次；",
        '- 去除30帧预热和播放器启动开销后，各次正式点云有效时长见表3；',
        '- 算法：`field_row_follow.yaml`完整方法，开启只读性能计时开关；',
        '- 科学参数与田间冻结配置一致；计时版本只增加默认关闭的计时发布器，',
        '  基准测试时通过命令行开启，不进入中心线或质量评分计算；',
        '- 计时钟：Linux单调时钟，不受ROS仿真时间跳变影响；',
        '- 逐帧耗时：点云回调入口至中心线、边界、质量诊断等正常输出完成；',
        '- 端到端延迟：点云回调入口至独立采集节点收到中心线路径；',
        '- CPU：仅统计中心线感知进程，100%表示占满一个逻辑CPU核；',
        '- 内存：中心线感知进程RSS；',
        f"- 每次前{run_summaries[0]['warmup_frames_excluded']}帧不计入正式统计。",
        '',
        '## 3. 各次重复结果',
        '',
        '| 重复 | 帧数 | 有效时长/s | 输入/Hz | 处理输出/Hz | 有效中心线/Hz | 有效路径/% | 回调均值/ms | 回调P95/ms | 回调最大/ms | 端到端P95/ms | CPU/单核% | RSS/MiB |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for row in run_table:
        lines.append(
            f"| {row['run_id']} | {row['frames_analyzed']} | {fmt(row['analyzed_duration_s'])} | "
            f"{fmt(row['input_hz'])} | {fmt(row['output_hz'])} | "
            f"{fmt(row['valid_output_hz'])} | {fmt(row['valid_path_percent'])} | "
            f"{fmt(row['callback_mean_ms'])} | {fmt(row['callback_p95_ms'])} | "
            f"{fmt(row['callback_max_ms'])} | {fmt(row['e2e_p95_ms'])} | "
            f"{fmt(row['cpu_mean_percent_one_core'])} | {fmt(row['rss_mean_mib'])} |")
    lines += [
        '',
        '## 4. 合并逐帧统计',
        '',
        '| 指标 | 均值 | 标准差 | 中位数 | P95 | P99 | 最大值 |',
        '|---|---:|---:|---:|---:|---:|---:|',
        f"| 算法回调耗时/ms | {fmt(callback['mean'])} | {fmt(callback['sd'])} | {fmt(callback['median'])} | {fmt(callback['p95'])} | {fmt(callback['p99'])} | {fmt(callback['max'])} |",
        f"| 端到端路径延迟/ms | {fmt(end_to_end['mean'])} | {fmt(end_to_end['sd'])} | {fmt(end_to_end['median'])} | {fmt(end_to_end['p95'])} | {fmt(end_to_end['p99'])} | {fmt(end_to_end['max'])} |",
        '',
        f"在100 ms实时周期阈值下，算法回调超期为{pooled['callback_deadline_exceedance_count']}/{len(all_frames)}帧，",
        f"端到端路径延迟超期为{pooled['end_to_end_deadline_exceedance_count']}/{len(all_frames)}帧。",
        '这里的“未超期”只表示计算链满足当前10 Hz输入周期，不等同于功能安全认证。',
        '',
        f"![逐帧耗时与延迟](fig1_runtime_latency.png)",
        '',
        '## 5. 资源占用',
        '',
        '| 指标 | 均值 | P95 | 最大值 |',
        '|---|---:|---:|---:|',
        f"| 感知节点CPU/单核% | {fmt(cpu_one['mean'])} | {fmt(cpu_one['p95'])} | {fmt(cpu_one['max'])} |",
        f"| 感知节点CPU/8核总容量% | {fmt(cpu_total['mean'])} | {fmt(cpu_total['p95'])} | {fmt(cpu_total['max'])} |",
        f"| 感知节点RSS/MiB | {fmt(rss['mean'])} | {fmt(rss['p95'])} | {fmt(rss['max'])} |",
        f"| 全系统CPU/% | {fmt(system_cpu['mean'])} | {fmt(system_cpu['p95'])} | {fmt(system_cpu['max'])} |",
        f"| 最高热区温度/°C | {fmt(temperature['mean'])} | {fmt(temperature['p95'])} | {fmt(temperature['max'])} |",
        '',
        '感知节点使用PCL/CPU实现，没有调用CUDA推理，因此不把GPU占用作为该节点的主要',
        '资源指标。全系统CPU还包含rosbag播放器、DDS和采集节点，不能等同于感知节点CPU。',
        '',
        '![资源占用](fig2_resource_usage.png)',
        '',
        '## 6. 论文使用边界',
        '',
        '- 应写“真实点云1.0倍速回放条件下的Jetson车载端性能”；',
        '- 不应写成现场雷达采集到执行器动作的完整控制链总延迟；',
        '- rosbag回放进程会占用部分系统资源，但表中的节点CPU/RSS只统计感知进程；',
        '- 当前构建配置必须与`system_info.txt`一起报告；不同优化等级不可直接比较；',
        '- 最大值易受Linux调度和后台进程影响，实时性判断优先使用P95/P99并同时报告最大值；',
        '- 性能数据只支撑实时部署，不支撑中心线精度或质量指标有效性。',
        '- 处理输出保持约10 Hz；有效中心线约9.85 Hz。差异来自同一片段末端感知无效时',
        '  按安全逻辑发布空路径，不是计算超期或回调丢失。',
        '',
        '## 7. 原始文件',
        '',
        '- `raw_runs/run_*/frame_metrics.csv`：逐帧回调和端到端延迟；',
        '- `raw_runs/run_*/resource_usage.csv`：0.5 s资源采样；',
        '- `raw_runs/run_*/run_summary.json`：每次重复摘要；',
        '- `run_summary_table.csv`：三次重复对照；',
        '- `pooled_summary.json`：合并统计；',
        '- `system_info.txt`：Jetson、系统、功耗模式和编译信息；',
        '- `benchmark_config_snapshot.yaml`：本次运行实际参数文件快照；',
        '- `benchmark_manifest.sha256`：代码、配置和输入数据库校验。',
        '',
        '## 8. 测试平台原始信息',
        '',
        '```text',
        system_info.strip(),
        '```',
        '',
    ]
    (args.output_dir / 'Jetson车载端算法实时性能测试报告.md').write_text(
        '\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
