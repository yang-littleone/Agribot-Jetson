#!/usr/bin/env python3
"""将记录器数据与视频/人工真值对齐，生成论文所需质量、风险和消融统计."""

import argparse
import csv
import glob
import json
import math
from collections import defaultdict
from pathlib import Path


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def average_ranks(values):
    order = sorted(range(len(values)), key=values.__getitem__)
    ranks = [0.0] * len(values)
    index = 0
    while index < len(order):
        end = index + 1
        while end < len(order) and values[order[end]] == values[order[index]]:
            end += 1
        rank = 0.5 * (index + end - 1) + 1.0
        for position in range(index, end):
            ranks[order[position]] = rank
        index = end
    return ranks


def pearson(left, right):
    if len(left) < 2:
        return None
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    numerator = sum(
        (x - left_mean) * (y - right_mean) for x, y in zip(left, right))
    denominator = math.sqrt(
        sum((x - left_mean) ** 2 for x in left) *
        sum((y - right_mean) ** 2 for y in right))
    return numerator / denominator if denominator > 1e-12 else None


def spearman(left, right):
    return pearson(average_ranks(left), average_ranks(right))


def roc_auc(labels, scores):
    positives = sum(labels)
    negatives = len(labels) - positives
    if positives == 0 or negatives == 0:
        return None
    ranks = average_ranks(scores)
    positive_rank_sum = sum(rank for rank, label in zip(ranks, labels) if label)
    return (
        positive_rank_sum - positives * (positives + 1) / 2
    ) / (positives * negatives)


def average_precision(labels, scores):
    positives = sum(labels)
    if positives == 0:
        return None
    ordered = sorted(
        zip(scores, labels), key=lambda item: item[0], reverse=True)
    hits = 0
    total = 0.0
    for index, (_, label) in enumerate(ordered, start=1):
        if label:
            hits += 1
            total += hits / index
    return total / positives


def error_metrics(rows):
    lateral = [
        abs(row['lateral_error_m']) for row in rows
        if row.get('lateral_error_m') is not None
    ]
    heading = [
        abs(row['heading_error_deg']) for row in rows
        if row.get('heading_error_deg') is not None
    ]
    if not lateral:
        return {'samples': 0}
    valid_flags = [
        row['valid'] for row in rows if row.get('valid') is not None
    ]
    ordered_offsets = [
        row['center_offset_m'] for row in sorted(
            rows, key=lambda item: item['time_s'])
        if row.get('center_offset_m') is not None
    ]
    jitter = [
        abs(current - previous)
        for previous, current in zip(ordered_offsets, ordered_offsets[1:])
    ]
    return {
        'samples': len(lateral),
        'lateral_mae_m': sum(lateral) / len(lateral),
        'lateral_rmse_m': math.sqrt(sum(value * value for value in lateral) / len(lateral)),
        'lateral_p95_m': percentile(lateral, 0.95),
        'lateral_max_m': max(lateral),
        'heading_mae_deg': sum(heading) / len(heading) if heading else None,
        'heading_p95_deg': percentile(heading, 0.95),
        'valid_detection_rate': (
            sum(flag >= 0.5 for flag in valid_flags) / len(valid_flags)
            if valid_flags else None),
        'centerline_frame_jitter_mae_m': (
            sum(jitter) / len(jitter) if jitter else None),
        'centerline_frame_jitter_p95_m': percentile(jitter, 0.95),
    }


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def resolve_trial_files(patterns):
    resolved = []
    for pattern in patterns:
        path = Path(pattern)
        if path.is_dir():
            resolved.extend(path.glob('**/timeseries.csv'))
        else:
            resolved.extend(Path(item) for item in glob.glob(pattern))
    return sorted(set(item.resolve() for item in resolved))


def load_trials(paths):
    rows = []
    for path in paths:
        for raw in read_csv(path):
            row = dict(raw)
            row['_source'] = str(path)
            row['time_s'] = finite(raw.get('time_s'))
            for name in (
                    'published_confidence', 'raw_confidence', 'support_score',
                    'observation_score', 'width_score', 'residual_score',
                    'safety_score', 'center_offset_m', 'row_yaw_rad', 'valid',
                    'support_weight', 'observation_weight', 'width_weight',
                    'residual_weight', 'safety_weight'):
                row[name] = finite(raw.get(name))
            rows.append(row)
    return rows


def load_truth(path):
    required = {'trial_id', 'time_s', 'lateral_error_m', 'heading_error_deg'}
    rows = read_csv(path)
    missing = required - set(rows[0] if rows else [])
    if missing:
        raise ValueError(f'真值CSV缺少字段: {sorted(missing)}')
    grouped = defaultdict(list)
    for raw in rows:
        grouped[raw['trial_id']].append({
            'time_s': float(raw['time_s']),
            'lateral_error_m': float(raw['lateral_error_m']),
            'heading_error_deg': float(raw['heading_error_deg']),
            'plant_contact': int(raw.get('plant_contact', '0') or 0),
            'intervention': int(raw.get('intervention', '0') or 0),
            'completed': int(raw.get('completed', '0') or 0),
        })
    for trial_rows in grouped.values():
        trial_rows.sort(key=lambda row: row['time_s'])
    return grouped


def align_truth(trial_rows, truth_rows, tolerance):
    aligned = []
    cursor = 0
    for row in sorted(trial_rows, key=lambda item: item['time_s']):
        time_s = row['time_s']
        if time_s is None or not truth_rows:
            continue
        while (cursor + 1 < len(truth_rows) and
               abs(truth_rows[cursor + 1]['time_s'] - time_s) <=
               abs(truth_rows[cursor]['time_s'] - time_s)):
            cursor += 1
        truth = truth_rows[cursor]
        if abs(truth['time_s'] - time_s) <= tolerance:
            aligned.append({**row, **truth})
    return aligned


def confidence_report(rows, safe_threshold):
    usable = [
        row for row in rows
        if row.get('raw_confidence') is not None and
        row.get('lateral_error_m') is not None
    ]
    if not usable:
        return {'samples': 0}
    confidence = [row['raw_confidence'] for row in usable]
    lateral = [abs(row['lateral_error_m']) for row in usable]
    heading = [abs(row['heading_error_deg']) for row in usable]
    labels = [int(error > safe_threshold) for error in lateral]
    danger_scores = [1.0 - value for value in confidence]
    curve = []
    for threshold_index in range(21):
        threshold = threshold_index / 20.0
        predicted = [score >= threshold for score in danger_scores]
        true_positive = sum(
            prediction and label for prediction, label in zip(predicted, labels))
        false_positive = sum(
            prediction and not label for prediction, label in zip(predicted, labels))
        false_negative = sum(
            not prediction and label for prediction, label in zip(predicted, labels))
        true_negative = len(labels) - true_positive - false_positive - false_negative
        curve.append({
            'danger_score_threshold': threshold,
            'true_positive_rate': (
                true_positive / (true_positive + false_negative)
                if true_positive + false_negative else None),
            'false_positive_rate': (
                false_positive / (false_positive + true_negative)
                if false_positive + true_negative else None),
            'precision': (
                true_positive / (true_positive + false_positive)
                if true_positive + false_positive else None),
            'recall': (
                true_positive / (true_positive + false_negative)
                if true_positive + false_negative else None),
        })
    bins = {}
    for label, low, high in (
            ('low', 0.0, 0.45), ('medium', 0.45, 0.75), ('high', 0.75, 1.01)):
        selected = [
            error for error, score in zip(lateral, confidence)
            if low <= score < high
        ]
        bins[label] = {
            'samples': len(selected),
            'mae_m': sum(selected) / len(selected) if selected else None,
            'p95_m': percentile(selected, 0.95),
            'risk_rate': (
                sum(error > safe_threshold for error in selected) / len(selected)
                if selected else None),
        }
    return {
        'samples': len(usable),
        'safe_error_threshold_m': safe_threshold,
        'spearman_confidence_vs_abs_lateral_error': spearman(confidence, lateral),
        'spearman_confidence_vs_abs_heading_error': spearman(confidence, heading),
        'risk_events': sum(labels),
        'roc_auc_using_one_minus_confidence': roc_auc(labels, danger_scores),
        'average_precision_using_one_minus_confidence': average_precision(
            labels, danger_scores),
        'roc_pr_curve_points': curve,
        'confidence_bins': bins,
    }


def sensitivity_report(rows):
    usable = [
        row for row in rows
        if all(row.get(name) is not None for name in (
            'support_score', 'observation_score', 'width_score',
            'residual_score', 'safety_score', 'lateral_error_m'))
    ]
    if not usable:
        return {'samples': 0}
    names = ('support', 'observation', 'width', 'residual', 'safety')
    components = (
        'support_score', 'observation_score', 'width_score', 'residual_score',
        'safety_score')
    weight_fields = (
        'support_weight', 'observation_weight', 'width_weight',
        'residual_weight', 'safety_weight')
    first_weights = [usable[0].get(field) for field in weight_fields]
    base = (
        first_weights if all(weight is not None for weight in first_weights)
        else [0.30, 0.20, 0.20, 0.15, 0.15])
    errors = [abs(row['lateral_error_m']) for row in usable]
    result = {'samples': len(usable), 'variants': {}}
    for index, name in enumerate(names):
        for scale in (0.8, 1.2):
            weights = list(base)
            weights[index] *= scale
            total = sum(weights)
            weights = [weight / total for weight in weights]
            scores = [
                sum(weight * row[component] for weight, component in zip(weights, components))
                for row in usable
            ]
            result['variants'][f'{name}_{int(scale * 100)}pct'] = {
                'weights': dict(zip(names, weights)),
                'spearman_vs_abs_lateral_error': spearman(scores, errors),
            }
    return result


def closed_loop_metrics(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row['trial_id']].append(row)
    contacts = 0
    interventions = 0
    completed = 0
    runtimes = []
    lost_durations = []
    for trial_rows in grouped.values():
        contacts += max((row['plant_contact'] for row in trial_rows), default=0)
        interventions += max((row['intervention'] for row in trial_rows), default=0)
        completed_from_state = any(
            row.get('navigation_safety_state') in {
                'STOP_DISTANCE_LIMIT', 'STOP_TIME_LIMIT', 'STOP_PATH_END'}
            for row in trial_rows)
        completed_from_truth = any(
            row.get('completed', 0) for row in trial_rows)
        completed += int(completed_from_state or completed_from_truth)
        times = [
            row['time_s'] for row in trial_rows if row.get('time_s') is not None
        ]
        if times:
            runtimes.append(max(times) - min(times))
        ordered = sorted(
            (row for row in trial_rows if row.get('time_s') is not None),
            key=lambda row: row['time_s'])
        lost_duration = 0.0
        for previous, current in zip(ordered, ordered[1:]):
            if previous.get('valid') is not None and previous['valid'] < 0.5:
                lost_duration += min(0.5, current['time_s'] - previous['time_s'])
        lost_durations.append(lost_duration)
    return {
        **error_metrics(rows),
        'trial_count': len(grouped),
        'completed_trials': completed,
        'completion_rate': completed / len(grouped) if grouped else None,
        'plant_contacts': contacts,
        'interventions': interventions,
        'mean_lost_line_duration_s': (
            sum(lost_durations) / len(lost_durations) if lost_durations else None),
        'mean_runtime_s': sum(runtimes) / len(runtimes) if runtimes else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--trials', nargs='+', required=True,
        help='timeseries.csv、通配符或包含试验目录的根目录')
    parser.add_argument('--ground-truth', required=True)
    parser.add_argument('--safe-threshold', required=True, type=float)
    parser.add_argument('--time-tolerance', type=float, default=0.08)
    parser.add_argument('--output', default='field_analysis_report.json')
    args = parser.parse_args()

    trial_files = resolve_trial_files(args.trials)
    if not trial_files:
        raise SystemExit('没有找到 timeseries.csv')
    trials = load_trials(trial_files)
    truth = load_truth(args.ground_truth)
    grouped = defaultdict(list)
    for row in trials:
        grouped[row['trial_id']].append(row)

    aligned = []
    missing_truth = []
    for trial_id, trial_rows in grouped.items():
        if trial_id not in truth:
            missing_truth.append(trial_id)
            continue
        aligned.extend(align_truth(
            trial_rows, truth[trial_id], args.time_tolerance))

    by_method = defaultdict(list)
    by_closed_loop_group = defaultdict(list)
    for row in aligned:
        if row.get('experiment_type') == 'perception_ablation':
            by_method[row.get('perception_method') or 'unspecified'].append(row)
        if row.get('experiment_type') == 'closed_loop':
            group = (
                row.get('scenario') or 'unspecified',
                row.get('quality_aware') or 'unspecified',
                row.get('nominal_speed_mps') or 'unspecified',
            )
            by_closed_loop_group['|'.join(group)].append(row)

    full_method_rows = [
        row for row in aligned
        if (row.get('perception_method') or 'full') == 'full'
    ]

    report = {
        'input_files': [str(path) for path in trial_files],
        'aligned_samples': len(aligned),
        'trials_without_truth': sorted(missing_truth),
        'perception_ablation': {
            method: error_metrics(rows) for method, rows in sorted(by_method.items())
        },
        'quality_validity': confidence_report(
            full_method_rows, args.safe_threshold),
        'single_weight_sensitivity': sensitivity_report(full_method_rows),
        'closed_loop_groups': {
            group: closed_loop_metrics(rows)
            for group, rows in sorted(by_closed_loop_group.items())
        },
    }
    Path(args.output).write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding='utf-8')
    print(f'已写入 {args.output}，对齐样本 {len(aligned)} 条。')


if __name__ == '__main__':
    main()
