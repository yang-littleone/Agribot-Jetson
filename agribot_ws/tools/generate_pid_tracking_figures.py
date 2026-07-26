#!/usr/bin/env python3
"""Generate reproducible figures for the fixed-path PID experiment report."""

import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
RESULTS = WORKSPACE / 'pid_tracking_results'
OUTPUT = WORKSPACE / 'docs' / 'experiments' / 'figures'


def load_runs():
    runs = []
    for report_path in sorted(RESULTS.glob('run_*/report.json')):
        report = json.loads(report_path.read_text(encoding='utf-8'))
        metrics = report['metrics']
        params = report['current_parameters']
        valid = (
            metrics['final_endpoint_error_m'] <= 0.12
            and metrics['max_progress_m'] >= 0.9 * metrics['path_length_m']
        )
        runs.append({
            'id': report_path.parent.name.replace('run_', ''),
            'dir': report_path.parent,
            'metrics': metrics,
            'params': params,
            'valid': valid,
        })
    return runs


def speed_figure(runs):
    selected = [
        run for run in runs
        if run['valid'] and abs(run['params']['lateral_kp'] - 1.15) < 1e-9
    ]
    grouped = defaultdict(list)
    for run in selected:
        grouped[run['params']['max_linear_speed']].append(run)

    speeds = sorted(grouped)
    fig, ax = plt.subplots(figsize=(8.4, 4.8), constrained_layout=True)
    series = [
        ('rmse_m', 'RMSE', '#1565c0'),
        ('p95_m', 'P95 error', '#ef6c00'),
        ('max_error_m', 'Maximum error', '#c62828'),
    ]
    for key, label, color in series:
        means = []
        for speed in speeds:
            values = [run['metrics'][key] * 100 for run in grouped[speed]]
            ax.scatter([speed] * len(values), values, color=color, alpha=0.75, s=44)
            means.append(np.mean(values))
        ax.plot(speeds, means, color=color, linewidth=2.2, marker='o', label=label)

    ax.axhline(4, color='#1565c0', linestyle='--', linewidth=1, alpha=0.55)
    ax.axhline(7, color='#ef6c00', linestyle='--', linewidth=1, alpha=0.55)
    ax.axhline(8, color='#c62828', linestyle='--', linewidth=1, alpha=0.55)
    ax.set_title('Tracking error versus speed (lateral Kp = 1.15)')
    ax.set_xlabel('Maximum linear speed (m/s)')
    ax.set_ylabel('Lateral error (cm)')
    ax.set_xticks(speeds)
    ax.grid(True, alpha=0.25)
    ax.legend(ncol=3, loc='upper left')
    fig.savefig(OUTPUT / 'pid_tracking_error_vs_speed.png', dpi=180)
    plt.close(fig)


def kp_figure(runs):
    selected = [
        run for run in runs
        if run['valid'] and abs(run['params']['max_linear_speed'] - 0.35) < 1e-9
    ]
    grouped = defaultdict(list)
    for run in selected:
        grouped[run['params']['lateral_kp']].append(run)

    gains = sorted(grouped)
    metrics = [
        ('rmse_m', 'RMSE', '#1565c0'),
        ('p95_m', 'P95 error', '#ef6c00'),
        ('max_error_m', 'Maximum error', '#c62828'),
    ]
    fig, ax = plt.subplots(figsize=(8.4, 4.8), constrained_layout=True)
    offsets = [-0.16, 0.0, 0.16]
    for (key, label, color), offset in zip(metrics, offsets):
        x_values = np.arange(len(gains)) + offset
        means = [np.mean([run['metrics'][key] * 100 for run in grouped[gain]]) for gain in gains]
        ax.bar(x_values, means, width=0.15, color=color, alpha=0.65, label=label)
        for index, gain in enumerate(gains):
            values = [run['metrics'][key] * 100 for run in grouped[gain]]
            jitter = np.linspace(-0.025, 0.025, len(values))
            ax.scatter(np.full(len(values), x_values[index]) + jitter, values,
                       color=color, edgecolor='white', linewidth=0.6, zorder=3)

    ax.set_title('Kp sweep at 0.35 m/s')
    ax.set_xlabel('Lateral Kp')
    ax.set_ylabel('Lateral error (cm)')
    ax.set_xticks(np.arange(len(gains)), [f'{gain:.2f}' for gain in gains])
    ax.grid(True, axis='y', alpha=0.25)
    ax.legend(ncol=3, loc='upper left')
    fig.savefig(OUTPUT / 'pid_tracking_kp_sweep_035.png', dpi=180)
    plt.close(fig)


def profile_for_run(run, grid):
    rows = []
    with (run['dir'] / 'tracking_samples.csv').open(encoding='utf-8') as file:
        for row in csv.DictReader(file):
            rows.append(row)
    cutoff = next(
        (index for index, row in enumerate(rows)
         if float(row['endpoint_distance_m']) <= 0.12), len(rows) - 1)
    rows = rows[:cutoff + 1]
    progress = np.array([float(row['path_progress_m']) for row in rows])
    error = np.array([float(row['abs_cross_track_error_m']) * 100 for row in rows])
    order = np.argsort(progress)
    progress = progress[order] / run['metrics']['path_length_m']
    error = error[order]
    unique_progress, unique_indices = np.unique(progress, return_index=True)
    return np.interp(grid, unique_progress, error[unique_indices])


def profile_figure(runs):
    groups = {
        '0.30 m/s, Kp=1.15': [
            run for run in runs if run['valid']
            and abs(run['params']['max_linear_speed'] - 0.30) < 1e-9
            and abs(run['params']['lateral_kp'] - 1.15) < 1e-9
        ],
        '0.35 m/s, Kp=1.05': [
            run for run in runs if run['valid']
            and abs(run['params']['max_linear_speed'] - 0.35) < 1e-9
            and abs(run['params']['lateral_kp'] - 1.05) < 1e-9
        ],
    }
    colors = ['#1565c0', '#c62828']
    grid = np.linspace(0.0, 1.0, 121)
    fig, ax = plt.subplots(figsize=(8.4, 4.8), constrained_layout=True)
    for (label, group), color in zip(groups.items(), colors):
        profiles = np.array([profile_for_run(run, grid) for run in group])
        for profile in profiles:
            ax.plot(grid * 100, profile, color=color, alpha=0.18, linewidth=1)
        mean = np.mean(profiles, axis=0)
        ax.plot(grid * 100, mean, color=color, linewidth=2.4, label=label)
        ax.fill_between(grid * 100, np.min(profiles, axis=0), np.max(profiles, axis=0),
                        color=color, alpha=0.12)

    ax.set_title('Absolute cross-track error along the path')
    ax.set_xlabel('Path progress (%)')
    ax.set_ylabel('Absolute cross-track error (cm)')
    ax.set_xlim(0, 100)
    ax.grid(True, alpha=0.25)
    ax.legend(loc='upper left')
    fig.savefig(OUTPUT / 'pid_tracking_error_profiles.png', dpi=180)
    plt.close(fig)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    runs = load_runs()
    speed_figure(runs)
    kp_figure(runs)
    profile_figure(runs)


if __name__ == '__main__':
    main()
