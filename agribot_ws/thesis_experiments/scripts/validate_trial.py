#!/usr/bin/env python3
"""Check whether one field trial has readable bag and logger outputs."""

import argparse
import csv
import subprocess
from pathlib import Path


REQUIRED_TOPICS = (
    "/odometry/filtered", "/corn_row_center_line", "/corridor_confidence",
    "/cmd_vel", "/headland_detected", "/navigation_mode",
    "/navigation_safety_state",
)


def main():
    parser = argparse.ArgumentParser(description="验收一次大论文试验的数据完整性。")
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    args = parser.parse_args()

    failures = []
    warnings = []
    if not args.bag.exists():
        failures.append(f"rosbag目录不存在: {args.bag}")
        bag_info = ""
    else:
        try:
            completed = subprocess.run(
                ["ros2", "bag", "info", str(args.bag)], check=False,
                capture_output=True, text=True)
            bag_info = completed.stdout + completed.stderr
            if completed.returncode != 0:
                failures.append("ros2 bag info读取失败")
        except FileNotFoundError:
            failures.append("找不到ros2命令")
            bag_info = ""

    for topic in REQUIRED_TOPICS:
        if topic not in bag_info:
            failures.append(f"rosbag缺少必需话题: {topic}")

    matches = []
    for csv_path in args.results_root.rglob("timeseries.csv"):
        try:
            with csv_path.open(newline="", encoding="utf-8-sig") as handle:
                row = next(csv.DictReader(handle), None)
            if row and row.get("trial_id", "").strip() == args.trial_id:
                matches.append(csv_path)
        except (OSError, csv.Error):
            warnings.append(f"无法读取CSV: {csv_path}")

    if not matches:
        failures.append(f"没有找到trial_id={args.trial_id}的timeseries.csv")
    elif len(matches) > 1:
        warnings.append(f"发现{len(matches)}个同名trial_id，请检查重复试验")

    for csv_path in matches:
        line_count = 0
        with csv_path.open(encoding="utf-8-sig") as handle:
            for line_count, _ in enumerate(handle, start=1):
                pass
        if line_count < 3:
            failures.append(f"CSV有效数据不足: {csv_path}")
        print(f"LOGGER: {csv_path.resolve()} ({max(0, line_count - 1)} samples)")

    for warning in warnings:
        print(f"WARN: {warning}")
    for failure in failures:
        print(f"FAIL: {failure}")
    if failures:
        raise SystemExit(1)
    print("PASS: rosbag与自动日志通过基本完整性检查。")


if __name__ == "__main__":
    main()
