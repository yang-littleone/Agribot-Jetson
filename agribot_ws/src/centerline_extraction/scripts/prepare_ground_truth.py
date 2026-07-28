#!/usr/bin/env python3
"""将有效断面测量表转换成田间统计程序使用的真值表."""

import argparse
import csv
import math
from pathlib import Path


OUTPUT_FIELDS = (
    'trial_id', 'time_s', 'ros_time_s', 'lateral_error_m',
    'heading_error_deg', 'true_center_offset_m', 'true_row_yaw_deg',
    'plant_contact', 'intervention', 'completed',
)


def number(raw, name):
    value = str(raw.get(name, '')).strip()
    if not value:
        raise ValueError(f'字段 {name} 为空')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'字段 {name} 不是有限数值')
    return result


def flag(raw, name):
    value = str(raw.get(name, '0')).strip().lower()
    if value in {'', '0', 'false', 'no'}:
        return 0
    if value in {'1', 'true', 'yes'}:
        return 1
    raise ValueError(f'字段 {name} 必须是0或1')


def convert_row(raw):
    lateral_error = number(raw, 'lateral_error_m')
    heading_error = number(raw, 'heading_error_deg')
    ros_time = number(raw, 'matched_ros_time_s')
    matched_time = str(raw.get('matched_time_s', '')).strip()
    time_s = float(matched_time) if matched_time else ros_time
    true_offset = str(raw.get('true_center_offset_m', '')).strip()
    true_yaw = str(raw.get('true_row_yaw_deg', '')).strip()
    if not true_offset:
        cosine = math.cos(math.radians(heading_error))
        if abs(cosine) < 1e-6:
            raise ValueError('航向误差接近90度，不能使用局部直线近似')
        true_offset = -lateral_error / cosine
    else:
        true_offset = float(true_offset)
    if not true_yaw:
        true_yaw = -heading_error
    else:
        true_yaw = float(true_yaw)
    return {
        'trial_id': str(raw.get('trial_id', '')).strip(),
        'time_s': time_s,
        'ros_time_s': ros_time,
        'lateral_error_m': lateral_error,
        'heading_error_deg': heading_error,
        'true_center_offset_m': true_offset,
        'true_row_yaw_deg': true_yaw,
        'plant_contact': flag(raw, 'plant_contact'),
        'intervention': flag(raw, 'intervention'),
        'completed': flag(raw, 'completed'),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cross-sections', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    input_path = Path(args.cross_sections)
    output_path = Path(args.output)
    converted = []
    skipped = 0
    with input_path.open(newline='', encoding='utf-8-sig') as stream:
        for line_number, raw in enumerate(csv.DictReader(stream), start=2):
            if str(raw.get('valid', '1')).strip().lower() in {
                    '0', 'false', 'no'}:
                skipped += 1
                continue
            if not str(raw.get('trial_id', '')).strip():
                skipped += 1
                continue
            try:
                converted.append(convert_row(raw))
            except (TypeError, ValueError) as error:
                raise SystemExit(f'{input_path}:{line_number}: {error}') from error

    if not converted:
        raise SystemExit('没有可转换的有效断面真值')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(converted)
    print(
        f'已写入 {output_path}：有效断面 {len(converted)} 条，'
        f'跳过 {skipped} 条。')


if __name__ == '__main__':
    main()
