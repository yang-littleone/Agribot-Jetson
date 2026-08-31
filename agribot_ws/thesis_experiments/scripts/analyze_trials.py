#!/usr/bin/env python3
"""Merge automatic field logs with the manual register and summarize thesis trials."""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


TURN_MODES = {
    "turn_left", "turn_right", "multirow_left", "multirow_right",
}
MANUAL_COLUMNS = (
    "true_headland", "detected", "trigger_position_error_m", "false_triggers",
    "turn_completed", "correct_next_row", "reacquired",
    "next_row_follow_distance_m", "final_lateral_error_m",
    "final_heading_error_deg", "next_row_p95_error_m", "leaf_contacts",
    "stalk_contacts", "interventions", "emergency_stops", "valid",
)


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def first_float(row, *names):
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def truthy(value):
    if value is None or str(value).strip() == "":
        return None
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "y", "是", "成功", "有效"}:
        return True
    if normalized in {"0", "false", "no", "n", "否", "失败", "无效"}:
        return False
    return None


def as_int(value, default=0):
    number = finite_float(value)
    return int(number) if number is not None else default


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def read_register(path):
    if path is None or not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    register = {}
    for row in rows:
        trial_id = row.get("trial_id", "").strip()
        if trial_id:
            register[trial_id] = row
    return register


def first_index(rows, state, start=0):
    for index in range(start, len(rows)):
        if rows[index].get("navigation_mode", "").strip() == state:
            return index
    return None


def state_entry_indices(rows, state):
    entries = []
    previous = None
    for index, row in enumerate(rows):
        current = row.get("navigation_mode", "").strip()
        if current == state and previous != state:
            entries.append(index)
        previous = current
    return entries


def completed_cycles(rows):
    """Return complete U_TURN -> REACQUIRE -> ROW_FOLLOW state cycles."""
    cycles = []
    turn_entries = state_entry_indices(rows, "U_TURN")
    for position, turn_index in enumerate(turn_entries):
        next_turn = turn_entries[position + 1] if position + 1 < len(turn_entries) else len(rows)
        reacquire_index = first_index(rows, "NEXT_ROW_REACQUIRE", turn_index + 1)
        if reacquire_index is None or reacquire_index >= next_turn:
            continue
        row_index = first_index(rows, "ROW_FOLLOW", reacquire_index + 1)
        if row_index is None or row_index >= next_turn:
            continue
        cycles.append((turn_index, reacquire_index, row_index))
    return cycles


def path_delta(rows, start, stop):
    if start is None or stop is None or stop < start:
        return None
    start_distance = first_float(rows[start], "travel_distance_m")
    stop_distance = first_float(rows[stop], "travel_distance_m")
    if start_distance is None or stop_distance is None:
        return None
    return max(0.0, stop_distance - start_distance)


def time_at(rows, index):
    return first_float(rows[index], "time_s") if index is not None else None


def duration_between(rows, start, stop):
    start_time = time_at(rows, start)
    stop_time = time_at(rows, stop)
    if start_time is None or stop_time is None:
        return None
    return max(0.0, stop_time - start_time)


def values_for_state(rows, field, state=None, absolute=False):
    values = []
    for row in rows:
        if state is not None and row.get("navigation_mode", "").strip() != state:
            continue
        value = finite_float(row.get(field))
        if value is not None:
            values.append(abs(value) if absolute else value)
    return values


def infer_mode(trial_id, scenario):
    text = f"{trial_id} {scenario}".lower()
    if "multi" in text and ("right" in text or "mr" in text):
        return "multirow_right"
    if "multi" in text:
        return "multirow_left"
    if "right" in text or "_tr" in text:
        return "turn_right"
    if "left" in text or "_tl" in text:
        return "turn_left"
    return "detection"


def analyze_log(csv_path, manual):
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return None

    first = rows[0]
    trial_id = first.get("trial_id", "").strip() or csv_path.parent.name
    scenario = first.get("scenario", "").strip()
    manual_row = manual.get(trial_id, {})
    mode = manual_row.get("mode", "").strip() or infer_mode(trial_id, scenario)

    turn_entries = state_entry_indices(rows, "U_TURN")
    reacquire_entries = state_entry_indices(rows, "NEXT_ROW_REACQUIRE")
    cycles = completed_cycles(rows)
    turn_index = turn_entries[0] if turn_entries else None
    reacquire_index = first_index(
        rows, "NEXT_ROW_REACQUIRE", turn_index if turn_index is not None else 0)
    returned_row_index = first_index(
        rows, "ROW_FOLLOW", (reacquire_index + 1) if reacquire_index is not None else len(rows))
    headland_index = next(
        (index for index, row in enumerate(rows)
         if truthy(row.get("headland_detected")) is True), None)

    times = [value for value in (finite_float(r.get("time_s")) for r in rows)
             if value is not None]
    distances = [value for value in
                 (finite_float(r.get("travel_distance_m")) for r in rows)
                 if value is not None]
    confidence = values_for_state(rows, "published_confidence")
    margins = values_for_state(rows, "safety_margin_m")
    row_speed = values_for_state(rows, "cmd_linear_mps", "ROW_FOLLOW")
    turn_speed = values_for_state(rows, "cmd_linear_mps", "U_TURN")
    angular = values_for_state(rows, "cmd_angular_rps", "U_TURN", absolute=True)
    cycle_turn_durations = [duration_between(rows, start, reacquire)
                            for start, reacquire, _ in cycles]
    cycle_reacquire_durations = [duration_between(rows, reacquire, row)
                                 for _, reacquire, row in cycles]
    cycle_turn_paths = [path_delta(rows, start, reacquire)
                        for start, reacquire, _ in cycles]
    cycle_reacquire_paths = [path_delta(rows, reacquire, row)
                             for _, reacquire, row in cycles]
    cycle_turn_durations = [v for v in cycle_turn_durations if v is not None]
    cycle_reacquire_durations = [v for v in cycle_reacquire_durations if v is not None]
    cycle_turn_paths = [v for v in cycle_turn_paths if v is not None]
    cycle_reacquire_paths = [v for v in cycle_reacquire_paths if v is not None]
    safety_states = [r.get("navigation_safety_state", "").strip() for r in rows]
    stop_samples = sum(
        1 for row, state in zip(rows, safety_states)
        if "STOP" in state.upper()
        or abs(finite_float(row.get("cmd_linear_mps")) or 0.0) < 1e-6
        and state not in {"", "NORMAL", "OK"})

    automatic = {
        "trial_id": trial_id,
        "mode": mode,
        "scenario": scenario,
        "repeat_index": first.get("repeat_index", ""),
        "timeseries_file": str(csv_path.resolve()),
        "samples": len(rows),
        "duration_s": max(times) - min(times) if times else None,
        "travel_distance_m": max(distances) - min(distances) if distances else None,
        "headland_signal_seen": headland_index is not None,
        "headland_signal_time_s": time_at(rows, headland_index),
        "uturn_count": len(turn_entries),
        "reacquire_count": len(reacquire_entries),
        "completed_turn_cycles": len(cycles),
        "uturn_seen": turn_index is not None,
        "uturn_start_time_s": time_at(rows, turn_index),
        "uturn_duration_s": duration_between(rows, turn_index, reacquire_index),
        "uturn_path_length_m": path_delta(rows, turn_index, reacquire_index),
        "reacquire_seen": reacquire_index is not None,
        "reacquire_start_time_s": time_at(rows, reacquire_index),
        "reacquire_duration_s": duration_between(rows, reacquire_index, returned_row_index),
        "reacquire_path_length_m": path_delta(rows, reacquire_index, returned_row_index),
        "returned_to_row_follow": returned_row_index is not None,
        "row_follow_return_time_s": time_at(rows, returned_row_index),
        "state_machine_complete": (
            turn_index is not None and reacquire_index is not None
            and returned_row_index is not None),
        "mean_cycle_uturn_duration_s": (
            statistics.fmean(cycle_turn_durations)
            if cycle_turn_durations else None),
        "mean_cycle_uturn_path_length_m": (
            statistics.fmean(cycle_turn_paths) if cycle_turn_paths else None),
        "mean_cycle_reacquire_duration_s": (
            statistics.fmean(cycle_reacquire_durations)
            if cycle_reacquire_durations else None),
        "mean_cycle_reacquire_path_length_m": (
            statistics.fmean(cycle_reacquire_paths)
            if cycle_reacquire_paths else None),
        "mean_row_cmd_speed_mps": statistics.fmean(row_speed) if row_speed else None,
        "mean_turn_cmd_speed_mps": statistics.fmean(turn_speed) if turn_speed else None,
        "max_abs_turn_cmd_angular_rps": max(angular) if angular else None,
        "min_confidence": min(confidence) if confidence else None,
        "min_safety_margin_m": min(margins) if margins else None,
        "safety_stop_samples": stop_samples,
    }

    for column in MANUAL_COLUMNS:
        automatic[column] = manual_row.get(column, "")
    for column in ("date", "plot_id", "row_pair_id", "planned_direction",
                   "bag_dir", "result_dir", "video_id", "notes"):
        automatic[column] = manual_row.get(column, "")

    valid = truthy(manual_row.get("valid"))
    if mode in TURN_MODES:
        required_cycles = 4 if mode.startswith("multirow_") else 1
        manual_success = all(
            truthy(manual_row.get(name)) is True
            for name in ("turn_completed", "correct_next_row", "reacquired"))
        no_harm = all(
            as_int(manual_row.get(name)) == 0
            for name in ("stalk_contacts", "interventions", "emergency_stops"))
        next_row_distance = finite_float(manual_row.get("next_row_follow_distance_m"))
        enough_follow = next_row_distance is not None and next_row_distance >= 5.0
        automatic["formal_success"] = (
            valid is not False
            and automatic["completed_turn_cycles"] >= required_cycles
            and manual_success and no_harm and enough_follow)
    else:
        true_headland = truthy(manual_row.get("true_headland"))
        detected = truthy(manual_row.get("detected"))
        false_triggers = as_int(manual_row.get("false_triggers"))
        automatic["formal_success"] = (
            valid is not False and true_headland is not None and detected is not None
            and detected == true_headland and false_triggers == 0)

    automatic["manual_register_found"] = bool(manual_row)
    return automatic


def mean_std(values):
    numbers = [finite_float(value) for value in values]
    numbers = [value for value in numbers if value is not None]
    if not numbers:
        return "", ""
    mean = statistics.fmean(numbers)
    std = statistics.stdev(numbers) if len(numbers) > 1 else 0.0
    return fmt(mean), fmt(std)


def write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: fmt(row.get(key)) for key in fieldnames})


def aggregate(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row["mode"], row["scenario"])].append(row)
    output = []
    metric_names = (
        "duration_s", "travel_distance_m", "uturn_duration_s",
        "uturn_path_length_m", "reacquire_duration_s",
        "reacquire_path_length_m", "trigger_position_error_m",
        "final_lateral_error_m", "final_heading_error_deg",
        "next_row_p95_error_m", "min_confidence", "min_safety_margin_m",
        "completed_turn_cycles", "mean_cycle_uturn_duration_s",
        "mean_cycle_uturn_path_length_m", "mean_cycle_reacquire_duration_s",
        "mean_cycle_reacquire_path_length_m",
    )
    for (mode, scenario), group in sorted(grouped.items()):
        registered = [row for row in group if row["manual_register_found"]]
        successes = sum(1 for row in registered if row["formal_success"])
        summary = {
            "mode": mode,
            "scenario": scenario,
            "automatic_logs_n": len(group),
            "registered_n": len(registered),
            "successes_n": successes,
            "success_rate": successes / len(registered) if registered else None,
        }
        for metric in metric_names:
            mean, std = mean_std(row.get(metric) for row in group)
            summary[f"{metric}_mean"] = mean
            summary[f"{metric}_std"] = std
        output.append(summary)
    return output


def main():
    parser = argparse.ArgumentParser(
        description="汇总大论文试验自动日志，并与现场登记表按trial_id合并。")
    parser.add_argument("--results-root", type=Path, required=True,
                        help="field_trial_logger输出根目录")
    parser.add_argument("--register", type=Path,
                        help="已填写的trial_register.csv")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="统计结果输出目录")
    args = parser.parse_args()

    manual = read_register(args.register)
    csv_files = sorted(args.results_root.rglob("timeseries.csv"))
    rows = []
    seen = set()
    for csv_path in csv_files:
        result = analyze_log(csv_path, manual)
        if result is None:
            continue
        trial_id = result["trial_id"]
        if trial_id in seen:
            print(f"警告: trial_id重复，仍分别保留: {trial_id} ({csv_path})")
        seen.add(trial_id)
        rows.append(result)

    if not rows:
        raise SystemExit(f"未找到有效timeseries.csv: {args.results_root}")

    trial_fields = list(rows[0].keys())
    trial_path = args.output_dir / "trial_summary.csv"
    write_csv(trial_path, rows, trial_fields)
    group_rows = aggregate(rows)
    group_fields = list(group_rows[0].keys()) if group_rows else []
    group_path = args.output_dir / "group_summary.csv"
    write_csv(group_path, group_rows, group_fields)

    missing = sum(1 for row in rows if not row["manual_register_found"])
    print(f"已分析自动日志: {len(rows)} 组")
    print(f"未匹配现场登记: {missing} 组")
    print(f"逐次结果: {trial_path.resolve()}")
    print(f"分组统计: {group_path.resolve()}")


if __name__ == "__main__":
    main()
