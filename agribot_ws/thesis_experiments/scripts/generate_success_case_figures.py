#!/usr/bin/env python3
"""Generate thesis figures from the four physically successful field turns."""

from __future__ import annotations

import argparse
import csv
from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
import tempfile

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection
from matplotlib.patches import Patch
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_DATA_ROOT = Path("/media/wheeltec/KINGSTON/agribot_field_data")
DEFAULT_OUTPUT = (
    WORKSPACE / "thesis_experiments" / "analysis" / "success_cases_20260901"
)

ENTRY_LATERAL_TOLERANCE_M = 0.15
ENTRY_HEADING_TOLERANCE_DEG = 35.0
ENTRY_SAMPLE_GAP_S = 0.15
FIELD_ROW_SPACING_M = 0.60


@dataclass(frozen=True)
class Case:
    short_name: str
    bag_name: str
    result_name: str


CASES = (
    Case("R01", "20260901_120240_F_TR_P01_v40_R01",
         "20260901_120247_F_TR_P01_v40_R01"),
    Case("R02", "20260901_120752_F_TR_P01_v40_R02",
         "20260901_120813_F_TR_P01_v40_R02"),
    Case("R03", "20260901_121437_F_TR_P01_v40_R03",
         "20260901_121452_F_TR_P01_v40_R03"),
    Case("R04", "20260901_123236_F_TR_P01_v40_R04",
         "20260901_123302_F_TR_P01_v40_R04"),
)


def configure_plot() -> None:
    candidates = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    )
    for candidate in candidates:
        if Path(candidate).exists():
            font_manager.fontManager.addfont(candidate)
            family = font_manager.FontProperties(fname=candidate).get_name()
            plt.rcParams["font.sans-serif"] = [family]
            break
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 150
    plt.rcParams["savefig.dpi"] = 300
    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.20


def finite(value, default=np.nan):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def read_csv_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def column(rows: list[dict], name: str) -> np.ndarray:
    return np.asarray([finite(row.get(name)) for row in rows], dtype=float)


def modes_in_order(rows: list[dict]) -> list[str]:
    output = []
    for row in rows:
        mode = row.get("navigation_mode", "").strip()
        if mode and (not output or mode != output[-1]):
            output.append(mode)
    return output


@contextmanager
def decompressed_database(bag_dir: Path):
    direct = sorted(bag_dir.glob("*.db3"))
    if direct:
        yield direct[0]
        return
    compressed = sorted(bag_dir.glob("*.db3.zstd"))
    if not compressed:
        raise FileNotFoundError(f"No db3 or db3.zstd found in {bag_dir}")
    temp_dir = Path(tempfile.mkdtemp(prefix="thesis_turn_", dir="/tmp"))
    database = temp_dir / "run.db3"
    try:
        subprocess.run(
            ["zstd", "-q", "-d", str(compressed[0]), "-o", str(database)],
            check=True,
        )
        yield database
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def topic_info(connection: sqlite3.Connection) -> dict[str, tuple[int, str]]:
    return {
        name: (topic_id, message_type)
        for topic_id, name, message_type in connection.execute(
            "SELECT id, name, type FROM topics"
        )
    }


def deserialize_message(data, message_type: str):
    from rclpy.serialization import deserialize_message as ros_deserialize
    from rosidl_runtime_py.utilities import get_message

    return ros_deserialize(data, get_message(message_type))


def first_nonempty_path(connection, topics, topic_name: str):
    topic_id, message_type = topics[topic_name]
    cursor = connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    )
    for timestamp, data in cursor:
        message = deserialize_message(data, message_type)
        if len(message.poses) >= 3:
            return timestamp, message
    raise RuntimeError(f"No non-empty path on {topic_name}")


def nearest_message(connection, topics, topic_name: str, timestamp_ns: int):
    topic_id, message_type = topics[topic_name]
    row = connection.execute(
        """
        SELECT timestamp, data FROM messages
        WHERE topic_id=? ORDER BY ABS(timestamp - ?) LIMIT 1
        """,
        (topic_id, timestamp_ns),
    ).fetchone()
    if row is None:
        raise RuntimeError(f"No messages on {topic_name}")
    return row[0], deserialize_message(row[1], message_type)


def nearest_nonempty_path(connection, topics, topic_name: str, timestamp_ns: int):
    topic_id, message_type = topics[topic_name]
    rows = connection.execute(
        """
        SELECT timestamp, data FROM messages
        WHERE topic_id=? ORDER BY ABS(timestamp - ?) LIMIT 80
        """,
        (topic_id, timestamp_ns),
    ).fetchall()
    for timestamp, data in rows:
        message = deserialize_message(data, message_type)
        if len(message.poses) >= 3:
            return timestamp, message
    raise RuntimeError(f"No nearby non-empty path on {topic_name}")


def path_xy(message) -> np.ndarray:
    return np.asarray(
        [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses],
        dtype=float,
    )


def unwrap_heading_difference(yaw: np.ndarray, reference: float) -> np.ndarray:
    return np.abs(np.arctan2(np.sin(yaw - reference), np.cos(yaw - reference)))


def longest_contiguous_segment(indices: np.ndarray, times: np.ndarray,
                               longitudinal: np.ndarray):
    if not len(indices):
        return None
    groups = []
    start = 0
    for i in range(1, len(indices) + 1):
        split = i == len(indices)
        if not split:
            split = times[indices[i]] - times[indices[i - 1]] > ENTRY_SAMPLE_GAP_S
        if split:
            group = indices[start:i]
            span = float(np.nanmax(longitudinal[group]) - np.nanmin(longitudinal[group]))
            duration = float(times[group[-1]] - times[group[0]])
            groups.append((span, duration, group))
            start = i
    return max(groups, key=lambda item: (item[0], item[1]))


def percentile_abs(values: np.ndarray, percentile: float):
    values = values[np.isfinite(values)]
    return float(np.percentile(np.abs(values), percentile)) if len(values) else np.nan


def analyze_case(case: Case, data_root: Path):
    result_dir = data_root / "field_trial_results" / case.result_name
    bag_dir = data_root / "field_trial_bags" / case.bag_name
    rows = read_csv_rows(result_dir / "timeseries.csv")
    time_s = column(rows, "time_s")
    ros_time_s = column(rows, "ros_time_s")
    x = column(rows, "x_m")
    y = column(rows, "y_m")
    yaw = column(rows, "yaw_rad")
    travel = column(rows, "travel_distance_m")
    modes = np.asarray([row.get("navigation_mode", "").strip() for row in rows])

    with decompressed_database(bag_dir) as database:
        connection = sqlite3.connect(str(database))
        try:
            topics = topic_info(connection)
            _, turn_path_message = first_nonempty_path(
                connection, topics, "/headland_turn_path"
            )
        finally:
            connection.close()

    turn_path = path_xy(turn_path_message)
    origin = turn_path[0]
    initial_heading = math.atan2(
        turn_path[1, 1] - turn_path[0, 1],
        turn_path[1, 0] - turn_path[0, 0],
    )
    along_axis = np.array([math.cos(initial_heading), math.sin(initial_heading)])
    lateral_axis = np.array([-math.sin(initial_heading), math.cos(initial_heading)])
    turn_relative = turn_path - origin
    planned_along = turn_relative @ along_axis
    planned_lateral = turn_relative @ lateral_axis
    target_lateral = float(np.mean(planned_lateral[-6:]))
    target_sign = 1.0 if target_lateral >= 0.0 else -1.0

    actual_relative = np.column_stack((x, y)) - origin
    actual_along = actual_relative @ along_axis
    actual_lateral = actual_relative @ lateral_axis
    reverse_heading = initial_heading + math.pi
    reverse_error = unwrap_heading_difference(yaw, reverse_heading)
    physical_row_lateral = target_sign * FIELD_ROW_SPACING_M
    entry_mask = (
        np.isfinite(actual_lateral)
        & np.isfinite(reverse_error)
        & (np.abs(actual_lateral - physical_row_lateral) <= ENTRY_LATERAL_TOLERANCE_M)
        & (reverse_error <= math.radians(ENTRY_HEADING_TOLERANCE_DEG))
    )
    entry_indices = np.flatnonzero(entry_mask)
    segment = longest_contiguous_segment(entry_indices, time_s, actual_along)
    if segment is None:
        raise RuntimeError(f"Physical row-entry evidence not found: {case.result_name}")
    entry_distance, entry_duration, entry_group = segment

    turn_indices = np.flatnonzero(modes == "U_TURN")
    turn_start = int(turn_indices[0])
    turn_end = int(turn_indices[-1])
    turn_yaw = np.unwrap(yaw[turn_indices])
    yaw_change_deg = math.degrees(turn_yaw[-1] - turn_yaw[0])

    pre_turn = np.flatnonzero((modes == "ROW_FOLLOW") & (np.arange(len(rows)) < turn_start))
    lateral_error = column(rows, "controller_lateral_error_m")
    heading_error = column(rows, "controller_heading_error_rad")
    valid = column(rows, "valid")

    reacquire = np.flatnonzero(modes == "NEXT_ROW_REACQUIRE")
    post_row_distance = np.nan
    post_row_duration = np.nan
    post_lateral_mae = np.nan
    post_lateral_p95 = np.nan
    if len(reacquire):
        returned_all = np.flatnonzero(
            (modes == "ROW_FOLLOW") & (np.arange(len(rows)) > reacquire[-1])
        )
        cmd_linear = column(rows, "cmd_linear_mps")
        returned = returned_all[np.abs(cmd_linear[returned_all]) > 0.01]
        if len(returned):
            start = returned[0]
            stop = returned[-1]
            post_row_distance = float(travel[stop] - travel[start])
            post_row_duration = float(time_s[stop] - time_s[start])
            post_lateral_mae = float(np.nanmean(np.abs(lateral_error[returned])))
            post_lateral_p95 = percentile_abs(lateral_error[returned], 95)

    close_heading = reverse_error <= math.radians(ENTRY_HEADING_TOLERANCE_DEG)
    close_lateral = (
        np.abs(actual_lateral - physical_row_lateral) <= ENTRY_LATERAL_TOLERANCE_M
    )
    metrics = {
        "case": case.short_name,
        "bag_name": case.bag_name,
        "result_name": case.result_name,
        "physical_turn_success": 1,
        "success_definition": "entered_adjacent_row",
        "planned_row_spacing_m": abs(target_lateral),
        "nominal_speed_mps": finite(rows[0].get("nominal_speed_mps")),
        "navigation_mode_sequence": ">".join(modes_in_order(rows)),
        "state_machine_complete": int(len(reacquire) > 0),
        "turn_yaw_change_deg": yaw_change_deg,
        "field_nominal_row_spacing_m": FIELD_ROW_SPACING_M,
        "measured_entry_lateral_offset_m": float(np.mean(
            target_sign * actual_lateral[entry_group]
        )),
        "entry_evidence_start_s": float(time_s[entry_group[0]]),
        "entry_evidence_end_s": float(time_s[entry_group[-1]]),
        "entry_evidence_duration_s": entry_duration,
        "entry_evidence_distance_m": entry_distance,
        "entry_lateral_mae_m": float(np.mean(np.abs(
            actual_lateral[entry_group] - physical_row_lateral
        ))),
        "entry_heading_mae_deg": float(np.degrees(np.mean(reverse_error[entry_group]))),
        "closest_lateral_error_when_aligned_m": float(np.nanmin(
            np.abs(actual_lateral[close_heading] - physical_row_lateral)
        )),
        "best_heading_error_near_row_deg": float(np.degrees(np.nanmin(
            reverse_error[close_lateral]
        ))),
        "pre_turn_valid_ratio": float(np.nanmean(valid[pre_turn])) if len(pre_turn) else np.nan,
        "pre_turn_lateral_mae_m": float(np.nanmean(np.abs(lateral_error[pre_turn]))),
        "pre_turn_lateral_p95_m": percentile_abs(lateral_error[pre_turn], 95),
        "pre_turn_heading_mae_deg": float(np.degrees(
            np.nanmean(np.abs(heading_error[pre_turn]))
        )),
        "post_turn_row_follow_distance_m": post_row_distance,
        "post_turn_row_follow_duration_s": post_row_duration,
        "post_turn_lateral_mae_m": post_lateral_mae,
        "post_turn_lateral_p95_m": post_lateral_p95,
    }
    arrays = {
        "rows": rows,
        "time": time_s,
        "ros_time": ros_time_s,
        "x": x,
        "y": y,
        "yaw": yaw,
        "travel": travel,
        "modes": modes,
        "turn_path": turn_path,
        "origin": origin,
        "initial_heading": initial_heading,
        "planned_along": planned_along,
        "planned_lateral": planned_lateral * target_sign,
        "actual_along": actual_along,
        "actual_lateral": actual_lateral * target_sign,
        "planned_lateral_offset": abs(target_lateral),
        "field_row_spacing": FIELD_ROW_SPACING_M,
        "entry_group": entry_group,
        "turn_start": turn_start,
        "turn_end": turn_end,
    }
    return metrics, arrays


def write_metrics(output_dir: Path, metrics: list[dict]) -> None:
    data_dir = output_dir / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    fields = list(metrics[0].keys())
    with (data_dir / "successful_turn_metrics.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(metrics)
    paper_fields = (
        "试验编号", "结果目录", "转向", "现场名义行距_m", "规划横移_m",
        "实测进入横移_m", "相邻行连续距离_m", "相邻行连续时间_s",
        "证据段横向MAE_cm", "反向航向MAE_deg", "掉头前中心线有效率_pct",
        "物理换行成功", "状态机完整闭环",
    )
    with (data_dir / "论文表_成功换行样例.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=paper_fields)
        writer.writeheader()
        for item in metrics:
            writer.writerow({
                "试验编号": item["case"],
                "结果目录": item["result_name"],
                "转向": "右转",
                "现场名义行距_m": f"{item['field_nominal_row_spacing_m']:.2f}",
                "规划横移_m": f"{item['planned_row_spacing_m']:.2f}",
                "实测进入横移_m": f"{item['measured_entry_lateral_offset_m']:.3f}",
                "相邻行连续距离_m": f"{item['entry_evidence_distance_m']:.3f}",
                "相邻行连续时间_s": f"{item['entry_evidence_duration_s']:.2f}",
                "证据段横向MAE_cm": f"{item['entry_lateral_mae_m'] * 100:.2f}",
                "反向航向MAE_deg": f"{item['entry_heading_mae_deg']:.2f}",
                "掉头前中心线有效率_pct": f"{item['pre_turn_valid_ratio'] * 100:.2f}",
                "物理换行成功": "是",
                "状态机完整闭环": "是" if item["state_machine_complete"] else "否",
            })

    summary_fields = (
        "selected_success_cases_n", "measured_entry_offset_mean_m",
        "measured_entry_offset_std_m", "entry_distance_mean_m",
        "entry_distance_std_m", "entry_duration_mean_s", "entry_duration_std_s",
        "entry_lateral_mae_mean_m", "entry_lateral_mae_std_m",
        "entry_heading_mae_mean_deg", "entry_heading_mae_std_deg",
    )
    values = lambda name: np.asarray([item[name] for item in metrics], dtype=float)
    summary = {
        "selected_success_cases_n": len(metrics),
        "measured_entry_offset_mean_m": np.mean(values("measured_entry_lateral_offset_m")),
        "measured_entry_offset_std_m": np.std(values("measured_entry_lateral_offset_m"), ddof=1),
        "entry_distance_mean_m": np.mean(values("entry_evidence_distance_m")),
        "entry_distance_std_m": np.std(values("entry_evidence_distance_m"), ddof=1),
        "entry_duration_mean_s": np.mean(values("entry_evidence_duration_s")),
        "entry_duration_std_s": np.std(values("entry_evidence_duration_s"), ddof=1),
        "entry_lateral_mae_mean_m": np.mean(values("entry_lateral_mae_m")),
        "entry_lateral_mae_std_m": np.std(values("entry_lateral_mae_m"), ddof=1),
        "entry_heading_mae_mean_deg": np.mean(values("entry_heading_mae_deg")),
        "entry_heading_mae_std_deg": np.std(values("entry_heading_mae_deg"), ddof=1),
    }
    with (data_dir / "selected_success_summary.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerow({key: f"{value:.6f}" if isinstance(value, float) else value
                         for key, value in summary.items()})
    with (data_dir / "analysis_definition.json").open("w", encoding="utf-8") as handle:
        json.dump(
            {
                "included_cases": [item["result_name"] for item in metrics],
                "scope": "physically successful adjacent-row entries only",
                "entry_lateral_tolerance_m": ENTRY_LATERAL_TOLERANCE_M,
                "entry_heading_tolerance_deg": ENTRY_HEADING_TOLERANCE_DEG,
                "entry_sample_gap_s": ENTRY_SAMPLE_GAP_S,
                "field_nominal_row_spacing_m": FIELD_ROW_SPACING_M,
                "warning": (
                    "The selected-case set is not a denominator for an overall success rate. "
                    "Only R03 completed the full navigation state sequence."
                ),
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )


def save_figure(fig, output_dir: Path, stem: str) -> None:
    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(figure_dir / f"{stem}.png", bbox_inches="tight")
    fig.savefig(figure_dir / f"{stem}.svg", bbox_inches="tight")
    plt.close(fig)


def plot_success_trajectories(output_dir: Path, analyses: list[tuple[dict, dict]]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.2, 9.2))
    fig.suptitle("四次成功换行实验的实测轨迹与规划掉头线", fontsize=17,
                 fontweight="bold")
    for ax, (metrics, arrays) in zip(axes.flat, analyses):
        start = max(0, arrays["turn_start"] - 25)
        stop_time = metrics["entry_evidence_end_s"] + 1.0
        stop_candidates = np.flatnonzero(arrays["time"] <= stop_time)
        stop = int(stop_candidates[-1]) + 1 if len(stop_candidates) else len(arrays["time"])
        segment = slice(start, stop)
        ax.axhline(0.0, color="#7f8c8d", linestyle=":", linewidth=1.4,
                   label="原行中心")
        ax.axhline(arrays["field_row_spacing"], color="#2ca02c", linestyle="--",
                   linewidth=1.6, label="现场名义相邻行中心（0.60 m）")
        ax.plot(arrays["planned_along"], arrays["planned_lateral"],
                color="#f39c12", linestyle="--", linewidth=2.3,
                label="规划掉头线")
        ax.plot(arrays["actual_along"][segment], arrays["actual_lateral"][segment],
                color="#34495e", linewidth=1.8, label="实测轨迹")
        group = arrays["entry_group"]
        ax.plot(arrays["actual_along"][group], arrays["actual_lateral"][group],
                color="#18a558", linewidth=3.4, label="进入相邻行证据段")
        ax.scatter(arrays["actual_along"][arrays["turn_start"]],
                   arrays["actual_lateral"][arrays["turn_start"]],
                   color="#c0392b", marker="o", s=45, zorder=6, label="掉头开始")
        ax.set_title(
            f"{metrics['case']}  {metrics['result_name'][:15]}\n"
            f"相邻行内连续 {metrics['entry_evidence_distance_m']:.2f} m / "
            f"{metrics['entry_evidence_duration_s']:.2f} s"
        )
        ax.set_xlabel("沿原作物行方向（m）")
        ax.set_ylabel("向相邻行横移（m）")
        ax.set_aspect("equal", adjustable="datalim")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=9,
               bbox_to_anchor=(0.5, 0.01))
    fig.subplots_adjust(left=0.08, right=0.98, top=0.88, bottom=0.14,
                        hspace=0.40, wspace=0.24)
    save_figure(fig, output_dir, "01_successful_turn_trajectories")


def mode_intervals(times, modes):
    intervals = []
    start = 0
    for i in range(1, len(modes) + 1):
        if i == len(modes) or modes[i] != modes[i - 1]:
            if modes[start]:
                intervals.append((modes[start], times[start], times[i - 1]))
            start = i
    return intervals


def plot_r03_control(output_dir: Path, metrics: dict, arrays: dict) -> None:
    rows = arrays["rows"]
    rel_time = arrays["time"] - arrays["time"][arrays["turn_start"]]
    lateral = np.abs(column(rows, "controller_lateral_error_m"))
    heading = np.degrees(np.abs(column(rows, "controller_heading_error_rad")))
    linear = column(rows, "cmd_linear_mps")
    angular = column(rows, "cmd_angular_rps")
    confidence = column(rows, "published_confidence")
    mask = (rel_time >= -4.0) & (rel_time <= 58.0)

    fig, axes = plt.subplots(4, 1, figsize=(12, 10.5), sharex=True,
                             constrained_layout=True)
    fig.suptitle("完整成功样例 R03 的掉头控制过程", fontsize=17,
                 fontweight="bold")
    colors = {"ROW_FOLLOW": "#d9f0d3", "U_TURN": "#fee8c8",
              "NEXT_ROW_REACQUIRE": "#dadaeb"}
    intervals = mode_intervals(rel_time[mask], arrays["modes"][mask])
    for ax in axes:
        for mode, begin, end in intervals:
            ax.axvspan(begin, end, color=colors.get(mode, "#eeeeee"), alpha=0.62)
        ax.axvline(0.0, color="#c0392b", linestyle="--", linewidth=1.2)

    axes[0].plot(rel_time[mask], lateral[mask], color="#1f77b4", label="横向误差")
    axes[0].set_ylabel("横向误差（m）")
    axes[0].legend(loc="upper left")

    axes[1].plot(rel_time[mask], heading[mask], color="#d62728",
                 label="航向误差")
    axes[1].set_ylabel("航向误差（°）")
    axes[1].legend(loc="upper left")

    axes[2].plot(rel_time[mask], linear[mask], color="#2ca02c",
                 label="线速度指令")
    axes[2].plot(rel_time[mask], angular[mask], color="#ff7f0e",
                 label="角速度指令")
    axes[2].set_ylabel("速度指令\n（m/s, rad/s）")
    axes[2].legend(loc="upper left")

    axes[3].plot(rel_time[mask], confidence[mask], color="#6a3d9a",
                 label="感知置信度")
    axes[3].set_ylabel("置信度")
    axes[3].set_xlabel("相对掉头开始时刻（s）")
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].legend(loc="upper left")
    legend_handles = [Patch(facecolor=color, alpha=0.62, label=mode)
                      for mode, color in colors.items()]
    axes[3].legend(handles=legend_handles, loc="lower right", ncol=3, fontsize=8)
    save_figure(fig, output_dir, "02_r03_control_process")


def plot_summary(output_dir: Path, metrics: list[dict]) -> None:
    labels = [item["case"] for item in metrics]
    x = np.arange(len(labels))
    distances = [item["entry_evidence_distance_m"] for item in metrics]
    durations = [item["entry_evidence_duration_s"] for item in metrics]
    lateral = [item["entry_lateral_mae_m"] * 100 for item in metrics]
    heading = [item["entry_heading_mae_deg"] for item in metrics]
    colors = ["#4c78a8", "#59a14f", "#f28e2b", "#e15759"]

    fig, axes = plt.subplots(2, 2, figsize=(11.8, 8.5), constrained_layout=True)
    fig.suptitle("成功换行样例的相邻行进入证据", fontsize=17, fontweight="bold")
    for ax, values, title, ylabel, formatter in (
        (axes[0, 0], distances, "相邻行内连续行驶距离", "距离（m）", "{:.2f}"),
        (axes[0, 1], durations, "相邻行内连续保持时间", "时间（s）", "{:.2f}"),
        (axes[1, 0], lateral, "证据段横向偏差均值", "绝对偏差（cm）", "{:.1f}"),
        (axes[1, 1], heading, "证据段反向航向偏差均值", "绝对偏差（°）", "{:.1f}"),
    ):
        bars = ax.bar(x, values, color=colors, width=0.65)
        ax.set_xticks(x, labels)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    formatter.format(value), ha="center", va="bottom", fontsize=9)
    save_figure(fig, output_dir, "03_success_case_metrics")


def quaternion_matrix(quaternion) -> np.ndarray:
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.array([
        [1-scale*(y*y+z*z), scale*(x*y-z*w), scale*(x*z+y*w)],
        [scale*(x*y+z*w), 1-scale*(x*x+z*z), scale*(y*z-x*w)],
        [scale*(x*z-y*w), scale*(y*z+x*w), 1-scale*(x*x+y*y)],
    ])


def transform_lidar_to_base(points: np.ndarray, tf_message) -> np.ndarray:
    transform = None
    for item in tf_message.transforms:
        if item.header.frame_id == "base_link" and item.child_frame_id == "laser_link":
            transform = item.transform
            break
    if transform is None:
        raise RuntimeError("base_link -> laser_link static transform not found")
    rotation = quaternion_matrix(transform.rotation)
    translation = np.array([
        transform.translation.x, transform.translation.y, transform.translation.z
    ])
    return points @ rotation.T + translation


def pointcloud_xyz(message) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    records = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    if getattr(records.dtype, "names", None):
        return np.column_stack(
            (records["x"], records["y"], records["z"])
        ).astype(np.float32, copy=False)
    return np.asarray(records, dtype=np.float32).reshape(-1, 3)


def odom_path_to_base(points: np.ndarray, pose_x: float, pose_y: float,
                      pose_yaw: float) -> np.ndarray:
    relative = points - np.array([pose_x, pose_y])
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)
    return relative @ np.array([[c, -s], [s, c]])


def load_representative_pointcloud(case: Case, arrays: dict, data_root: Path):
    target_index = max(0, arrays["turn_start"] - 30)
    target_ns = int(arrays["ros_time"][target_index] * 1e9)
    bag_dir = data_root / "field_trial_bags" / case.bag_name
    with decompressed_database(bag_dir) as database:
        connection = sqlite3.connect(str(database))
        try:
            topics = topic_info(connection)
            center_ts, center_message = nearest_nonempty_path(
                connection, topics, "/corn_row_center_line", target_ns
            )
            _, left_message = nearest_nonempty_path(
                connection, topics, "/under_canopy_left_boundary", center_ts
            )
            _, right_message = nearest_nonempty_path(
                connection, topics, "/under_canopy_right_boundary", center_ts
            )
            _, lidar_message = nearest_message(
                connection, topics, "/livox/lidar", center_ts
            )
            _, tf_message = nearest_message(connection, topics, "/tf_static", center_ts)
        finally:
            connection.close()

    cloud = transform_lidar_to_base(pointcloud_xyz(lidar_message), tf_message)
    ros_seconds = center_ts / 1e9
    pose_index = int(np.nanargmin(np.abs(arrays["ros_time"] - ros_seconds)))
    pose = (arrays["x"][pose_index], arrays["y"][pose_index], arrays["yaw"][pose_index])
    center = odom_path_to_base(path_xy(center_message), *pose)
    left = odom_path_to_base(path_xy(left_message), *pose)
    right = odom_path_to_base(path_xy(right_message), *pose)
    return {
        "cloud": cloud,
        "center": center,
        "left": left,
        "right": right,
        "timestamp_s": ros_seconds,
        "pose_index": pose_index,
    }


def boundary_distance_mask(points, boundary, half_width=0.055):
    order = np.argsort(boundary[:, 0])
    bx = boundary[order, 0]
    by = boundary[order, 1]
    inside = (points[:, 0] >= bx[0]) & (points[:, 0] <= bx[-1])
    predicted = np.interp(points[:, 0], bx, by)
    return inside & (np.abs(points[:, 1] - predicted) <= half_width)


def plot_pointcloud(output_dir: Path, sample: dict) -> None:
    cloud = sample["cloud"]
    xy = ((cloud[:, 0] >= -0.2) & (cloud[:, 0] <= 3.0)
          & (cloud[:, 1] >= -1.5) & (cloud[:, 1] <= 1.5)
          & (cloud[:, 2] >= -0.1) & (cloud[:, 2] <= 1.2))
    raw = cloud[xy]
    roi_mask = ((raw[:, 0] >= 0.1) & (raw[:, 0] <= 2.2)
                & (raw[:, 1] >= -1.4) & (raw[:, 1] <= 1.4)
                & (raw[:, 2] >= 0.1) & (raw[:, 2] <= 0.5))
    roi = raw[roi_mask]
    left_mask = boundary_distance_mask(roi[:, :2], sample["left"])
    right_mask = boundary_distance_mask(roi[:, :2], sample["right"])
    other = ~(left_mask | right_mask)

    fig, axes = plt.subplots(1, 3, figsize=(15.2, 5.1), constrained_layout=True)
    fig.suptitle("完整成功样例 R03 的真实点云处理与中心线输出", fontsize=17,
                 fontweight="bold")
    axes[0].grid(False)
    with plt.rc_context({"axes.grid": False}):
        scatter = axes[0].scatter(
            raw[:, 0], raw[:, 1], c=raw[:, 2], s=3,
            cmap="viridis", alpha=0.55, linewidths=0
        )
        fig.colorbar(scatter, ax=axes[0], label="高度 z（m）")
    axes[0].set_title(f"(a) MID360 原始前方点云（{len(raw)}点）")

    axes[1].grid(False)
    axes[1].scatter(roi[:, 0], roi[:, 1], c=roi[:, 2], s=5,
                    cmap="viridis", alpha=0.65, linewidths=0)
    axes[1].set_title(f"(b) 高度与范围 ROI（{len(roi)}点）")

    axes[2].scatter(roi[other, 0], roi[other, 1], s=4, color="#b9c0c5",
                    alpha=0.38, linewidths=0, label="ROI其他点")
    axes[2].scatter(roi[left_mask, 0], roi[left_mask, 1], s=7,
                    color="#d62728", alpha=0.72, label="左行支撑点")
    axes[2].scatter(roi[right_mask, 0], roi[right_mask, 1], s=7,
                    color="#1f77b4", alpha=0.72, label="右行支撑点")
    axes[2].plot(sample["left"][:, 0], sample["left"][:, 1],
                 color="#d62728", linewidth=2.0, label="左边界")
    axes[2].plot(sample["right"][:, 0], sample["right"][:, 1],
                 color="#1f77b4", linewidth=2.0, label="右边界")
    axes[2].plot(sample["center"][:, 0], sample["center"][:, 1],
                 color="#2ca02c", linewidth=2.5, label="中心线")
    axes[2].scatter([0], [0], marker="^", s=80, color="#222", label="机器人")
    axes[2].set_title("(c) 双行提取与中心线生成")
    axes[2].legend(loc="upper right", fontsize=8)

    for ax in axes:
        ax.set_xlim(-0.2, 2.5)
        ax.set_ylim(-1.2, 1.2)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("车体前向 x（m）")
        ax.set_ylabel("车体横向 y（m）")
    save_figure(fig, output_dir, "04_real_pointcloud_centerline")

    return {
        "raw_visible_points": int(len(raw)),
        "roi_points": int(len(roi)),
        "left_support_points": int(left_mask.sum()),
        "right_support_points": int(right_mask.sum()),
        "other_roi_points": int(other.sum()),
        "ros_timestamp_s": sample["timestamp_s"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    configure_plot()

    analyses = []
    for case in CASES:
        print(f"Analyzing {case.result_name} ...")
        analyses.append(analyze_case(case, args.data_root))
    metrics = [item[0] for item in analyses]
    write_metrics(args.output_dir, metrics)
    plot_success_trajectories(args.output_dir, analyses)
    r03_metrics, r03_arrays = analyses[2]
    plot_r03_control(args.output_dir, r03_metrics, r03_arrays)
    plot_summary(args.output_dir, metrics)
    pointcloud = load_representative_pointcloud(CASES[2], r03_arrays, args.data_root)
    point_metrics = plot_pointcloud(args.output_dir, pointcloud)
    with (args.output_dir / "data" / "r03_pointcloud_frame_metrics.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(point_metrics, handle, ensure_ascii=False, indent=2)
    print(f"Output: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
