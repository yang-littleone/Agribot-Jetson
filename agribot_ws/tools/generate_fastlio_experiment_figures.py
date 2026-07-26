#!/usr/bin/env python3
"""Generate reproducible figures for the recorded FAST-LIO odometry tests.

Inputs are ROS 2 sqlite3 bags in the workspace root.  The program does not
modify a bag; it only creates PNG figures under docs/experiments/figures.
"""

from __future__ import annotations

import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / "docs" / "experiments" / "figures"


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1.0e-9


def planar_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def read_bag(bag_name: str, requested_topics: set[str]) -> dict[str, np.ndarray]:
    """Read header time and planar odometry / IMU fields needed by the plots."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(ROOT / bag_name), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_names[topic])
        for topic in requested_topics
        if topic in type_names
    }
    samples: dict[str, list[tuple[float, ...]]] = defaultdict(list)

    while reader.has_next():
        topic, raw_data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(raw_data, message_types[topic])
        header_time = stamp_seconds(message.header.stamp)
        if "imu" in topic:
            samples[topic].append((header_time, planar_yaw(message.orientation)))
        else:
            position = message.pose.pose.position
            samples[topic].append(
                (
                    header_time,
                    position.x,
                    position.y,
                    planar_yaw(message.pose.pose.orientation),
                )
            )
    return {topic: np.asarray(values, dtype=float) for topic, values in samples.items()}


def relative_yaw_degrees(data: np.ndarray, yaw_column: int) -> np.ndarray:
    yaw = np.unwrap(data[:, yaw_column])
    return np.degrees(yaw - yaw[0])


def draw_static_position_figure() -> None:
    data = read_bag(
        "h30_test_01_static_fastlio",
        {"/Odometry", "/odometry/filtered"},
    )
    raw_lio = data["/Odometry"]
    final_odom = data["/odometry/filtered"]
    reference = raw_lio[: max(1, np.searchsorted(raw_lio[:, 0], raw_lio[0, 0] + 10.0)), 1:3].mean(axis=0)

    figure, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=False, constrained_layout=True)
    axes[0].plot(
        raw_lio[:, 0] - raw_lio[0, 0],
        (raw_lio[:, 1] - reference[0]) * 1000.0,
        label="Raw FAST-LIO x",
        linewidth=0.9,
    )
    axes[0].plot(
        raw_lio[:, 0] - raw_lio[0, 0],
        (raw_lio[:, 2] - reference[1]) * 1000.0,
        label="Raw FAST-LIO y",
        linewidth=0.9,
    )
    axes[0].set(title="Static test: raw FAST-LIO planar variation", ylabel="Position relative to first 10 s mean (mm)")
    axes[0].grid(alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(
        final_odom[:, 0] - final_odom[0, 0],
        (final_odom[:, 1] - final_odom[0, 1]) * 1000.0,
        label="Final odom x",
        linewidth=1.2,
    )
    axes[1].plot(
        final_odom[:, 0] - final_odom[0, 0],
        (final_odom[:, 2] - final_odom[0, 2]) * 1000.0,
        label="Final odom y",
        linewidth=1.2,
    )
    axes[1].set(title="Static test: final odometry translation hold", xlabel="Time (s)", ylabel="Position relative to start (mm)")
    axes[1].grid(alpha=0.3)
    axes[1].legend(loc="upper right")
    figure.savefig(FIGURES / "fastlio_static_translation.png", dpi=180)
    plt.close(figure)


def draw_turn_figures() -> None:
    data = read_bag(
        "h30_test_02_turn_90_fastlio",
        {"/imu/data_h30", "/Odometry", "/lio/odom", "/wheel/odom", "/odometry/filtered"},
    )
    h30 = data["/imu/data_h30"]
    raw_lio = data["/Odometry"]
    lio = data["/lio/odom"]
    wheel = data["/wheel/odom"]
    final_odom = data["/odometry/filtered"]
    epoch = h30[0, 0]

    figure, axis = plt.subplots(figsize=(10, 5.5), constrained_layout=True)
    axis.plot(h30[:, 0] - epoch, relative_yaw_degrees(h30, 1), label="H30 /imu/data_h30", linewidth=1.0)
    axis.plot(raw_lio[:, 0] - epoch, relative_yaw_degrees(raw_lio, 3), label="Raw FAST-LIO /Odometry", linewidth=1.0)
    axis.plot(lio[:, 0] - epoch, relative_yaw_degrees(lio, 3), label="Aligned LIO /lio/odom", linewidth=1.0, linestyle="--")
    axis.plot(final_odom[:, 0] - epoch, relative_yaw_degrees(final_odom, 3), label="Final /odometry/filtered", linewidth=1.4)
    axis.plot(wheel[:, 0] - epoch, relative_yaw_degrees(wheel, 3), label="Wheel odom", linewidth=1.0, linestyle=":")
    axis.axhline(-90.0, color="black", linewidth=0.8, alpha=0.6, label="-90 deg reference")
    axis.set(
        title="Manual body rotation with stationary wheels: yaw response",
        xlabel="Time from H30 bag start (s)",
        ylabel="Yaw change from each stream start (deg)",
    )
    axis.grid(alpha=0.3)
    axis.legend(loc="lower left", ncol=2)
    figure.savefig(FIGURES / "fastlio_manual_turn_yaw.png", dpi=180)
    plt.close(figure)

    relative_xy_cm = (final_odom[:, 1:3] - final_odom[0, 1:3]) * 100.0
    figure, axis = plt.subplots(figsize=(6.2, 6.0), constrained_layout=True)
    line = axis.plot(relative_xy_cm[:, 0], relative_xy_cm[:, 1], linewidth=1.2, label="Final odom path")[0]
    axis.scatter(relative_xy_cm[0, 0], relative_xy_cm[0, 1], c="green", s=55, label="Start", zorder=3)
    axis.scatter(relative_xy_cm[-1, 0], relative_xy_cm[-1, 1], c="red", s=55, label="End", zorder=3)
    axis.set(
        title="Final odometry translation during the manual-rotation test",
        xlabel="x relative to start (cm)",
        ylabel="y relative to start (cm)",
        aspect="equal",
    )
    axis.grid(alpha=0.3)
    axis.legend(loc="best")
    figure.savefig(FIGURES / "fastlio_manual_turn_xy.png", dpi=180)
    plt.close(figure)


def read_drive_bag(bag_name: str) -> dict[str, np.ndarray]:
    """Read the fields used by the wheel-slip and normal-drive figures."""
    topics = {
        "/cmd_vel",
        "/wheel/odom",
        "/wheel/odom_validated",
        "/lio/odom",
        "/odometry/filtered",
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(ROOT / bag_name), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_names[topic]) for topic in topics}
    samples: dict[str, list[tuple[float, ...]]] = defaultdict(list)
    while reader.has_next():
        topic, raw_data, bag_time_nanoseconds = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(raw_data, message_types[topic])
        if topic == "/cmd_vel":
            samples[topic].append((bag_time_nanoseconds * 1.0e-9, message.linear.x))
            continue
        position = message.pose.pose.position
        samples[topic].append(
            (
                stamp_seconds(message.header.stamp),
                position.x,
                position.y,
                planar_yaw(message.pose.pose.orientation),
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.covariance[0],
            )
        )
    return {topic: np.asarray(values, dtype=float) for topic, values in samples.items()}


def nearest_index(times: np.ndarray, target: float) -> int:
    return int(np.argmin(np.abs(times - target)))


def motion_window(raw_wheel: np.ndarray) -> tuple[float, float]:
    speed = np.hypot(raw_wheel[:, 4], raw_wheel[:, 5])
    moving = np.flatnonzero(speed > 0.05)
    return raw_wheel[moving[0], 0], raw_wheel[moving[-1], 0]


def radial_displacement(odometry: np.ndarray, start_time: float) -> tuple[np.ndarray, np.ndarray]:
    start_index = nearest_index(odometry[:, 0], start_time)
    relative_xy = odometry[:, 1:3] - odometry[start_index, 1:3]
    return odometry[:, 0] - start_time, np.hypot(relative_xy[:, 0], relative_xy[:, 1])


def forward_displacement(odometry: np.ndarray, start_time: float) -> tuple[np.ndarray, np.ndarray]:
    start_index = nearest_index(odometry[:, 0], start_time)
    heading = odometry[start_index, 3]
    relative_xy = odometry[:, 1:3] - odometry[start_index, 1:3]
    forward = np.cos(heading) * relative_xy[:, 0] + np.sin(heading) * relative_xy[:, 1]
    return odometry[:, 0] - start_time, forward


def draw_blocked_wheel_figure() -> None:
    data = read_drive_bag("h30_test_03_blocked_wheel_spin_fastlio")
    command = data["/cmd_vel"]
    command_times = command[command[:, 1] > 0.02, 0]
    start_time, end_time = command_times[0], command_times[-1]

    figure, axes = plt.subplots(2, 1, figsize=(10, 7.0), sharex=True, constrained_layout=True)
    for topic, label, color in [
        ("/wheel/odom", "Raw wheel odom", "tab:orange"),
        ("/lio/odom", "Aligned LIO", "tab:blue"),
        ("/odometry/filtered", "Final odom", "tab:red"),
    ]:
        time, distance = radial_displacement(data[topic], start_time)
        axes[0].plot(time, distance, label=label, color=color, linewidth=1.2)
    axes[0].axvspan(0.0, end_time - start_time, color="gray", alpha=0.13, label="Forward command active")
    axes[0].set(title="Blocked-wheel test: reported displacement during wheel spin", ylabel="Displacement from command start (m)", ylim=(-0.04, 2.55))
    axes[0].grid(alpha=0.3)
    axes[0].legend(loc="upper left")

    final_time, final_distance = radial_displacement(data["/odometry/filtered"], start_time)
    axes[1].plot(final_time, final_distance * 100.0, color="tab:red", label="Final odom radial displacement")
    axes[1].axhline(2.0, color="black", linestyle="--", linewidth=0.9, label="2 cm target")
    validated = data["/wheel/odom_validated"]
    axes_covariance = axes[1].twinx()
    axes_covariance.semilogy(
        validated[:, 0] - start_time,
        validated[:, 6],
        color="tab:purple",
        alpha=0.8,
        label="Validated-wheel vx variance",
    )
    axes[1].axvspan(0.0, end_time - start_time, color="gray", alpha=0.13)
    axes[1].set(xlabel="Time from nonzero command (s)", ylabel="Final displacement (cm)")
    axes_covariance.set_ylabel("Validated-wheel vx variance (log scale)")
    axes[1].grid(alpha=0.3)
    handles, labels = axes[1].get_legend_handles_labels()
    handles2, labels2 = axes_covariance.get_legend_handles_labels()
    axes[1].legend(handles + handles2, labels + labels2, loc="upper right")
    figure.savefig(FIGURES / "fastlio_blocked_wheel_spin.png", dpi=180)
    plt.close(figure)


def draw_normal_drive_figure() -> None:
    data = read_drive_bag("h30_test_04_normal_forward_fastlio")
    start_time, end_time = motion_window(data["/wheel/odom"])
    figure, axis = plt.subplots(figsize=(10, 5.7), constrained_layout=True)
    for topic, label, color in [
        ("/wheel/odom", "Raw wheel odom", "tab:orange"),
        ("/lio/odom", "Aligned LIO", "tab:blue"),
        ("/odometry/filtered", "Final odom", "tab:red"),
    ]:
        time, distance = forward_displacement(data[topic], start_time)
        within_motion = (time >= -0.5) & (time <= end_time - start_time + 1.0)
        axis.plot(time[within_motion], distance[within_motion], label=label, color=color, linewidth=1.1)
    axis.axhspan(5.3, 5.4, color="green", alpha=0.13, label="Tape-measured endpoint range")
    axis.axvline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axis.axvline(end_time - start_time, color="black", linestyle="--", linewidth=0.8, alpha=0.6, label="Wheel motion end")
    axis.set(
        title="Normal 5.3–5.4 m drive: forward displacement comparison",
        xlabel="Time from wheel-motion start (s)",
        ylabel="Forward displacement from motion start (m)",
    )
    axis.grid(alpha=0.3)
    axis.legend(loc="upper left", ncol=2)
    figure.savefig(FIGURES / "fastlio_normal_drive_distance.png", dpi=180)
    plt.close(figure)


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    draw_static_position_figure()
    draw_turn_figures()
    draw_blocked_wheel_figure()
    draw_normal_drive_figure()
    print(f"Generated figures in {FIGURES}")


if __name__ == "__main__":
    main()
