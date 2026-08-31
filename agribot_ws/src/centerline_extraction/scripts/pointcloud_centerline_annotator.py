#!/usr/bin/env python3
"""
Interactively annotate ground truth for corn-row PointCloud2 rosbag data.

The source bag is always opened read-only.  Compressed SQLite files are
decompressed into a workspace cache, never beside or over the source bag.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
from collections import deque

import numpy as np


DEFAULT_WORKSPACE = Path("/home/wheeltec/agribot/agribot_ws")
DEFAULT_OUTPUT_ROOT = (
    DEFAULT_WORKSPACE / "field_ground_truth" / "pointcloud_annotations"
)
DEFAULT_CACHE_ROOT = (
    DEFAULT_WORKSPACE / "field_ground_truth" / "pointcloud_annotation_cache"
)
ANNOTATION_VERSION = 2


def normalize_frame(frame):
    return str(frame).strip().lstrip("/")


def quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return np.eye(3, dtype=float)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def transform_matrix(translation, quaternion):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_matrix(*quaternion)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def quaternion_from_matrix(matrix):
    rotation = np.asarray(matrix, dtype=float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w_value = 0.25 * scale
        x_value = (rotation[2, 1] - rotation[1, 2]) / scale
        y_value = (rotation[0, 2] - rotation[2, 0]) / scale
        z_value = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            w_value = (rotation[2, 1] - rotation[1, 2]) / scale
            x_value = 0.25 * scale
            y_value = (rotation[0, 1] + rotation[1, 0]) / scale
            z_value = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            w_value = (rotation[0, 2] - rotation[2, 0]) / scale
            x_value = (rotation[0, 1] + rotation[1, 0]) / scale
            y_value = 0.25 * scale
            z_value = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            w_value = (rotation[1, 0] - rotation[0, 1]) / scale
            x_value = (rotation[0, 2] + rotation[2, 0]) / scale
            y_value = (rotation[1, 2] + rotation[2, 1]) / scale
            z_value = 0.25 * scale
    quaternion = np.array(
        [x_value, y_value, z_value, w_value], dtype=float
    )
    norm = np.linalg.norm(quaternion)
    return quaternion / norm if norm > 1.0e-12 else np.array(
        [0.0, 0.0, 0.0, 1.0]
    )


def rpy_transform(translation, rpy_deg):
    roll, pitch, yaw = np.deg2rad(np.asarray(rpy_deg, dtype=float))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def apply_transform(points, matrix):
    if points.size == 0:
        return points.copy()
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def fit_parallel_rows(left_points, right_points):
    """
    Fit y=a*x+b_left/right with one shared slope.

    The manually selected stalk centers remain the ground-truth observations;
    the shared slope only encodes the physical parallel-row constraint.
    """
    left = np.asarray(left_points, dtype=float)
    right = np.asarray(right_points, dtype=float)
    if left.shape[0] < 2 or right.shape[0] < 2:
        raise ValueError("左右两侧至少各标注2个茎秆中心")
    values = np.vstack((left, right))
    design = np.zeros((values.shape[0], 3), dtype=float)
    design[:, 0] = values[:, 0]
    design[: left.shape[0], 1] = 1.0
    design[left.shape[0]:, 2] = 1.0
    slope, left_intercept, right_intercept = np.linalg.lstsq(
        design, values[:, 1], rcond=None
    )[0]
    residuals = values[:, 1] - design @ np.array(
        [slope, left_intercept, right_intercept]
    )
    return {
        "slope": float(slope),
        "left_intercept_m": float(left_intercept),
        "right_intercept_m": float(right_intercept),
        "center_intercept_m": float(
            0.5 * (left_intercept + right_intercept)
        ),
        "row_width_m": float(
            abs(left_intercept - right_intercept)
            / math.sqrt(1.0 + slope * slope)
        ),
        "heading_deg": float(math.degrees(math.atan(slope))),
        "fit_rmse_m": float(math.sqrt(np.mean(residuals * residuals))),
    }


def sampled_frame_indices(timestamps_ns, period_s):
    if not timestamps_ns:
        return []
    if period_s <= 0.0:
        return list(range(len(timestamps_ns)))
    result = [0]
    last_timestamp = timestamps_ns[0]
    period_ns = int(period_s * 1.0e9)
    for index, timestamp in enumerate(timestamps_ns[1:], start=1):
        if timestamp - last_timestamp >= period_ns:
            result.append(index)
            last_timestamp = timestamp
    if result[-1] != len(timestamps_ns) - 1:
        result.append(len(timestamps_ns) - 1)
    return result


class TransformGraph:
    def __init__(self):
        self._edges = {}

    def add(self, parent, child, parent_from_child):
        parent = normalize_frame(parent)
        child = normalize_frame(child)
        if not parent or not child:
            return
        self._edges.setdefault(child, {})[parent] = parent_from_child
        self._edges.setdefault(parent, {})[child] = np.linalg.inv(
            parent_from_child
        )

    def lookup(self, target, source):
        target = normalize_frame(target)
        source = normalize_frame(source)
        if target == source:
            return np.eye(4, dtype=float)
        queue = deque([(source, np.eye(4, dtype=float))])
        visited = {source}
        while queue:
            current, current_from_source = queue.popleft()
            for neighbor, neighbor_from_current in self._edges.get(
                current, {}
            ).items():
                if neighbor in visited:
                    continue
                neighbor_from_source = (
                    neighbor_from_current @ current_from_source
                )
                if neighbor == target:
                    return neighbor_from_source
                visited.add(neighbor)
                queue.append((neighbor, neighbor_from_source))
        raise KeyError(f"TF中找不到 {source} → {target} 的变换")


class RosbagPointCloudReader:
    def __init__(
        self,
        bag_path,
        topic="/livox/lidar",
        target_frame="base_link",
        cache_root=DEFAULT_CACHE_ROOT,
        manual_transform=None,
    ):
        self.source_path = Path(bag_path).expanduser().resolve()
        self.topic = topic
        self.target_frame = normalize_frame(target_frame)
        self.cache_root = Path(cache_root).expanduser().resolve()
        self.manual_transform = manual_transform
        self.bag_name = (
            self.source_path.name
            if self.source_path.is_dir()
            else self.source_path.name.split(".db3")[0]
        )
        self.db_paths = self._resolve_databases()
        self.frames = self._index_frames()
        if not self.frames:
            raise RuntimeError(
                f"{self.source_path} 中没有 {self.topic} 点云消息"
            )
        self.graph = self._read_static_transforms()

    def _resolve_databases(self):
        if not self.source_path.exists():
            raise FileNotFoundError(f"rosbag路径不存在：{self.source_path}")
        if self.source_path.is_dir():
            db_paths = sorted(self.source_path.glob("*.db3"))
            compressed = sorted(self.source_path.glob("*.db3.zstd"))
        elif self.source_path.name.endswith(".db3"):
            db_paths = [self.source_path]
            compressed = []
        elif self.source_path.name.endswith(".db3.zstd"):
            db_paths = []
            compressed = [self.source_path]
        else:
            raise ValueError("请选择rosbag目录、.db3或.db3.zstd文件")
        resolved = list(db_paths)
        existing_names = {path.name for path in db_paths}
        for compressed_path in compressed:
            expected_name = compressed_path.name[: -len(".zstd")]
            if expected_name not in existing_names:
                resolved.append(self._cached_decompress(compressed_path))
        if not resolved:
            raise RuntimeError(f"{self.source_path} 中没有SQLite rosbag文件")
        return sorted(resolved)

    def _cached_decompress(self, compressed_path):
        cache_dir = self.cache_root / self.bag_name
        cache_dir.mkdir(parents=True, exist_ok=True)
        output = cache_dir / compressed_path.name[: -len(".zstd")]
        if output.exists() and output.stat().st_size > 0:
            return output
        if shutil.which("zstd") is None:
            raise RuntimeError("未找到zstd，无法读取压缩rosbag")
        free_bytes = shutil.disk_usage(cache_dir).free
        if free_bytes < compressed_path.stat().st_size * 2:
            raise RuntimeError(
                "工作空间剩余空间不足，无法建立rosbag只读解压缓存"
            )
        partial = output.with_suffix(output.suffix + ".partial")
        if partial.exists():
            partial.unlink()
        print(f"正在建立只读缓存：{compressed_path.name}", flush=True)
        try:
            subprocess.run(
                [
                    "zstd",
                    "-d",
                    "-f",
                    "-q",
                    str(compressed_path),
                    "-o",
                    str(partial),
                ],
                check=True,
            )
            os.replace(partial, output)
        finally:
            if partial.exists():
                partial.unlink()
        return output

    @staticmethod
    def _readonly_connection(path):
        uri = f"file:{path}?mode=ro"
        return sqlite3.connect(uri, uri=True)

    def _index_frames(self):
        frames = []
        for db_path in self.db_paths:
            connection = self._readonly_connection(db_path)
            try:
                topic_row = connection.execute(
                    "SELECT id, type FROM topics WHERE name = ?",
                    (self.topic,),
                ).fetchone()
                if topic_row is None:
                    continue
                rows = connection.execute(
                    """
                    SELECT id, timestamp FROM messages
                    WHERE topic_id = ? ORDER BY timestamp
                    """,
                    (topic_row[0],),
                ).fetchall()
                frames.extend(
                    {
                        "db_path": str(db_path),
                        "message_id": int(message_id),
                        "bag_timestamp_ns": int(timestamp),
                    }
                    for message_id, timestamp in rows
                )
            finally:
                connection.close()
        frames.sort(key=lambda item: item["bag_timestamp_ns"])
        return frames

    def _read_static_transforms(self):
        from rclpy.serialization import deserialize_message
        from tf2_msgs.msg import TFMessage

        graph = TransformGraph()
        for db_path in self.db_paths:
            connection = self._readonly_connection(db_path)
            try:
                rows = connection.execute(
                    """
                    SELECT messages.data
                    FROM messages JOIN topics
                      ON messages.topic_id = topics.id
                    WHERE topics.name = '/tf_static'
                    ORDER BY messages.timestamp
                    """
                ).fetchall()
            finally:
                connection.close()
            for (serialized,) in rows:
                message = deserialize_message(serialized, TFMessage)
                for transform in message.transforms:
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    graph.add(
                        transform.header.frame_id,
                        transform.child_frame_id,
                        transform_matrix(
                            (translation.x, translation.y, translation.z),
                            (rotation.x, rotation.y, rotation.z, rotation.w),
                        ),
                    )
        return graph

    def load_frame(self, frame_index):
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2

        frame = self.frames[frame_index]
        connection = self._readonly_connection(Path(frame["db_path"]))
        try:
            row = connection.execute(
                "SELECT data FROM messages WHERE id = ?",
                (frame["message_id"],),
            ).fetchone()
        finally:
            connection.close()
        if row is None:
            raise IndexError(f"无法读取第{frame_index}帧")
        message = deserialize_message(row[0], PointCloud2)
        available_fields = {field.name for field in message.fields}
        has_intensity = "intensity" in available_fields
        field_names = (
            ("x", "y", "z", "intensity")
            if has_intensity
            else ("x", "y", "z")
        )
        records = point_cloud2.read_points(
            message, field_names=field_names, skip_nans=True
        )
        if getattr(records.dtype, "names", None):
            xyz = np.column_stack(
                (records["x"], records["y"], records["z"])
            ).astype(np.float32, copy=False)
            intensity = (
                np.asarray(records["intensity"], dtype=np.float32)
                if has_intensity
                else np.zeros(xyz.shape[0], dtype=np.float32)
            )
        else:
            values = np.asarray(records, dtype=np.float32).reshape(
                -1, len(field_names)
            )
            xyz = values[:, :3]
            intensity = (
                values[:, 3]
                if has_intensity
                else np.zeros(xyz.shape[0], dtype=np.float32)
            )
        source_frame = normalize_frame(message.header.frame_id)
        if source_frame == self.target_frame:
            matrix = np.eye(4, dtype=float)
            transform_source = "identity"
        elif self.manual_transform is not None:
            matrix = self.manual_transform
            transform_source = "manual"
        else:
            matrix = self.graph.lookup(self.target_frame, source_frame)
            transform_source = "bag_tf_static"
        xyz = apply_transform(xyz, matrix).astype(
            np.float32, copy=False
        )
        points = np.column_stack((xyz, intensity)).astype(
            np.float32, copy=False
        )
        cloud_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        metadata = dict(frame)
        metadata.update(
            {
                "frame_index": int(frame_index),
                "cloud_stamp_ns": cloud_stamp_ns,
                "cloud_frame": source_frame,
                "target_frame": self.target_frame,
                "transform_source": transform_source,
                "transform_matrix": matrix.tolist(),
                "raw_point_count": int(points.shape[0]),
                "has_intensity": has_intensity,
            }
        )
        return points, metadata

    def load_nearest_odometry(
        self, frame_index, odom_topic="/odometry/filtered"
    ):
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message

        target_timestamp = self.frames[frame_index]["bag_timestamp_ns"]
        candidates = []
        for db_path in self.db_paths:
            connection = self._readonly_connection(db_path)
            try:
                topic_row = connection.execute(
                    "SELECT id FROM topics WHERE name = ?", (odom_topic,)
                ).fetchone()
                if topic_row is None:
                    continue
                before = connection.execute(
                    """
                    SELECT timestamp, data FROM messages
                    WHERE topic_id = ? AND timestamp <= ?
                    ORDER BY timestamp DESC LIMIT 1
                    """,
                    (topic_row[0], target_timestamp),
                ).fetchone()
                after = connection.execute(
                    """
                    SELECT timestamp, data FROM messages
                    WHERE topic_id = ? AND timestamp >= ?
                    ORDER BY timestamp ASC LIMIT 1
                    """,
                    (topic_row[0], target_timestamp),
                ).fetchone()
                candidates.extend(
                    row for row in (before, after) if row is not None
                )
            finally:
                connection.close()
        if not candidates:
            raise RuntimeError(f"rosbag中没有{odom_topic}消息")
        timestamp, serialized = min(
            candidates,
            key=lambda row: abs(int(row[0]) - target_timestamp),
        )
        message = deserialize_message(serialized, Odometry)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        parent_frame = normalize_frame(message.header.frame_id)
        child_frame = normalize_frame(message.child_frame_id)
        parent_from_child = transform_matrix(
            (position.x, position.y, position.z),
            (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        if child_frame and child_frame != self.target_frame:
            try:
                child_from_target = self.graph.lookup(
                    child_frame, self.target_frame
                )
                parent_from_target = (
                    parent_from_child @ child_from_target
                )
            except KeyError:
                parent_from_target = parent_from_child
        else:
            parent_from_target = parent_from_child
        quaternion = quaternion_from_matrix(parent_from_target)
        return {
            "parent_frame": parent_frame or "odom",
            "child_frame": self.target_frame,
            "translation": parent_from_target[:3, 3].tolist(),
            "quaternion": quaternion.tolist(),
            "bag_timestamp_ns": int(timestamp),
            "time_delta_s": (
                int(timestamp) - int(target_timestamp)
            )
            / 1.0e9,
        }

    def distance_sample_indices(
        self,
        distance_m=0.5,
        odom_topic="/odometry/filtered",
        moving_speed_threshold=0.03,
    ):
        """Select moving cloud frames at approximately equal travel distances."""
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message

        odometry = []
        for db_path in self.db_paths:
            connection = self._readonly_connection(db_path)
            try:
                topic_row = connection.execute(
                    "SELECT id FROM topics WHERE name = ?", (odom_topic,)
                ).fetchone()
                if topic_row is None:
                    continue
                rows = connection.execute(
                    """
                    SELECT timestamp, data FROM messages
                    WHERE topic_id = ? ORDER BY timestamp
                    """,
                    (topic_row[0],),
                ).fetchall()
            finally:
                connection.close()
            for timestamp, serialized in rows:
                message = deserialize_message(serialized, Odometry)
                position = message.pose.pose.position
                velocity = message.twist.twist.linear
                odometry.append(
                    (
                        int(timestamp),
                        float(position.x),
                        float(position.y),
                        math.hypot(float(velocity.x), float(velocity.y)),
                    )
                )
        if not odometry:
            return [], "no_odometry"
        odometry.sort(key=lambda item: item[0])
        cumulative = np.zeros(len(odometry), dtype=float)
        for index in range(1, len(odometry)):
            step = math.hypot(
                odometry[index][1] - odometry[index - 1][1],
                odometry[index][2] - odometry[index - 1][2],
            )
            # Reject localization resets; this distance is only for sampling.
            cumulative[index] = cumulative[index - 1] + (
                step if step <= 0.5 else 0.0
            )
        selected = []
        cursor = 0
        last_distance = None
        last_moving_index = None
        for frame_index, frame in enumerate(self.frames):
            timestamp = frame["bag_timestamp_ns"]
            while (
                cursor + 1 < len(odometry)
                and abs(odometry[cursor + 1][0] - timestamp)
                <= abs(odometry[cursor][0] - timestamp)
            ):
                cursor += 1
            if odometry[cursor][3] < moving_speed_threshold:
                continue
            last_moving_index = frame_index
            distance = cumulative[cursor]
            if (
                last_distance is None
                or distance - last_distance >= distance_m
            ):
                selected.append(frame_index)
                last_distance = distance
        if (
            last_moving_index is not None
            and selected
            and selected[-1] != last_moving_index
        ):
            selected.append(last_moving_index)
        return selected, "distance"


class AnnotationStore:
    CSV_FIELDS = [
        "bag_name",
        "bag_path",
        "sampling_anchor_frame_index",
        "frame_index",
        "selected_frame_delta",
        "selected_time_delta_s",
        "bag_timestamp_ns",
        "cloud_stamp_ns",
        "cloud_stamp_s",
        "cloud_frame",
        "target_frame",
        "status",
        "invalid_reason",
        "left_point_count",
        "right_point_count",
        "left_points_xy",
        "right_points_xy",
        "slope",
        "left_intercept_m",
        "right_intercept_m",
        "true_center_offset_m",
        "true_row_yaw_deg",
        "row_width_m",
        "fit_rmse_m",
        "roi_x_min_m",
        "roi_x_max_m",
        "roi_y_min_m",
        "roi_y_max_m",
        "roi_z_min_m",
        "roi_z_max_m",
        "annotator",
        "saved_at",
    ]

    def __init__(self, output_dir, reader, annotator, sample_period_s):
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.json_path = self.output_dir / "centerline_annotations.json"
        self.csv_path = self.output_dir / "centerline_ground_truth.csv"
        self.reader = reader
        self.annotator = annotator
        self.sample_period_s = float(sample_period_s)
        self.annotations = {}
        if self.json_path.exists():
            with self.json_path.open("r", encoding="utf-8") as stream:
                document = json.load(stream)
            source = Path(document.get("bag_path", "")).expanduser()
            if source and source.resolve() != reader.source_path:
                raise RuntimeError(
                    f"输出目录已有另一rosbag的标注：{source}"
                )
            self.annotations = document.get("annotations", {})

    def get(self, frame_index):
        return self.annotations.get(str(frame_index))

    def find_by_anchor(self, anchor_frame_index):
        for annotation in self.annotations.values():
            stored_anchor = annotation.get(
                "sampling_anchor_frame_index",
                annotation.get("frame_index"),
            )
            if int(stored_anchor) == int(anchor_frame_index):
                return annotation
        return None

    def save(
        self,
        frame_metadata,
        roi,
        left_points,
        right_points,
        status,
        invalid_reason="",
        sampling_anchor_frame_index=None,
        sampling_anchor_timestamp_ns=None,
    ):
        fit = {}
        if status == "valid":
            fit = fit_parallel_rows(left_points, right_points)
            invalid_reason = ""
        selected_frame_index = int(frame_metadata["frame_index"])
        if sampling_anchor_frame_index is None:
            sampling_anchor_frame_index = selected_frame_index
        if sampling_anchor_timestamp_ns is None:
            sampling_anchor_timestamp_ns = int(
                frame_metadata["bag_timestamp_ns"]
            )
        annotation = {
            "bag_name": self.reader.bag_name,
            "bag_path": str(self.reader.source_path),
            "sampling_anchor_frame_index": int(
                sampling_anchor_frame_index
            ),
            "sampling_anchor_timestamp_ns": int(
                sampling_anchor_timestamp_ns
            ),
            "frame_index": selected_frame_index,
            "selected_frame_delta": (
                selected_frame_index - int(sampling_anchor_frame_index)
            ),
            "selected_time_delta_s": (
                int(frame_metadata["bag_timestamp_ns"])
                - int(sampling_anchor_timestamp_ns)
            )
            / 1.0e9,
            "bag_timestamp_ns": int(frame_metadata["bag_timestamp_ns"]),
            "cloud_stamp_ns": int(frame_metadata["cloud_stamp_ns"]),
            "cloud_frame": frame_metadata["cloud_frame"],
            "target_frame": frame_metadata["target_frame"],
            "transform_source": frame_metadata["transform_source"],
            "transform_matrix": frame_metadata["transform_matrix"],
            "raw_point_count": int(frame_metadata["raw_point_count"]),
            "roi": {key: float(value) for key, value in roi.items()},
            "left_points_xy": [
                [float(point[0]), float(point[1])] for point in left_points
            ],
            "right_points_xy": [
                [float(point[0]), float(point[1])] for point in right_points
            ],
            "status": status,
            "invalid_reason": invalid_reason,
            "annotator": self.annotator,
            "saved_at": datetime.now(timezone.utc).isoformat(),
        }
        annotation.update(fit)
        self.annotations[str(frame_metadata["frame_index"])] = annotation
        self._write()

    def _write(self):
        document = {
            "format": "corn-row-pointcloud-centerline-ground-truth",
            "version": ANNOTATION_VERSION,
            "bag_name": self.reader.bag_name,
            "bag_path": str(self.reader.source_path),
            "cloud_topic": self.reader.topic,
            "target_frame": self.reader.target_frame,
            "sample_period_s": self.sample_period_s,
            "annotations": self.annotations,
        }
        temporary = self.json_path.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary, self.json_path)
        with self.csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.CSV_FIELDS)
            writer.writeheader()
            for key in sorted(self.annotations, key=lambda value: int(value)):
                annotation = self.annotations[key]
                roi = annotation["roi"]
                writer.writerow(
                    {
                        "bag_name": annotation["bag_name"],
                        "bag_path": annotation["bag_path"],
                        "sampling_anchor_frame_index": annotation.get(
                            "sampling_anchor_frame_index",
                            annotation["frame_index"],
                        ),
                        "frame_index": annotation["frame_index"],
                        "selected_frame_delta": annotation.get(
                            "selected_frame_delta", 0
                        ),
                        "selected_time_delta_s": annotation.get(
                            "selected_time_delta_s", 0.0
                        ),
                        "bag_timestamp_ns": annotation["bag_timestamp_ns"],
                        "cloud_stamp_ns": annotation["cloud_stamp_ns"],
                        "cloud_stamp_s": (
                            annotation["cloud_stamp_ns"] / 1.0e9
                        ),
                        "cloud_frame": annotation["cloud_frame"],
                        "target_frame": annotation["target_frame"],
                        "status": annotation["status"],
                        "invalid_reason": annotation["invalid_reason"],
                        "left_point_count": len(
                            annotation["left_points_xy"]
                        ),
                        "right_point_count": len(
                            annotation["right_points_xy"]
                        ),
                        "left_points_xy": json.dumps(
                            annotation["left_points_xy"],
                            ensure_ascii=False,
                        ),
                        "right_points_xy": json.dumps(
                            annotation["right_points_xy"],
                            ensure_ascii=False,
                        ),
                        "slope": annotation.get("slope", ""),
                        "left_intercept_m": annotation.get(
                            "left_intercept_m", ""
                        ),
                        "right_intercept_m": annotation.get(
                            "right_intercept_m", ""
                        ),
                        "true_center_offset_m": annotation.get(
                            "center_intercept_m", ""
                        ),
                        "true_row_yaw_deg": annotation.get(
                            "heading_deg", ""
                        ),
                        "row_width_m": annotation.get("row_width_m", ""),
                        "fit_rmse_m": annotation.get("fit_rmse_m", ""),
                        "roi_x_min_m": roi["x_min"],
                        "roi_x_max_m": roi["x_max"],
                        "roi_y_min_m": roi["y_min"],
                        "roi_y_max_m": roi["y_max"],
                        "roi_z_min_m": roi["z_min"],
                        "roi_z_max_m": roi["z_max"],
                        "annotator": annotation["annotator"],
                        "saved_at": annotation["saved_at"],
                    }
                )


def launch_gui(args, reader, store, sample_indices, sampling_method):
    from PyQt5 import QtCore, QtGui, QtWidgets
    from matplotlib import font_manager, rcParams
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure

    chinese_font = Path(
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    )
    if chinese_font.exists():
        font_manager.fontManager.addfont(str(chinese_font))
        rcParams["font.sans-serif"] = [
            font_manager.FontProperties(fname=str(chinese_font)).get_name()
        ]
        rcParams["axes.unicode_minus"] = False

    class AnnotationWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("玉米行点云中心线快速真值标注")
            screen = QtWidgets.QApplication.primaryScreen()
            available = screen.availableGeometry() if screen else None
            if available:
                self.resize(
                    min(1420, available.width()),
                    min(880, available.height()),
                )
            else:
                self.resize(1024, 600)
            self.reader = reader
            self.store = store
            self.sample_indices = sample_indices
            self.sampling_method = sampling_method
            self.sample_position = 0
            self.current_frame_index = self.sample_indices[0]
            self.annotation_anchor_index = self.current_frame_index
            self.annotation_anchor_timestamp_ns = reader.frames[
                self.current_frame_index
            ]["bag_timestamp_ns"]
            self.raw_points = np.empty((0, 3), dtype=np.float32)
            self.roi_points = np.empty((0, 3), dtype=np.float32)
            self.frame_metadata = {}
            self.left_points = []
            self.right_points = []
            self.active_side = "left"
            self.preview_active = False
            self.preview_anchor_index = self.current_frame_index
            self.preview_frame_index = self.current_frame_index
            self.preview_points = np.empty((0, 3), dtype=np.float32)
            self.preview_indices = []
            self.preview_position = 0
            self.preview_timer = QtCore.QTimer(self)
            self.preview_timer.timeout.connect(self.advance_preview)
            self._build_ui()
            self._bind_shortcuts()
            self.load_frame(self.current_frame_index)

        def _build_ui(self):
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QHBoxLayout(central)
            layout.setContentsMargins(4, 4, 4, 4)

            self.view_tabs = QtWidgets.QTabWidget()
            self.figure = Figure(figsize=(10, 7))
            self.canvas = FigureCanvasQTAgg(self.figure)
            self.axes = self.figure.add_subplot(111)
            self.figure.subplots_adjust(
                left=0.10, right=0.98, bottom=0.10, top=0.92
            )
            self.canvas.mpl_connect("button_press_event", self.on_plot_click)
            self.view_tabs.addTab(self.canvas, "标注俯视图")

            self.figure_3d = Figure(figsize=(10, 7))
            self.canvas_3d = FigureCanvasQTAgg(self.figure_3d)
            self.axes_3d = self.figure_3d.add_subplot(111, projection="3d")
            self.figure_3d.subplots_adjust(
                left=0.02, right=0.96, bottom=0.03, top=0.94
            )
            self.view_tabs.addTab(self.canvas_3d, "三维上下文（可拖动旋转）")
            layout.addWidget(self.view_tabs, stretch=5)

            panel = QtWidgets.QWidget()
            panel_layout = QtWidgets.QVBoxLayout(panel)
            panel_layout.setContentsMargins(5, 5, 5, 5)
            panel_scroll = QtWidgets.QScrollArea()
            panel_scroll.setWidgetResizable(True)
            panel_scroll.setHorizontalScrollBarPolicy(
                QtCore.Qt.ScrollBarAlwaysOff
            )
            panel_scroll.setMinimumWidth(285)
            panel_scroll.setMaximumWidth(345)
            panel_scroll.setWidget(panel)
            layout.addWidget(panel_scroll, stretch=2)

            help_label = QtWidgets.QLabel(
                "操作：选择“左行/右行”，在对应茎秆中心点击。"
                "左右各≥3点后保存最可靠；至少各2点可拟合。"
                "图中不显示算法结果，避免真值标注偏向算法。"
            )
            help_label.setWordWrap(True)
            panel_layout.addWidget(help_label)

            navigation = QtWidgets.QGroupBox("帧导航")
            navigation_layout = QtWidgets.QGridLayout(navigation)
            self.frame_spin = QtWidgets.QSpinBox()
            self.frame_spin.setRange(0, len(reader.frames) - 1)
            self.frame_spin.setValue(self.current_frame_index)
            go_button = QtWidgets.QPushButton("跳转")
            previous_button = QtWidgets.QPushButton("上一个抽样帧 [P]")
            next_button = QtWidgets.QPushButton("下一个抽样帧 [N]")
            go_button.clicked.connect(
                lambda: self.load_frame(self.frame_spin.value())
            )
            previous_button.clicked.connect(self.previous_sample)
            next_button.clicked.connect(self.next_sample)
            navigation_layout.addWidget(
                QtWidgets.QLabel(
                    f"总点云帧：{len(reader.frames)}；"
                    f"抽样帧：{len(self.sample_indices)}（{sampling_method}）"
                ),
                0,
                0,
                1,
                2,
            )
            navigation_layout.addWidget(self.frame_spin, 1, 0)
            navigation_layout.addWidget(go_button, 1, 1)
            navigation_layout.addWidget(previous_button, 2, 0)
            navigation_layout.addWidget(next_button, 2, 1)
            panel_layout.addWidget(navigation)

            preview_group = QtWidgets.QGroupBox("短时点云动态观察")
            preview_layout = QtWidgets.QGridLayout(preview_group)
            self.preview_button = QtWidgets.QPushButton(
                "播放当前帧前后点云 [Space]"
            )
            self.preview_button.clicked.connect(self.toggle_preview)
            self.preview_cancel_button = QtWidgets.QPushButton(
                "取消并返回锚定帧"
            )
            self.preview_cancel_button.setEnabled(False)
            self.preview_cancel_button.clicked.connect(self.stop_preview)
            self.preview_lock_button = QtWidgets.QPushButton(
                "锁定当前预览帧用于标注"
            )
            self.preview_lock_button.setEnabled(False)
            self.preview_lock_button.clicked.connect(
                self.lock_preview_frame
            )
            self.preview_half_frames = QtWidgets.QSpinBox()
            self.preview_half_frames.setRange(2, 50)
            self.preview_half_frames.setValue(3)
            self.preview_fps = QtWidgets.QDoubleSpinBox()
            self.preview_fps.setRange(1.0, 15.0)
            self.preview_fps.setValue(5.0)
            self.preview_fps.setSuffix(" fps")
            preview_layout.addWidget(self.preview_button, 0, 0, 1, 2)
            preview_layout.addWidget(
                self.preview_cancel_button, 1, 0, 1, 2
            )
            preview_layout.addWidget(
                self.preview_lock_button, 2, 0, 1, 2
            )
            preview_layout.addWidget(QtWidgets.QLabel("前后帧数"), 3, 0)
            preview_layout.addWidget(self.preview_half_frames, 3, 1)
            preview_layout.addWidget(QtWidgets.QLabel("播放速度"), 4, 0)
            preview_layout.addWidget(self.preview_fps, 4, 1)
            preview_hint = QtWidgets.QLabel(
                "Space播放/暂停。找到清晰帧后先暂停，再锁定用于标注。"
            )
            preview_hint.setWordWrap(True)
            preview_layout.addWidget(preview_hint, 5, 0, 1, 2)
            panel_layout.addWidget(preview_group)

            roi_group = QtWidgets.QGroupBox("显示ROI（base_link，m）")
            roi_layout = QtWidgets.QGridLayout(roi_group)
            self.roi_edits = {}
            defaults = [
                ("x_min", args.roi[0]),
                ("x_max", args.roi[1]),
                ("y_min", args.roi[2]),
                ("y_max", args.roi[3]),
                ("z_min", args.roi[4]),
                ("z_max", args.roi[5]),
            ]
            for row, (name, value) in enumerate(defaults):
                edit = QtWidgets.QDoubleSpinBox()
                edit.setRange(-10.0, 10.0)
                edit.setDecimals(3)
                edit.setSingleStep(0.05)
                edit.setValue(value)
                self.roi_edits[name] = edit
                roi_layout.addWidget(QtWidgets.QLabel(name), row, 0)
                roi_layout.addWidget(edit, row, 1)
            apply_roi = QtWidgets.QPushButton("应用ROI")
            apply_roi.clicked.connect(self.apply_roi)
            roi_layout.addWidget(apply_roi, len(defaults), 0, 1, 2)
            panel_layout.addWidget(roi_group)

            context_group = QtWidgets.QGroupBox("三维图显示高度（m）")
            context_layout = QtWidgets.QGridLayout(context_group)
            self.context_z_min = QtWidgets.QDoubleSpinBox()
            self.context_z_max = QtWidgets.QDoubleSpinBox()
            for edit in (self.context_z_min, self.context_z_max):
                edit.setRange(-5.0, 5.0)
                edit.setDecimals(2)
                edit.setSingleStep(0.10)
            self.context_z_min.setValue(-0.20)
            self.context_z_max.setValue(2.50)
            context_apply = QtWidgets.QPushButton("更新三维图")
            context_apply.clicked.connect(self.redraw)
            context_layout.addWidget(QtWidgets.QLabel("z_min"), 0, 0)
            context_layout.addWidget(self.context_z_min, 0, 1)
            context_layout.addWidget(QtWidgets.QLabel("z_max"), 1, 0)
            context_layout.addWidget(self.context_z_max, 1, 1)
            context_layout.addWidget(context_apply, 2, 0, 1, 2)
            panel_layout.addWidget(context_group)

            selection_group = QtWidgets.QGroupBox("茎秆中心点选择")
            selection_layout = QtWidgets.QGridLayout(selection_group)
            self.left_button = QtWidgets.QPushButton("左行（y>0）[L]")
            self.right_button = QtWidgets.QPushButton("右行（y<0）[R]")
            self.left_button.setCheckable(True)
            self.right_button.setCheckable(True)
            self.left_button.setChecked(True)
            self.left_button.clicked.connect(
                lambda: self.set_active_side("left")
            )
            self.right_button.clicked.connect(
                lambda: self.set_active_side("right")
            )
            self.snap_checkbox = QtWidgets.QCheckBox(
                f"吸附至{args.snap_radius:.2f} m邻域中值"
            )
            self.snap_checkbox.setChecked(not args.no_snap)
            undo_button = QtWidgets.QPushButton("撤销当前侧一点 [U]")
            clear_button = QtWidgets.QPushButton("清空本帧 [C]")
            undo_button.clicked.connect(self.undo_point)
            clear_button.clicked.connect(self.clear_points)
            selection_layout.addWidget(self.left_button, 0, 0)
            selection_layout.addWidget(self.right_button, 0, 1)
            selection_layout.addWidget(self.snap_checkbox, 1, 0, 1, 2)
            selection_layout.addWidget(undo_button, 2, 0)
            selection_layout.addWidget(clear_button, 2, 1)
            panel_layout.addWidget(selection_group)

            save_group = QtWidgets.QGroupBox("保存")
            save_layout = QtWidgets.QGridLayout(save_group)
            self.invalid_reason = QtWidgets.QComboBox()
            self.invalid_reason.addItems(
                [
                    "左行不可判定",
                    "右行不可判定",
                    "双侧均不可判定",
                    "多行归属歧义",
                    "有效点不足",
                    "其他",
                ]
            )
            valid_button = QtWidgets.QPushButton("保存有效并下一帧 [S]")
            invalid_button = QtWidgets.QPushButton("不可判定并下一帧 [I]")
            valid_button.clicked.connect(self.save_valid)
            invalid_button.clicked.connect(self.save_invalid)
            save_layout.addWidget(QtWidgets.QLabel("不可判定原因"), 0, 0)
            save_layout.addWidget(self.invalid_reason, 0, 1)
            save_layout.addWidget(valid_button, 1, 0, 1, 2)
            save_layout.addWidget(invalid_button, 2, 0, 1, 2)
            panel_layout.addWidget(save_group)

            self.fit_label = QtWidgets.QLabel("尚未形成有效拟合")
            self.fit_label.setWordWrap(True)
            panel_layout.addWidget(self.fit_label)
            self.status_label = QtWidgets.QLabel()
            self.status_label.setWordWrap(True)
            panel_layout.addWidget(self.status_label)
            panel_layout.addStretch(1)

        def _bind_shortcuts(self):
            bindings = {
                "L": lambda: self.set_active_side("left"),
                "R": lambda: self.set_active_side("right"),
                "U": self.undo_point,
                "C": self.clear_points,
                "S": self.save_valid,
                "I": self.save_invalid,
                "N": self.next_sample,
                "P": self.previous_sample,
                "Space": self.toggle_preview,
            }
            for key, callback in bindings.items():
                shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
                shortcut.activated.connect(callback)

        def current_roi(self):
            return {
                name: edit.value() for name, edit in self.roi_edits.items()
            }

        def displayed_points(self):
            if self.preview_active and self.preview_points.size:
                return self.preview_points
            return self.raw_points

        def filtered_points(self, points, use_context_height=False):
            roi = self.current_roi()
            z_min = (
                self.context_z_min.value()
                if use_context_height
                else roi["z_min"]
            )
            z_max = (
                self.context_z_max.value()
                if use_context_height
                else roi["z_max"]
            )
            mask = (
                (points[:, 0] >= roi["x_min"])
                & (points[:, 0] <= roi["x_max"])
                & (points[:, 1] >= roi["y_min"])
                & (points[:, 1] <= roi["y_max"])
                & (points[:, 2] >= z_min)
                & (points[:, 2] <= z_max)
            )
            return points[mask]

        def apply_roi(self):
            self.roi_points = self.filtered_points(self.raw_points)
            self.redraw()

        def load_frame(self, frame_index, anchor_frame_index=None):
            self.stop_preview(redraw=False)
            stored = self.store.get(frame_index)
            if anchor_frame_index is None:
                if stored:
                    anchor_frame_index = int(
                        stored.get(
                            "sampling_anchor_frame_index", frame_index
                        )
                    )
                else:
                    anchor_frame_index = frame_index
                    stored_for_anchor = self.store.find_by_anchor(
                        anchor_frame_index
                    )
                    if stored_for_anchor:
                        stored = stored_for_anchor
                        frame_index = int(stored["frame_index"])
            try:
                self.raw_points, self.frame_metadata = (
                    self.reader.load_frame(frame_index)
                )
            except Exception as error:
                QtWidgets.QMessageBox.critical(
                    self, "点云读取失败", str(error)
                )
                return
            self.current_frame_index = frame_index
            self.annotation_anchor_index = int(anchor_frame_index)
            self.annotation_anchor_timestamp_ns = int(
                self.reader.frames[self.annotation_anchor_index][
                    "bag_timestamp_ns"
                ]
            )
            self.frame_spin.setValue(frame_index)
            if stored is None:
                stored = self.store.get(frame_index)
            if stored:
                self.left_points = [
                    tuple(point) for point in stored["left_points_xy"]
                ]
                self.right_points = [
                    tuple(point) for point in stored["right_points_xy"]
                ]
            else:
                self.left_points = []
                self.right_points = []
            try:
                self.sample_position = self.sample_indices.index(
                    self.annotation_anchor_index
                )
            except ValueError:
                pass
            self.apply_roi()
            state = (
                stored["status"] if stored else "未标注"
            )
            self.status_label.setText(
                f"标注帧 {frame_index}/{len(reader.frames)-1}；"
                f"锚定帧 {self.annotation_anchor_index}；"
                f"帧差 {frame_index-self.annotation_anchor_index:+d}\n"
                f"抽样位置 {self.sample_position+1}/{len(self.sample_indices)}；"
                f"状态：{state}\n"
                f"点云时间：{self.frame_metadata['cloud_stamp_ns']/1e9:.9f} s\n"
                f"输出：{self.store.csv_path}"
            )

        def toggle_preview(self):
            if self.preview_active:
                if self.preview_timer.isActive():
                    self.preview_timer.stop()
                    self.preview_button.setText("继续播放 [Space]")
                    self.status_label.setText(
                        f"预览已暂停在帧 {self.preview_frame_index}；"
                        f"锚定帧 {self.preview_anchor_index}；"
                        f"帧差 "
                        f"{self.preview_frame_index-self.preview_anchor_index:+d}\n"
                        "确认清晰后点击“锁定当前预览帧用于标注”。"
                    )
                else:
                    interval_ms = max(
                        50, int(1000.0 / self.preview_fps.value())
                    )
                    self.preview_timer.start(interval_ms)
                    self.preview_button.setText("暂停播放 [Space]")
                return
            half_frames = self.preview_half_frames.value()
            self.preview_anchor_index = self.annotation_anchor_index
            start = max(0, self.preview_anchor_index - half_frames)
            end = min(
                len(self.reader.frames) - 1,
                self.preview_anchor_index + half_frames,
            )
            self.preview_indices = list(range(start, end + 1))
            self.preview_position = 0
            self.preview_active = True
            self.preview_button.setText("暂停播放 [Space]")
            self.preview_cancel_button.setEnabled(True)
            self.preview_lock_button.setEnabled(True)
            self.view_tabs.setCurrentWidget(self.canvas_3d)
            interval_ms = max(50, int(1000.0 / self.preview_fps.value()))
            self.preview_timer.start(interval_ms)
            self.advance_preview()

        def advance_preview(self):
            if not self.preview_active or not self.preview_indices:
                return
            frame_index = self.preview_indices[self.preview_position]
            try:
                self.preview_points, _ = self.reader.load_frame(frame_index)
            except Exception as error:
                self.stop_preview()
                QtWidgets.QMessageBox.warning(
                    self, "短时点云播放失败", str(error)
                )
                return
            self.preview_frame_index = frame_index
            self.preview_position = (
                self.preview_position + 1
            ) % len(self.preview_indices)
            self.status_label.setText(
                f"动态预览帧：{frame_index}；"
                f"标注锚定帧：{self.preview_anchor_index}\n"
                "Space暂停；确认后锁定当前预览帧用于标注。"
            )
            self.redraw()

        def stop_preview(self, redraw=True):
            self.preview_timer.stop()
            was_active = self.preview_active
            self.preview_active = False
            self.preview_points = np.empty((0, 3), dtype=np.float32)
            if hasattr(self, "preview_button"):
                self.preview_button.setText(
                    "播放当前帧前后点云 [Space]"
                )
                self.preview_cancel_button.setEnabled(False)
                self.preview_lock_button.setEnabled(False)
            if was_active and redraw:
                self.load_frame(
                    self.preview_anchor_index,
                    anchor_frame_index=self.preview_anchor_index,
                )

        def lock_preview_frame(self):
            if not self.preview_active:
                return
            selected_frame = self.preview_frame_index
            anchor_frame = self.preview_anchor_index
            self.preview_timer.stop()
            self.preview_active = False
            self.preview_points = np.empty((0, 3), dtype=np.float32)
            self.preview_button.setText(
                "播放当前帧前后点云 [Space]"
            )
            self.preview_cancel_button.setEnabled(False)
            self.preview_lock_button.setEnabled(False)
            self.load_frame(
                selected_frame, anchor_frame_index=anchor_frame
            )
            self.view_tabs.setCurrentWidget(self.canvas)
            self.statusBar().showMessage(
                f"已锁定帧{selected_frame}用于标注；"
                f"原锚定帧{anchor_frame}，"
                f"帧差{selected_frame-anchor_frame:+d}",
                6000,
            )

        def previous_sample(self):
            self.sample_position = max(0, self.sample_position - 1)
            self.load_frame(self.sample_indices[self.sample_position])

        def next_sample(self):
            self.sample_position = min(
                len(self.sample_indices) - 1, self.sample_position + 1
            )
            self.load_frame(self.sample_indices[self.sample_position])

        def set_active_side(self, side):
            self.active_side = side
            self.left_button.setChecked(side == "left")
            self.right_button.setChecked(side == "right")

        def on_plot_click(self, event):
            if self.preview_active:
                self.statusBar().showMessage(
                    "预览中不能标注；请暂停并锁定当前帧，"
                    "或取消返回锚定帧",
                    3000,
                )
                return
            if (
                event.inaxes != self.axes
                or event.xdata is None
                or event.ydata is None
                or event.button != 1
            ):
                return
            point = np.array([event.xdata, event.ydata], dtype=float)
            if self.snap_checkbox.isChecked() and self.roi_points.size:
                distances = np.linalg.norm(
                    self.roi_points[:, :2] - point, axis=1
                )
                neighbors = self.roi_points[
                    distances <= args.snap_radius, :2
                ]
                if neighbors.size:
                    point = np.median(neighbors, axis=0)
            target = (
                self.left_points
                if self.active_side == "left"
                else self.right_points
            )
            target.append((float(point[0]), float(point[1])))
            self.redraw()

        def undo_point(self):
            target = (
                self.left_points
                if self.active_side == "left"
                else self.right_points
            )
            if target:
                target.pop()
                self.redraw()

        def clear_points(self):
            self.left_points = []
            self.right_points = []
            self.redraw()

        def redraw(self):
            displayed = self.displayed_points()
            display_roi_points = self.filtered_points(displayed)
            context_points = self.filtered_points(
                displayed, use_context_height=True
            )
            shown_frame = (
                self.preview_frame_index
                if self.preview_active
                else self.current_frame_index
            )
            intensity_source = (
                context_points[:, 3]
                if context_points.ndim == 2
                and context_points.shape[1] > 3
                and context_points.size
                else np.array([0.0, 1.0])
            )
            finite_intensity = intensity_source[
                np.isfinite(intensity_source)
            ]
            if finite_intensity.size:
                intensity_min, intensity_max = np.percentile(
                    finite_intensity, (1.0, 99.0)
                )
            else:
                intensity_min, intensity_max = 0.0, 1.0
            if intensity_max <= intensity_min + 1.0e-6:
                intensity_max = intensity_min + 1.0
            self.axes.clear()
            self.axes.set_facecolor("#303030")
            self.figure.patch.set_facecolor("#303030")
            if display_roi_points.size:
                plot_points = display_roi_points
                if plot_points.shape[0] > args.max_plot_points:
                    step = int(
                        math.ceil(
                            plot_points.shape[0] / args.max_plot_points
                        )
                    )
                    plot_points = plot_points[::step]
                scatter = self.axes.scatter(
                    plot_points[:, 0],
                    plot_points[:, 1],
                    c=plot_points[:, 3],
                    s=9,
                    alpha=1.0,
                    cmap="turbo",
                    marker="s",
                    linewidths=0,
                )
                scatter.set_clim(
                    intensity_min, intensity_max
                )
            if self.left_points and not self.preview_active:
                points = np.asarray(self.left_points)
                self.axes.scatter(
                    points[:, 0],
                    points[:, 1],
                    c="#e74c3c",
                    s=70,
                    marker="x",
                    linewidths=2.5,
                    label=f"左行人工点 ({len(points)})",
                )
            if self.right_points and not self.preview_active:
                points = np.asarray(self.right_points)
                self.axes.scatter(
                    points[:, 0],
                    points[:, 1],
                    c="#3498db",
                    s=70,
                    marker="x",
                    linewidths=2.5,
                    label=f"右行人工点 ({len(points)})",
                )
            try:
                if self.preview_active:
                    raise ValueError("preview")
                fit = fit_parallel_rows(self.left_points, self.right_points)
                x_values = np.array(
                    [
                        self.current_roi()["x_min"],
                        self.current_roi()["x_max"],
                    ]
                )
                for intercept, color, label in [
                    (
                        fit["left_intercept_m"],
                        "#e74c3c",
                        "左拟合行",
                    ),
                    (
                        fit["right_intercept_m"],
                        "#3498db",
                        "右拟合行",
                    ),
                    (
                        fit["center_intercept_m"],
                        "#f1c40f",
                        "真值中心线",
                    ),
                ]:
                    self.axes.plot(
                        x_values,
                        fit["slope"] * x_values + intercept,
                        color=color,
                        linewidth=2.2,
                        label=label,
                    )
                self.fit_label.setText(
                    f"中心横向偏移：{fit['center_intercept_m']:+.3f} m\n"
                    f"真实行航向：{fit['heading_deg']:+.2f}°\n"
                    f"行距：{fit['row_width_m']:.3f} m；"
                    f"人工点拟合RMSE：{fit['fit_rmse_m']:.3f} m"
                )
            except ValueError:
                if self.preview_active:
                    self.fit_label.setText(
                        "正在短时动态观察；停止后恢复人工点和拟合线"
                    )
                else:
                    self.fit_label.setText(
                        f"左侧{len(self.left_points)}点，"
                        f"右侧{len(self.right_points)}点；"
                        "左右至少各2点后显示中心线"
                    )
            roi = self.current_roi()
            self.axes.set_xlim(roi["x_min"], roi["x_max"])
            self.axes.set_ylim(roi["y_min"], roi["y_max"])
            self.axes.set_xlabel("车体前向 x（m）")
            self.axes.set_ylabel("车体横向 y（m，左正右负）")
            self.axes.xaxis.label.set_color("white")
            self.axes.yaxis.label.set_color("white")
            self.axes.tick_params(colors="white")
            for spine in self.axes.spines.values():
                spine.set_color("#aaaaaa")
            self.axes.set_title(
                f"{reader.bag_name} — 点云帧 {shown_frame}"
                + ("（动态预览）" if self.preview_active else "")
                + f" · ROI {display_roi_points.shape[0]}点（Intensity）",
                color="white",
            )
            self.axes.grid(color="#777777", alpha=0.35)
            handles, labels = self.axes.get_legend_handles_labels()
            if handles:
                legend = self.axes.legend(loc="upper right")
                legend.get_frame().set_facecolor("#303030")
                for text in legend.get_texts():
                    text.set_color("white")
            self.canvas.draw_idle()

            elevation = getattr(self.axes_3d, "elev", 22.0)
            azimuth = getattr(self.axes_3d, "azim", -65.0)
            self.axes_3d.clear()
            self.axes_3d.set_facecolor("#303030")
            self.figure_3d.patch.set_facecolor("#303030")
            if context_points.size:
                plot_3d = context_points
                if plot_3d.shape[0] > args.max_3d_points:
                    step = int(
                        math.ceil(
                            plot_3d.shape[0] / args.max_3d_points
                        )
                    )
                    plot_3d = plot_3d[::step]
                self.axes_3d.scatter(
                    plot_3d[:, 0],
                    plot_3d[:, 1],
                    plot_3d[:, 2],
                    c=plot_3d[:, 3],
                    s=5,
                    alpha=0.90,
                    cmap="turbo",
                    marker="s",
                    linewidths=0,
                ).set_clim(intensity_min, intensity_max)
            if display_roi_points.size:
                highlighted = display_roi_points
                if highlighted.shape[0] > args.max_3d_points:
                    step = int(
                        math.ceil(
                            highlighted.shape[0] / args.max_3d_points
                        )
                    )
                    highlighted = highlighted[::step]
                highlighted_scatter = self.axes_3d.scatter(
                    highlighted[:, 0],
                    highlighted[:, 1],
                    highlighted[:, 2],
                    c=highlighted[:, 3],
                    s=11,
                    alpha=1.0,
                    cmap="turbo",
                    marker="s",
                    linewidths=0,
                )
                highlighted_scatter.set_clim(
                    intensity_min, intensity_max
                )
            if not self.preview_active:
                if self.left_points:
                    points = np.asarray(self.left_points)
                    self.axes_3d.scatter(
                        points[:, 0],
                        points[:, 1],
                        np.full(points.shape[0], roi["z_min"]),
                        c="#e74c3c",
                        s=65,
                        marker="x",
                        linewidths=2.5,
                    )
                if self.right_points:
                    points = np.asarray(self.right_points)
                    self.axes_3d.scatter(
                        points[:, 0],
                        points[:, 1],
                        np.full(points.shape[0], roi["z_min"]),
                        c="#3498db",
                        s=65,
                        marker="x",
                        linewidths=2.5,
                    )
            self.axes_3d.set_xlim(roi["x_min"], roi["x_max"])
            self.axes_3d.set_ylim(roi["y_min"], roi["y_max"])
            self.axes_3d.set_zlim(
                self.context_z_min.value(), self.context_z_max.value()
            )
            self.axes_3d.set_xlabel("前向 x / m")
            self.axes_3d.set_ylabel("横向 y / m")
            self.axes_3d.set_zlabel("高度 z / m")
            self.axes_3d.xaxis.label.set_color("white")
            self.axes_3d.yaxis.label.set_color("white")
            self.axes_3d.zaxis.label.set_color("white")
            self.axes_3d.tick_params(colors="white")
            for axis in (
                self.axes_3d.xaxis,
                self.axes_3d.yaxis,
                self.axes_3d.zaxis,
            ):
                axis.pane.set_facecolor((0.12, 0.12, 0.12, 1.0))
                axis.pane.set_edgecolor((0.55, 0.55, 0.55, 1.0))
                axis._axinfo["grid"]["color"] = (
                    0.45,
                    0.45,
                    0.45,
                    0.55,
                )
            self.axes_3d.set_title(
                f"三维点云帧 {shown_frame}"
                + ("（短时动态）" if self.preview_active else "")
                + f" · 近场 {context_points.shape[0]}点"
                + (
                    "（完整）"
                    if context_points.shape[0] <= args.max_3d_points
                    else "（显示抽稀）"
                ),
                color="white",
            )
            self.axes_3d.view_init(elev=elevation, azim=azimuth)
            try:
                self.axes_3d.set_box_aspect(
                    (
                        max(0.1, roi["x_max"] - roi["x_min"]),
                        max(0.1, roi["y_max"] - roi["y_min"]),
                        max(
                            0.1,
                            self.context_z_max.value()
                            - self.context_z_min.value(),
                        ),
                    )
                )
            except AttributeError:
                pass
            self.canvas_3d.draw_idle()

        def save_valid(self):
            if self.preview_active:
                self.statusBar().showMessage(
                    "请先按Space停止动态预览，再保存当前标注帧", 3000
                )
                return
            try:
                self.store.save(
                    self.frame_metadata,
                    self.current_roi(),
                    self.left_points,
                    self.right_points,
                    "valid",
                    sampling_anchor_frame_index=(
                        self.annotation_anchor_index
                    ),
                    sampling_anchor_timestamp_ns=(
                        self.annotation_anchor_timestamp_ns
                    ),
                )
            except ValueError as error:
                QtWidgets.QMessageBox.warning(self, "无法保存有效帧", str(error))
                return
            if len(self.left_points) < 3 or len(self.right_points) < 3:
                self.statusBar().showMessage(
                    "已保存，但建议左右各标3个以上茎秆中心", 5000
                )
            self.next_sample()

        def save_invalid(self):
            if self.preview_active:
                self.statusBar().showMessage(
                    "请先按Space停止动态预览，再保存当前标注帧", 3000
                )
                return
            self.store.save(
                self.frame_metadata,
                self.current_roi(),
                self.left_points,
                self.right_points,
                "invalid",
                self.invalid_reason.currentText(),
                sampling_anchor_frame_index=self.annotation_anchor_index,
                sampling_anchor_timestamp_ns=(
                    self.annotation_anchor_timestamp_ns
                ),
            )
            self.next_sample()

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("CornRowPointCloudAnnotator")
    window = AnnotationWindow()
    window.showMaximized()
    return app.exec_()


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="直接从rosbag快速标注左右玉米行和真实中心线"
    )
    parser.add_argument(
        "--bag",
        required=True,
        help="rosbag目录、.db3或.db3.zstd文件",
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument(
        "--output",
        default="",
        help="输出目录；默认保存到工作空间field_ground_truth内",
    )
    parser.add_argument(
        "--cache-dir",
        default=str(DEFAULT_CACHE_ROOT),
        help="压缩包解压缓存目录",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=2.0,
        help="无有效运动里程时的时间抽样间隔，单位s",
    )
    parser.add_argument(
        "--sample-distance",
        type=float,
        default=0.5,
        help="运动包按里程抽样的间隔，单位m；设为0则按时间抽样",
    )
    parser.add_argument(
        "--odom-topic",
        default="/odometry/filtered",
        help="仅用于筛选运动帧和等距离抽样，不参与真值计算",
    )
    parser.add_argument(
        "--moving-speed-threshold",
        type=float,
        default=0.03,
        help="判定车辆正在运动的里程计速度阈值，单位m/s",
    )
    parser.add_argument(
        "--roi",
        type=float,
        nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        default=(0.10, 2.20, -1.40, 1.40, 0.10, 0.50),
    )
    parser.add_argument(
        "--snap-radius",
        type=float,
        default=0.04,
        help="点击后局部点云中值吸附半径，单位m",
    )
    parser.add_argument("--no-snap", action="store_true")
    parser.add_argument("--annotator", default=os.environ.get("USER", ""))
    parser.add_argument("--max-plot-points", type=int, default=50000)
    parser.add_argument(
        "--max-3d-points",
        type=int,
        default=50000,
        help="三维视图最大绘制点数，只影响显示流畅度",
    )
    parser.add_argument(
        "--manual-translation",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="仅在包内没有TF时指定 target_frame<-sensor 平移",
    )
    parser.add_argument(
        "--manual-rpy-deg",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0),
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="只检查点云数量、TF和ROI，不启动界面",
    )
    parser.add_argument("--frame-index", type=int, default=0)
    return parser.parse_args()


def main():
    args = parse_arguments()
    manual_transform = None
    if args.manual_translation is not None:
        manual_transform = rpy_transform(
            args.manual_translation, args.manual_rpy_deg
        )
    reader = RosbagPointCloudReader(
        args.bag,
        topic=args.topic,
        target_frame=args.target_frame,
        cache_root=args.cache_dir,
        manual_transform=manual_transform,
    )
    if args.inspect:
        frame_index = min(max(0, args.frame_index), len(reader.frames) - 1)
        points, metadata = reader.load_frame(frame_index)
        roi = args.roi
        mask = (
            (points[:, 0] >= roi[0])
            & (points[:, 0] <= roi[1])
            & (points[:, 1] >= roi[2])
            & (points[:, 1] <= roi[3])
            & (points[:, 2] >= roi[4])
            & (points[:, 2] <= roi[5])
        )
        print(f"bag: {reader.source_path}")
        print(f"pointcloud_frames: {len(reader.frames)}")
        print(f"frame_index: {frame_index}")
        print(f"cloud_frame: {metadata['cloud_frame']}")
        print(f"target_frame: {metadata['target_frame']}")
        print(f"transform_source: {metadata['transform_source']}")
        print(f"raw_points: {points.shape[0]}")
        print(f"roi_points: {int(mask.sum())}")
        return 0
    output = (
        Path(args.output)
        if args.output
        else DEFAULT_OUTPUT_ROOT / reader.bag_name
    )
    sample_indices = []
    sampling_method = "time"
    if args.sample_distance > 0.0:
        sample_indices, sampling_method = reader.distance_sample_indices(
            args.sample_distance,
            args.odom_topic,
            args.moving_speed_threshold,
        )
    if not sample_indices:
        time_indices = sampled_frame_indices(
            [item["bag_timestamp_ns"] for item in reader.frames],
            args.sample_period,
        )
        # A long static bag represents one physical position.  Start at its
        # middle frame; arbitrary frames remain reachable by direct jump.
        if sampling_method == "distance" and len(time_indices) > 3:
            sample_indices = [time_indices[len(time_indices) // 2]]
            sampling_method = "static-middle"
        else:
            sample_indices = time_indices
            sampling_method = "time"
    store = AnnotationStore(
        output, reader, args.annotator, args.sample_period
    )
    return launch_gui(
        args, reader, store, sample_indices, sampling_method
    )


if __name__ == "__main__":
    raise SystemExit(main())
