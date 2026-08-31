#!/usr/bin/env python3
"""Use RViz PointCloud2 rendering and Publish Point for row annotation."""

import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys

import numpy as np


def load_annotation_core():
    module_path = Path(__file__).resolve().parent / (
        "pointcloud_centerline_annotator.py"
    )
    spec = importlib.util.spec_from_file_location(
        "pointcloud_annotation_core", module_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


core = load_annotation_core()


def parse_arguments():
    from ament_index_python.packages import get_package_share_directory

    default_rviz_config = (
        Path(get_package_share_directory("centerline_extraction"))
        / "config"
        / "rviz_pointcloud_annotation.rviz"
    )
    parser = argparse.ArgumentParser(
        description="使用RViz原生点云显示进行玉米行真值标注"
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--sample-distance", type=float, default=0.5)
    parser.add_argument("--sample-period", type=float, default=2.0)
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument("--moving-speed-threshold", type=float, default=0.03)
    parser.add_argument("--annotator", default=os.environ.get("USER", ""))
    parser.add_argument("--output", default="")
    parser.add_argument(
        "--cache-dir", default=str(core.DEFAULT_CACHE_ROOT)
    )
    parser.add_argument(
        "--rviz-config",
        default=str(default_rviz_config),
    )
    return parser.parse_args()


def make_sample_indices(reader, args):
    indices = []
    method = "time"
    if args.sample_distance > 0.0:
        indices, method = reader.distance_sample_indices(
            args.sample_distance,
            args.odom_topic,
            args.moving_speed_threshold,
        )
    if indices:
        return indices, method
    time_indices = core.sampled_frame_indices(
        [item["bag_timestamp_ns"] for item in reader.frames],
        args.sample_period,
    )
    if method == "distance" and len(time_indices) > 3:
        return [time_indices[len(time_indices) // 2]], "static-middle"
    return time_indices, "time"


def make_cloud_message(points, frame_id, stamp):
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    fields = [
        PointField(
            name=name,
            offset=offset,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for name, offset in (
            ("x", 0),
            ("y", 4),
            ("z", 8),
            ("intensity", 12),
        )
    ]
    return point_cloud2.create_cloud(
        header, fields, np.asarray(points[:, :4], dtype=np.float32)
    )


class RvizAnnotationNode:
    def __init__(self, rclpy, frame_id):
        from geometry_msgs.msg import PointStamped
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import TransformBroadcaster
        from visualization_msgs.msg import MarkerArray

        class AnnotationNode(Node):
            pass

        self.node = AnnotationNode("rviz_corn_row_annotation")
        self.frame_id = frame_id
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2, "/annotation/cloud", qos
        )
        self.marker_publisher = self.node.create_publisher(
            MarkerArray, "/annotation/markers", qos
        )
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.clicked_subscription = self.node.create_subscription(
            PointStamped,
            "/clicked_point",
            self._clicked_callback,
            10,
        )
        self.points = np.empty((0, 4), dtype=np.float32)
        self.current_cloud = None
        self.odometry_transform = None
        self.parent_from_child = None
        self.click_handler = None
        self.rclpy = rclpy

    def set_click_handler(self, handler):
        self.click_handler = handler

    def set_cloud(self, points):
        self.points = points
        self.current_cloud = make_cloud_message(
            points,
            self.frame_id,
            self.node.get_clock().now().to_msg(),
        )
        self.cloud_publisher.publish(self.current_cloud)

    def set_odometry_transform(self, odometry):
        from geometry_msgs.msg import TransformStamped

        transform = TransformStamped()
        transform.header.frame_id = odometry["parent_frame"]
        transform.child_frame_id = odometry["child_frame"]
        translation = odometry["translation"]
        quaternion = odometry["quaternion"]
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.odometry_transform = transform
        self.parent_from_child = core.transform_matrix(
            translation, quaternion
        )
        self.publish_transform()

    def publish_transform(self):
        if self.odometry_transform is None:
            return
        self.odometry_transform.header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        self.tf_broadcaster.sendTransform(self.odometry_transform)

    def republish_cloud(self):
        if self.current_cloud is not None:
            self.current_cloud.header.stamp = (
                self.node.get_clock().now().to_msg()
            )
            self.cloud_publisher.publish(self.current_cloud)
        self.publish_transform()

    def _clicked_callback(self, message):
        clicked_frame = core.normalize_frame(message.header.frame_id)
        target_frame = core.normalize_frame(self.frame_id)
        clicked_point = np.array(
            [message.point.x, message.point.y, message.point.z, 1.0],
            dtype=float,
        )
        if clicked_frame == target_frame:
            point = clicked_point[:3]
        elif (
            self.odometry_transform is not None
            and self.parent_from_child is not None
            and clicked_frame
            == core.normalize_frame(
                self.odometry_transform.header.frame_id
            )
        ):
            point = (
                np.linalg.inv(self.parent_from_child) @ clicked_point
            )[:3]
        else:
            self.node.get_logger().warning(
                f"忽略来自{message.header.frame_id}的点击，"
                f"只接受{self.frame_id}或当前odom坐标"
            )
            return
        if self.click_handler is not None:
            self.click_handler(
                float(point[0]),
                float(point[1]),
                float(point[2]),
            )

    def publish_markers(
        self, left_points, right_points, frame_index, anchor_index
    ):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray

        stamp = self.node.get_clock().now().to_msg()
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        marker_id = 0
        for side_points, color in (
            (left_points, (1.0, 0.1, 0.1)),
            (right_points, (0.1, 0.45, 1.0)),
        ):
            for x_value, y_value, z_value in side_points:
                marker = Marker()
                marker.header.frame_id = self.frame_id
                marker.header.stamp = stamp
                marker.ns = "manual_stalk_points"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x_value
                marker.pose.position.y = y_value
                marker.pose.position.z = z_value
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.085
                marker.scale.y = 0.085
                marker.scale.z = 0.085
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                markers.markers.append(marker)

        try:
            fit = core.fit_parallel_rows(
                [(point[0], point[1]) for point in left_points],
                [(point[0], point[1]) for point in right_points],
            )
        except ValueError:
            fit = None
        if fit is not None:
            for intercept, color, namespace in (
                (
                    fit["left_intercept_m"],
                    (1.0, 0.1, 0.1),
                    "left_truth",
                ),
                (
                    fit["right_intercept_m"],
                    (0.1, 0.45, 1.0),
                    "right_truth",
                ),
                (
                    fit["center_intercept_m"],
                    (1.0, 0.9, 0.0),
                    "center_truth",
                ),
            ):
                line = Marker()
                line.header.frame_id = self.frame_id
                line.header.stamp = stamp
                line.ns = namespace
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.025
                line.color.r, line.color.g, line.color.b = color
                line.color.a = 1.0
                for x_value in (0.10, 2.20):
                    point = Point()
                    point.x = x_value
                    point.y = fit["slope"] * x_value + intercept
                    point.z = 0.25
                    line.points.append(point)
                markers.markers.append(line)

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = stamp
        text.ns = "annotation_state"
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.35
        text.pose.position.y = 0.0
        text.pose.position.z = 1.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = (
            f"标注帧 {frame_index}  锚定帧 {anchor_index}  "
            f"左{len(left_points)} 右{len(right_points)}"
        )
        markers.markers.append(text)
        self.marker_publisher.publish(markers)

    def destroy(self):
        self.node.destroy_node()


def launch_controller(args, reader, store, sample_indices, sampling_method):
    import rclpy
    from PyQt5 import QtCore, QtWidgets

    rclpy.init(args=None)
    ros = RvizAnnotationNode(rclpy, reader.target_frame)
    rviz_process = None
    rviz_config = Path(args.rviz_config)
    if rviz_config.exists():
        rviz_process = subprocess.Popen(
            ["rviz2", "-d", str(rviz_config)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("RvizCornRowAnnotationController")

    class Controller(QtWidgets.QWidget):
        def __init__(self):
            super().__init__(
                flags=QtCore.Qt.Window
                | QtCore.Qt.WindowStaysOnTopHint
            )
            self.setWindowTitle("RViz玉米行标注控制器")
            self.setMinimumWidth(310)
            self.sample_position = 0
            self.current_frame_index = sample_indices[0]
            self.anchor_frame_index = self.current_frame_index
            self.frame_metadata = {}
            self.odometry = {}
            self.points = np.empty((0, 4), dtype=np.float32)
            self.left_points = []
            self.right_points = []
            self.click_stack = []
            self.preview_indices = []
            self.preview_position = 0
            self.preview_active = False
            self.preview_timer = QtCore.QTimer(self)
            self.preview_timer.timeout.connect(self.advance_preview)
            self._build_ui()
            ros.set_click_handler(self.add_clicked_point)
            self.load_frame(self.current_frame_index)

        def _build_ui(self):
            layout = QtWidgets.QVBoxLayout(self)
            instructions = QtWidgets.QLabel(
                "在RViz工具栏选择一次“Publish Point”，连续点击左右茎秆。"
                "点击点按y坐标自动分为左红、右蓝；标记和拟合线会立即"
                "显示在RViz中。"
            )
            instructions.setWordWrap(True)
            layout.addWidget(instructions)

            self.frame_label = QtWidgets.QLabel()
            self.frame_label.setWordWrap(True)
            layout.addWidget(self.frame_label)

            navigation = QtWidgets.QGridLayout()
            previous_button = QtWidgets.QPushButton("上一个抽样帧")
            next_button = QtWidgets.QPushButton("下一个抽样帧")
            previous_raw = QtWidgets.QPushButton("上一原始帧")
            next_raw = QtWidgets.QPushButton("下一原始帧")
            previous_button.clicked.connect(self.previous_sample)
            next_button.clicked.connect(self.next_sample)
            previous_raw.clicked.connect(
                lambda: self.load_frame(
                    max(0, self.current_frame_index - 1),
                    self.anchor_frame_index,
                )
            )
            next_raw.clicked.connect(
                lambda: self.load_frame(
                    min(
                        len(reader.frames) - 1,
                        self.current_frame_index + 1,
                    ),
                    self.anchor_frame_index,
                )
            )
            navigation.addWidget(previous_button, 0, 0)
            navigation.addWidget(next_button, 0, 1)
            navigation.addWidget(previous_raw, 1, 0)
            navigation.addWidget(next_raw, 1, 1)
            layout.addLayout(navigation)

            preview_layout = QtWidgets.QGridLayout()
            self.play_button = QtWidgets.QPushButton("播放锚定帧前后3帧")
            self.lock_button = QtWidgets.QPushButton("锁定当前帧")
            self.cancel_button = QtWidgets.QPushButton("返回锚定帧")
            self.lock_button.setEnabled(False)
            self.cancel_button.setEnabled(False)
            self.play_button.clicked.connect(self.toggle_preview)
            self.lock_button.clicked.connect(self.lock_preview)
            self.cancel_button.clicked.connect(self.cancel_preview)
            preview_layout.addWidget(self.play_button, 0, 0, 1, 2)
            preview_layout.addWidget(self.lock_button, 1, 0)
            preview_layout.addWidget(self.cancel_button, 1, 1)
            layout.addLayout(preview_layout)

            self.fit_label = QtWidgets.QLabel()
            self.fit_label.setWordWrap(True)
            layout.addWidget(self.fit_label)

            edit_layout = QtWidgets.QGridLayout()
            undo_button = QtWidgets.QPushButton("撤销最后一点")
            clear_button = QtWidgets.QPushButton("清空本帧")
            undo_button.clicked.connect(self.undo)
            clear_button.clicked.connect(self.clear)
            edit_layout.addWidget(undo_button, 0, 0)
            edit_layout.addWidget(clear_button, 0, 1)
            layout.addLayout(edit_layout)

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
            layout.addWidget(self.invalid_reason)
            save_button = QtWidgets.QPushButton("保存有效并进入下一帧")
            invalid_button = QtWidgets.QPushButton(
                "保存为不可判定并进入下一帧"
            )
            save_button.clicked.connect(self.save_valid)
            invalid_button.clicked.connect(self.save_invalid)
            layout.addWidget(save_button)
            layout.addWidget(invalid_button)
            layout.addStretch(1)

        def load_frame(self, frame_index, anchor_index=None):
            if anchor_index is None:
                anchor_index = frame_index
            self.preview_timer.stop()
            self.preview_active = False
            self.play_button.setText("播放锚定帧前后3帧")
            self.lock_button.setEnabled(False)
            self.cancel_button.setEnabled(False)
            self.points, self.frame_metadata = reader.load_frame(frame_index)
            self.odometry = reader.load_nearest_odometry(
                frame_index, args.odom_topic
            )
            self.current_frame_index = frame_index
            self.anchor_frame_index = anchor_index
            stored = store.get(frame_index)
            self.left_points = []
            self.right_points = []
            self.click_stack = []
            if stored:
                self.left_points = [
                    (point[0], point[1], 0.25)
                    for point in stored["left_points_xy"]
                ]
                self.right_points = [
                    (point[0], point[1], 0.25)
                    for point in stored["right_points_xy"]
                ]
                self.click_stack = [
                    ("left", point) for point in self.left_points
                ] + [("right", point) for point in self.right_points]
            try:
                self.sample_position = sample_indices.index(anchor_index)
            except ValueError:
                pass
            ros.set_odometry_transform(self.odometry)
            ros.set_cloud(self.points)
            self.update_markers_and_labels()

        def update_markers_and_labels(self):
            ros.publish_markers(
                self.left_points,
                self.right_points,
                self.current_frame_index,
                self.anchor_frame_index,
            )
            self.frame_label.setText(
                f"抽样 {self.sample_position+1}/{len(sample_indices)}"
                f"（{sampling_method}）\n"
                f"锚定帧：{self.anchor_frame_index}；"
                f"当前帧：{self.current_frame_index}；"
                f"帧差：{self.current_frame_index-self.anchor_frame_index:+d}\n"
                f"原始点：{self.points.shape[0]}，完整发布，无抽稀\n"
                f"TF {self.odometry.get('parent_frame', 'odom')}→"
                f"{self.odometry.get('child_frame', reader.target_frame)}："
                f"x={self.odometry.get('translation', [0, 0])[0]:+.3f} m，"
                f"y={self.odometry.get('translation', [0, 0])[1]:+.3f} m，"
                f"匹配时间差={self.odometry.get('time_delta_s', 0):+.3f} s"
            )
            try:
                fit = core.fit_parallel_rows(
                    [(point[0], point[1]) for point in self.left_points],
                    [(point[0], point[1]) for point in self.right_points],
                )
                self.fit_label.setText(
                    f"左侧{len(self.left_points)}点，"
                    f"右侧{len(self.right_points)}点\n"
                    f"中心偏移{fit['center_intercept_m']:+.3f} m，"
                    f"航向{fit['heading_deg']:+.2f}°，"
                    f"行距{fit['row_width_m']:.3f} m"
                )
            except ValueError:
                self.fit_label.setText(
                    f"左侧{len(self.left_points)}点，"
                    f"右侧{len(self.right_points)}点；"
                    "左右至少各2点后拟合"
                )

        def add_clicked_point(self, x_value, y_value, z_value):
            if self.preview_active:
                return
            point = (x_value, y_value, z_value)
            side = "left" if y_value >= 0.0 else "right"
            target = (
                self.left_points if side == "left" else self.right_points
            )
            target.append(point)
            self.click_stack.append((side, point))
            self.update_markers_and_labels()

        def undo(self):
            if not self.click_stack:
                return
            side, _ = self.click_stack.pop()
            target = (
                self.left_points if side == "left" else self.right_points
            )
            if target:
                target.pop()
            self.update_markers_and_labels()

        def clear(self):
            self.left_points = []
            self.right_points = []
            self.click_stack = []
            self.update_markers_and_labels()

        def previous_sample(self):
            self.sample_position = max(0, self.sample_position - 1)
            self.load_frame(sample_indices[self.sample_position])

        def next_sample(self):
            self.sample_position = min(
                len(sample_indices) - 1, self.sample_position + 1
            )
            self.load_frame(sample_indices[self.sample_position])

        def toggle_preview(self):
            if self.preview_active:
                if self.preview_timer.isActive():
                    self.preview_timer.stop()
                    self.play_button.setText("继续播放")
                else:
                    self.preview_timer.start(200)
                    self.play_button.setText("暂停播放")
                return
            start = max(0, self.anchor_frame_index - 3)
            end = min(
                len(reader.frames) - 1, self.anchor_frame_index + 3
            )
            self.preview_indices = list(range(start, end + 1))
            self.preview_position = 0
            self.preview_active = True
            self.lock_button.setEnabled(True)
            self.cancel_button.setEnabled(True)
            self.play_button.setText("暂停播放")
            self.preview_timer.start(200)
            self.advance_preview()

        def advance_preview(self):
            frame_index = self.preview_indices[self.preview_position]
            self.points, self.frame_metadata = reader.load_frame(frame_index)
            self.odometry = reader.load_nearest_odometry(
                frame_index, args.odom_topic
            )
            self.current_frame_index = frame_index
            ros.set_odometry_transform(self.odometry)
            ros.set_cloud(self.points)
            self.preview_position = (
                self.preview_position + 1
            ) % len(self.preview_indices)
            self.update_markers_and_labels()

        def lock_preview(self):
            selected = self.current_frame_index
            anchor = self.anchor_frame_index
            self.load_frame(selected, anchor)

        def cancel_preview(self):
            self.load_frame(
                self.anchor_frame_index, self.anchor_frame_index
            )

        def save_valid(self):
            try:
                store.save(
                    self.frame_metadata,
                    {
                        "x_min": 0.10,
                        "x_max": 2.20,
                        "y_min": -1.40,
                        "y_max": 1.40,
                        "z_min": 0.10,
                        "z_max": 0.50,
                    },
                    [(p[0], p[1]) for p in self.left_points],
                    [(p[0], p[1]) for p in self.right_points],
                    "valid",
                    sampling_anchor_frame_index=self.anchor_frame_index,
                    sampling_anchor_timestamp_ns=reader.frames[
                        self.anchor_frame_index
                    ]["bag_timestamp_ns"],
                )
            except ValueError as error:
                QtWidgets.QMessageBox.warning(
                    self, "无法保存有效帧", str(error)
                )
                return
            self.next_sample()

        def save_invalid(self):
            store.save(
                self.frame_metadata,
                {
                    "x_min": 0.10,
                    "x_max": 2.20,
                    "y_min": -1.40,
                    "y_max": 1.40,
                    "z_min": 0.10,
                    "z_max": 0.50,
                },
                [(p[0], p[1]) for p in self.left_points],
                [(p[0], p[1]) for p in self.right_points],
                "invalid",
                self.invalid_reason.currentText(),
                sampling_anchor_frame_index=self.anchor_frame_index,
                sampling_anchor_timestamp_ns=reader.frames[
                    self.anchor_frame_index
                ]["bag_timestamp_ns"],
            )
            self.next_sample()

    controller = Controller()
    screen = app.primaryScreen().availableGeometry()
    controller.resize(330, min(560, screen.height()))
    controller.move(
        max(0, screen.x() + screen.width() - controller.width()),
        screen.y(),
    )
    controller.show()

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(
        lambda: rclpy.spin_once(ros.node, timeout_sec=0.0)
    )
    spin_timer.start(10)
    republish_timer = QtCore.QTimer()
    republish_timer.timeout.connect(ros.republish_cloud)
    republish_timer.start(1000)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    result = app.exec_()
    if rviz_process is not None and rviz_process.poll() is None:
        rviz_process.terminate()
    ros.destroy()
    rclpy.shutdown()
    return result


def main():
    args = parse_arguments()
    reader = core.RosbagPointCloudReader(
        args.bag,
        topic=args.topic,
        target_frame=args.target_frame,
        cache_root=args.cache_dir,
    )
    sample_indices, sampling_method = make_sample_indices(reader, args)
    output = (
        Path(args.output)
        if args.output
        else core.DEFAULT_OUTPUT_ROOT / reader.bag_name
    )
    store = core.AnnotationStore(
        output, reader, args.annotator, args.sample_period
    )
    return launch_controller(
        args, reader, store, sample_indices, sampling_method
    )


if __name__ == "__main__":
    raise SystemExit(main())
