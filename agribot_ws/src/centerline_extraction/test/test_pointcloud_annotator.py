import importlib.util
import json
from pathlib import Path
import tempfile

import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "pointcloud_centerline_annotator.py"
)
SPEC = importlib.util.spec_from_file_location(
    "pointcloud_centerline_annotator", MODULE_PATH
)
annotator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(annotator)


def test_parallel_row_fit_recovers_centerline_and_heading():
    left = [(0.2, 0.35), (1.0, 0.39), (1.8, 0.43)]
    right = [(0.2, -0.27), (1.0, -0.23), (1.8, -0.19)]
    result = annotator.fit_parallel_rows(left, right)
    assert abs(result["slope"] - 0.05) < 1.0e-12
    assert abs(result["center_intercept_m"] - 0.03) < 1.0e-12
    expected_width = 0.62 / np.sqrt(1.0 + 0.05**2)
    assert abs(result["row_width_m"] - expected_width) < 1.0e-12


def test_transform_graph_composes_child_to_parent_transforms():
    graph = annotator.TransformGraph()
    base_from_sensor = np.eye(4)
    base_from_sensor[:3, 3] = [0.03, 0.0, 0.115]
    odom_from_base = np.eye(4)
    odom_from_base[:3, 3] = [1.0, 2.0, 0.0]
    graph.add("base_link", "laser_link", base_from_sensor)
    graph.add("odom", "base_link", odom_from_base)
    result = graph.lookup("odom", "laser_link")
    assert np.allclose(result[:3, 3], [1.03, 2.0, 0.115])


def test_quaternion_round_trip_preserves_rotation():
    angle = np.deg2rad(37.0)
    quaternion = np.array(
        [0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0)]
    )
    matrix = annotator.transform_matrix(
        [1.2, -0.4, 0.1], quaternion
    )
    recovered = annotator.quaternion_from_matrix(matrix)
    assert abs(float(np.dot(quaternion, recovered))) > 1.0 - 1.0e-12


def test_time_sampling_includes_first_and_last_frame():
    timestamps = [0, 100_000_000, 200_000_000, 900_000_000]
    assert annotator.sampled_frame_indices(timestamps, 0.2) == [0, 2, 3]


def test_annotation_records_anchor_and_selected_frame_delta():
    class FakeReader:
        bag_name = "test_bag"
        source_path = Path("/tmp/test_bag")
        topic = "/livox/lidar"
        target_frame = "base_link"

    metadata = {
        "frame_index": 103,
        "bag_timestamp_ns": 10_300_000_000,
        "cloud_stamp_ns": 10_300_000_000,
        "cloud_frame": "laser_link",
        "target_frame": "base_link",
        "transform_source": "bag_tf_static",
        "transform_matrix": np.eye(4).tolist(),
        "raw_point_count": 10,
    }
    roi = {
        "x_min": 0.1,
        "x_max": 2.2,
        "y_min": -1.4,
        "y_max": 1.4,
        "z_min": 0.1,
        "z_max": 0.5,
    }
    with tempfile.TemporaryDirectory() as output_dir:
        store = annotator.AnnotationStore(
            output_dir, FakeReader(), "tester", 2.0
        )
        store.save(
            metadata,
            roi,
            [(0.2, 0.3), (1.0, 0.3)],
            [(0.2, -0.3), (1.0, -0.3)],
            "valid",
            sampling_anchor_frame_index=100,
            sampling_anchor_timestamp_ns=10_000_000_000,
        )
        document = json.loads(
            Path(
                output_dir, "centerline_annotations.json"
            ).read_text(encoding="utf-8")
        )
        saved = document["annotations"]["103"]
        assert saved["sampling_anchor_frame_index"] == 100
        assert saved["selected_frame_delta"] == 3
        assert saved["selected_time_delta_s"] == 0.3
        assert store.find_by_anchor(100)["frame_index"] == 103
