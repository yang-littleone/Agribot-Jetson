import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'analyze_field_trials.py')
SPEC = importlib.util.spec_from_file_location('field_analysis', MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def test_spearman_has_expected_direction():
    assert analysis.spearman([0.9, 0.7, 0.4, 0.2], [0.01, 0.03, 0.08, 0.15]) == -1.0


def test_risk_scores_give_perfect_auc():
    labels = [0, 0, 1, 1]
    danger_scores = [0.1, 0.2, 0.8, 0.9]
    assert analysis.roc_auc(labels, danger_scores) == 1.0
    assert analysis.average_precision(labels, danger_scores) == 1.0


def test_error_metrics_include_tail_and_jitter():
    rows = [
        {
            'time_s': 0.0, 'lateral_error_m': -0.01,
            'heading_error_deg': 1.0, 'valid': 1.0, 'center_offset_m': 0.00,
        },
        {
            'time_s': 0.1, 'lateral_error_m': 0.03,
            'heading_error_deg': -2.0, 'valid': 1.0, 'center_offset_m': 0.02,
        },
        {
            'time_s': 0.2, 'lateral_error_m': 0.10,
            'heading_error_deg': 3.0, 'valid': 0.0, 'center_offset_m': -0.01,
        },
    ]
    report = analysis.error_metrics(rows)
    assert report['samples'] == 3
    assert report['lateral_max_m'] == 0.10
    assert report['valid_detection_rate'] == 2 / 3
    assert report['centerline_frame_jitter_p95_m'] == 0.03


def test_single_weight_sensitivity_generates_all_variants():
    rows = []
    for index in range(6):
        rows.append({
            'support_score': 1.0 - index * 0.1,
            'observation_score': 0.9 - index * 0.08,
            'width_score': 0.95 - index * 0.05,
            'residual_score': 0.85 - index * 0.07,
            'safety_score': 0.8 - index * 0.06,
            'support_weight': 0.30,
            'observation_weight': 0.20,
            'width_weight': 0.20,
            'residual_weight': 0.15,
            'safety_weight': 0.15,
            'lateral_error_m': index * 0.02,
        })
    report = analysis.sensitivity_report(rows)
    assert report['samples'] == 6
    assert len(report['variants']) == 10


def test_manual_completion_annotation_is_counted():
    rows = [
        {
            'trial_id': 'run_1', 'time_s': 0.0, 'lateral_error_m': 0.01,
            'heading_error_deg': 1.0, 'plant_contact': 0, 'intervention': 0,
            'completed': 0, 'navigation_safety_state': 'TRACKING',
        },
        {
            'trial_id': 'run_1', 'time_s': 1.0, 'lateral_error_m': 0.02,
            'heading_error_deg': 1.5, 'plant_contact': 0, 'intervention': 0,
            'completed': 1, 'navigation_safety_state': 'TRACKING',
        },
    ]
    report = analysis.closed_loop_metrics(rows)
    assert report['completed_trials'] == 1
    assert report['completion_rate'] == 1.0
