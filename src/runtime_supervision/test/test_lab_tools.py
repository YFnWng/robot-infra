"""Lab tools: dry-run analysis, home check and prior re-fit helpers. No ROS graph, hardware or recordings."""
import ast
import json
from pathlib import Path

import numpy as np
import pytest

from runtime_supervision.dry_run_analysis import summarize, target_intervals
from runtime_supervision.home_check import evaluate
from runtime_supervision.prior_refit import select_images, still_windows

PACKAGE = Path(__file__).resolve().parents[1]/'runtime_supervision'


def plan(wall, valid=True, tick=20., tip=(0., 0., .05)):
    return dict(wall_ns=wall, valid=valid, reason='ok' if valid else 'deadline_missed', tick_ms=tick,
                planner_ms=tick, rollout_ms=5., root_ms=2., root_age_ms=40., deadline_missed=tick > 50,
                best_tip_sequence_m=[list(tip)]*4)


def test_dry_run_summary_distributions_and_per_target_intervals():
    report = dict(estimator_health='TRACKING', ready=True, images_dropped=0, images_skipped=1,
                  filter_rows=[dict(source_ns=1, status='initialized', reason=None, step_ms=400., consecutive_rejections=0),
                               dict(source_ns=2, status='tracking', reason=None, step_ms=40., consecutive_rejections=0),
                               dict(source_ns=3, status='holding', reason='x', step_ms=30., consecutive_rejections=1)],
                  plan_rows=[plan(10), plan(20, tick=60.), plan(40, valid=False)], counts={})
    log = [json.dumps(dict(event='target_started', index=0, repeat=0, target_m=[0., 0., .051], ros_time_ns=5)),
           json.dumps(dict(event='target_ended', index=0, repeat=0, ros_time_ns=30))]
    summary = summarize(report, target_intervals(log))
    assert summary['plans']['valid'] == 2 and summary['plans']['deadline_missed'] == 1
    assert summary['filter_step_ms']['count'] == 1 and summary['first_holds'][0]['consecutive'] == 1
    assert summary['per_target'][0]['plans'] == 2
    assert summary['per_target'][0]['predicted_terminal_error_mm']['p50'] == pytest.approx(1.0)


def test_home_check_passes_only_near_zero_still_encoders_and_close_markers():
    counts = np.zeros((10, 6))
    markers = np.tile(np.arange(12.).reshape(4, 3)/1000, (10, 1, 1))
    reference = dict(marker_mean_m=(np.arange(12.).reshape(4, 3)/1000+.0002).tolist())
    assert evaluate(counts, markers, reference)['passed']
    assert not evaluate(counts+[80, 0, 0, 0, 0, 0], markers)['encoder_near_zero']
    moving = counts.copy()
    moving[:, 2] = np.arange(10)
    assert not evaluate(moving, markers)['encoder_stationary']
    far = dict(marker_mean_m=(np.arange(12.).reshape(4, 3)/1000+.001).tolist())
    assert not evaluate(counts, markers, far)['marker_pass']


def test_refit_selection_and_still_windows():
    arrays = dict(marker_ns=np.arange(0, 1_000_000_000, 33_333_333), marker_valid=np.ones(30, bool))
    taken = select_images(arrays)
    assert np.all(np.diff(arrays['marker_ns'][taken]) >= 90_000_000) and len(taken) == 10
    encoder_ns = np.arange(0, 1_000_000_000, 10_000_000)
    counts = np.zeros((len(encoder_ns), 6))
    counts[50:, 0] = 500
    runs = still_windows(dict(encoder_ns=encoder_ns, encoder_counts=counts), arrays['marker_ns'][taken])
    assert len(runs) == 2 and runs[0][0] == 0


@pytest.mark.parametrize('name', ['home_check.py', 'dry_run_analysis.py', 'prior_refit.py'])
def test_lab_tools_cannot_command(name):
    text = (PACKAGE/name).read_text(encoding='utf-8')
    tree = ast.parse(text)
    called = {getattr(n.func, 'attr', getattr(n.func, 'id', None)) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    assert not called & {'create_publisher', 'create_client', 'create_service', 'ActionClient'}
    assert 'SET_ZERO' not in text and '/teleop/' not in text


def test_identification_labels_arms_and_still_intervals():
    from runtime_supervision.prior_identification import PASS, SWEEP, arm_table, label_mask, still_intervals
    episode = np.array(['compensated_bend_insertion_p1_level_000.000_slow_high_first',
                        'insertion_sweep_p2_level_040.000_dwell', 'stationary_noise'])
    assert list(label_mask(episode, PASS[1]) & label_mask(episode, SWEEP)) == [True, False, False]
    assert list(label_mask(episode, PASS[2]) & label_mask(episode, SWEEP)) == [False, True, False]
    u = np.r_[np.linspace(15, 0, 151), np.linspace(0, 15, 151)[1:]]
    lam = np.where(u > 10, 40.0, 40.0-8*(10-u))          # falling arm: held for 5 mm, then follows the tendon
    lam[151:] = np.where(u[151:] < 5, lam[150], lam[150]+8*(u[151:]-5))
    n = len(u)
    export = dict(tendon_mm=u, lam=lam, lam_var=np.full(n, .01), valid=np.ones(n, bool),
                  chassis_mm=np.zeros(n), knob_mm=u.copy(), tip_mm=np.c_[u, u, lam])
    arms = arm_table(export)
    assert [a['direction'] for a in arms] == [-1, 1]
    assert [round(a['width'], 1) for a in arms] == [5.1, 5.1]
    dp, dk, dl = still_intervals(export, np.ones(n, bool))
    assert len(dp) == n-1 and np.allclose(dk, np.diff(u))


def test_identification_tool_cannot_command():
    text = (PACKAGE/'prior_identification.py').read_text(encoding='utf-8')
    tree = ast.parse(text)
    called = {getattr(n.func, 'attr', getattr(n.func, 'id', None)) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    assert not called & {'create_publisher', 'create_client', 'create_service', 'ActionClient', 'create_node'}
    assert 'SET_ZERO' not in text
