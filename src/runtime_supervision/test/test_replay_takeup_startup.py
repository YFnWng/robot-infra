"""Non-actuating bag-evidence and matched-cost replay contract tests."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from catheter_control.planning.mppi import MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from runtime_supervision.replay_takeup_startup import (
    FixedCandidates, compare_policies, finite, status_array,
    transmission_from_status, read_cases, restore_engaged_root, main,
    constant_candidates, candidate_attribution)


def test_constant_grid_preserves_shaft_coordinates_and_sustained_direction():
    contract = SimpleNamespace(velocity_max=np.array([10., 0., 4.5]),
                               velocity_min=np.array([2., 0., 2.]))
    proposals = constant_candidates(contract, 4)
    np.testing.assert_array_equal(proposals[0], 0)
    shaft = proposals[..., 0]-proposals[..., 2]
    assert np.min(shaft) == -10 and np.max(shaft) == 10
    assert np.min(proposals[..., 2]) == -4.5
    np.testing.assert_array_equal(proposals[:, 0], proposals[:, -1])
    _, internal = FixedCandidates._first_physical_direction(proposals)
    assert not internal.any()


def test_attribution_keeps_better_excluded_motion_separate_from_executable():
    ledger = dict(requested=np.array([[[0, 0, 0]], [[2, 0, 0]], [[4, 0, 0]]]),
                  motor_rates=np.array([[[0]*6], [[1]*6], [[2]*6]]),
                  projected=np.zeros((3, 1, 6)), eligible=np.array([True, True, False]),
                  takeup_position_offset=np.zeros((3, 6)))
    for key in ('tracking_cost', 'total_cost'):
        ledger[key] = np.array([10., 11., 1.])
    for key in ('slew_cost', 'boundary_cost', 'takeup_cost'):
        ledger[key] = np.zeros(3)
    rows = candidate_attribution(ledger)
    assert rows['hold']['index'] == 0
    assert rows['best_eligible_motion']['index'] == 1
    assert rows['best_excluded_sustained_motion']['index'] == 2
    assert not rows['best_excluded_motion']['eligible']


def status():
    values = {f'backlash_{key}': json.dumps(value) for key, value in {
        'width_rad': [15, 0, 43], 'width_positive_rad': [15, 0, 43],
        'width_negative_rad': [15, 0, 43], 'remaining_rad': [0, 0, 0],
        'remaining_lower_rad': [0, 0, 0], 'remaining_upper_rad': [0, 0, 0],
        'confidence': [0, 0, 0], 'motion_direction': [0, 0, 0],
        'engaged_direction': [0, 0, 0], 'confirmation_count': [0, 0, 0],
        'width_positive_interval_rad': [[5, 0, 14], [15, 0, 43]],
        'width_negative_interval_rad': [[5, 0, 14], [15, 0, 43]],
        'phase': ['UNKNOWN']*3}.items()}
    return values


def test_recorded_intervals_are_not_replaced_with_engaged_workload():
    belief = transmission_from_status(status())
    assert belief.phase == ('UNKNOWN',)*3
    np.testing.assert_array_equal(belief.width_positive_upper_rad, [15, 0, 43])
    np.testing.assert_array_equal(belief.confidence, 0)
    assert belief.engaged_gain is None


@pytest.mark.parametrize('key,value', [
    ('confidence', [0, 2, 0]), ('confirmation_count', [0, .5, 0]),
    ('motion_direction', [0, .5, 0]), ('motion_direction', [0, 256, 0]),
    ('phase', ['UNKNOWN']), ('phase', ['MAYBE']*3),
    ('width_positive_interval_rad', [[16, 0, 0], [15, 0, 43]]),
    ('remaining_upper_rad', [-1, 0, 0]), ('remaining_rad', [float('nan')]*3)])
def test_invalid_recorded_belief_fails_closed(key, value):
    values = status()
    values['backlash_'+key] = json.dumps(value)
    with pytest.raises(ValueError):
        transmission_from_status(values)


def test_missing_evidence_is_not_invented():
    values = status()
    del values['backlash_confidence']
    with pytest.raises(KeyError):
        transmission_from_status(values)


def test_shape_and_sign_validation():
    with pytest.raises(ValueError):
        finite([1], (3,), 'x')
    with pytest.raises(ValueError):
        status_array({'x': '[1,2,3]'}, 'x', signs=True)


def test_visual_root_rejects_unregistered_frame_before_runtime_calls():
    trace = SimpleNamespace(header=SimpleNamespace(frame_id='camera'), state_timestamp_ns=1)
    with pytest.raises(ValueError, match='frame'):
        restore_engaged_root(None, trace)


class Response:
    def predict_sequence(self, root, rates, dt):
        rates = torch.as_tensor(rates, dtype=torch.float32)
        tips = torch.zeros(*rates.shape[:-1], 3)
        tips[..., 0] = ((rates[..., 0] > 0) & (rates[..., 2] < 0))*.001
        return SimpleNamespace(tip_base_m=tips, markers_base_m=None)


def test_full_candidate_ledger_and_timed_selection_agree():
    repo = Path(__file__).resolve().parents[3]
    contract = load_hardware_contract(repo/'src/control_interface/config/catheter_limits.yaml', 'imricor_test')
    vmax = contract.velocity_max.copy(); vmax[1] = 0
    contract = replace(contract, velocity_max=vmax, velocity_min=np.minimum(contract.velocity_min, vmax))
    config = MppiConfig(samples=16, horizon_steps=4, step_s=.04,
                        grouped_mode_sampling=True, best_candidate_guard=True,
                        slew_weight=0, boundary_weight=0, planning_deadline_s=2)
    candidates = np.zeros((16, 4, 3)); candidates[1:] = [10, 0, 4.5]
    class Root:
        def clone(self):
            return Root()
    rows, ledgers = compare_policies(Response(), contract, config, Root(),
        [20, 0, 0, 0, 0, 0], [.002, 0, 0], dict(
            transmission_state=transmission_from_status(status()),
            takeup_motor_radians_per_second=[1.88496, 1, 2.61799],
            target_tangent_base=[1, 0, 0]), candidates, repeats=2)
    assert rows['historical']['selected_candidate'] == 0
    assert rows['bounded']['selected_candidate'] == rows['zero_penalty']['selected_candidate'] > 0
    assert rows['bounded']['delay_cost'] == pytest.approx(.64)
    assert all(row['invalid_timed_plans'] == 0 for row in rows.values())
    for ledger in ledgers.values():
        assert len(ledger['eligible']) >= 16
        assert not np.shares_memory(ledger['total_cost'], ledger['tracking_cost'])
    with pytest.raises(ValueError):
        FixedCandidates(Response(), contract, config, candidates, 'unknown')


@pytest.mark.parametrize('offsets', [['nan'], ['2', '1'], ['1', '1'], ['-1']])
def test_invalid_windows_rejected_before_loading_artifacts(offsets):
    with pytest.raises(SystemExit):
        main(['session', '--model-manifest', 'missing', '--limits-file', 'missing',
              '--offsets-s', *offsets, '--output', 'unused'])


def test_recorded_visual_root_fk_and_history_independence():
    """Actual deployed CPU model, optional external evidence, no ROS node."""
    session = Path('/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/'
        'continuous_twoaxis_circle_home_grouped_takeup_v171_n512_speed1_mount01_repeat01_20261005T235706766686Z')
    if not session.is_dir():
        pytest.skip('external recording unavailable')
    from catheter_control.orchestration.runtime import load_runtime
    model = '/home/chen-lab/Yifan/cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json'
    runtime = load_runtime(model, device='cpu', options={
        'marker_estimator': 'ukf', 'adaptation_enabled': False}).runtime
    case = read_cases(session, [.9], .20)[0]
    assert case['actual_offset_s'] >= .9
    assert all(0 <= age <= .20 for age in case['evidence_age_s'].values())
    root = restore_engaged_root(runtime, case['trace'])
    np.testing.assert_allclose(runtime.markers_for_state(root)[-1].detach().numpy(),
                               case['trace'].estimated_tip_m, atol=1e-7, rtol=0)
    altered = root.clone()
    altered.lambda_value += 100
    altered.history = runtime.history_model.initial_state(root.downstream[2]+100)
    rates = np.zeros((2, 4, 3)); rates[1, :, [0, 2]] = .1
    a = runtime.predict_control_sequence_coarse(root, rates, .04, always_engaged=True)
    b = runtime.predict_control_sequence_coarse(altered, rates, .04, always_engaged=True)
    torch.testing.assert_close(a.tip_base_m, b.tip_base_m, atol=0, rtol=0)
