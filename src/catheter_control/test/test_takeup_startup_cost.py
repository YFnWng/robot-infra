"""Matched scoring/eligibility fixtures; no ROS or actuators.

Recorded uncertainty intervals, synthetic response: these isolate objective
algebra and hardware projection, not the accuracy of the recorded model.
"""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import torch
from types import SimpleNamespace

from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.transmission.backlash import BacklashSnapshot


class Response:
    def __init__(self, improvement):
        self.improvement = improvement
        self.rates = None

    def predict_sequence(self, state, rates, dt):
        self.rates = np.array(rates, copy=True)
        rates = torch.as_tensor(rates, dtype=torch.float32)
        tips = torch.zeros(*rates.shape[:-1], 3)
        moving = (rates[..., 0] > 0) & (rates[..., 2] < 0)
        tips[..., 0] = moving*self.improvement
        return SimpleNamespace(tip_base_m=tips, markers_base_m=None)


class ScoringFixture(CatheterMppi):
    def _project(self, *args, **kwargs):
        projected = super()._project(*args, **kwargs)
        # Selection later projects its executable command separately. Retain
        # the full scored population, including grouped tendon probes.
        if len(projected[0]) > 1:
            self.projected_candidates = tuple(value.copy() for value in projected)
        return projected

    def _takeup_risk_cost(self, risk_s, prediction_horizon_s):
        charge = super()._takeup_risk_cost(risk_s, prediction_horizon_s)
        self.risk_s, self.risk_charge = risk_s.copy(), charge.copy()
        return charge


class HistoricalCost(ScoringFixture):
    def _takeup_risk_cost(self, risk_s, prediction_horizon_s):
        self.risk_s = risk_s.copy()
        self.risk_charge = self.config.takeup_risk_weight*risk_s
        return self.risk_charge


def matched_case(planner_type=ScoringFixture, weight=4., improvement=.001,
                 blocked=False):
    limits = (Path(__file__).resolve().parents[2]/'control_interface'
              /'config'/'catheter_limits.yaml')
    contract = load_hardware_contract(limits, 'imricor_test')
    minimum, maximum = contract.velocity_min.copy(), contract.velocity_max.copy()
    minimum[1] = maximum[1] = 0
    contract = replace(contract, velocity_min=minimum, velocity_max=maximum)
    cfg = MppiConfig(samples=16, horizon_steps=4, step_s=.04,
                     grouped_mode_sampling=True, best_candidate_guard=True,
                     tip_weight=1., terminal_weight=6., slew_weight=0.,
                     boundary_weight=0., takeup_risk_weight=weight,
                     planning_deadline_s=2.)
    runtime = Response(improvement)
    planner = planner_type(runtime, contract, cfg)
    candidates = np.zeros((16, 4, 3))
    # Includes a physical insertion command and coupled tendon motion. The
    # canonical projection quantizes the lower-speed command for the shafts.
    candidates[1:, :, :] = [10., 0., 4.5]
    planner._samples = lambda rotation_lock_direction=0: candidates.copy()
    upper = np.array([15.04796, 0., 43.30240])
    lower = upper-np.array([10.03197, 0., 28.86827])
    snapshot = BacklashSnapshot(
        width_rad=upper, width_positive_rad=upper,
        width_negative_rad=upper, remaining_rad=np.zeros(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8),
        confidence=np.zeros(3), confirmation_count=np.zeros(3, dtype=np.int32),
        phase=('UNKNOWN',)*3,
        width_positive_lower_rad=lower, width_positive_upper_rad=upper,
        width_negative_lower_rad=lower, width_negative_upper_rad=upper)
    result = planner.plan(
        object(), [20., 0., 0., 0., 0., 0.], [.002, 0., 0.],
        target_tangent_base=[1., 0., 0.],
        previous_logical_velocity=np.zeros(6), transmission_state=snapshot,
        takeup_motor_radians_per_second=[1.88496, 1., 2.61799],
        blocked_motor_direction=(np.array([1, 0, -1], dtype=np.int8)
                                 if blocked else np.zeros(3, dtype=np.int8)))
    return result, runtime.rates, planner


def test_matched_startup_separates_penalty_from_projection():
    historical, old_rates, old = matched_case(HistoricalCost)
    bounded, bounded_rates, new = matched_case()
    unpriced, unpriced_rates, free = matched_case(weight=0.)
    assert historical.valid and bounded.valid and unpriced.valid
    np.testing.assert_array_equal(old_rates, bounded_rates)
    np.testing.assert_array_equal(old_rates, unpriced_rates)
    for old_projection, new_projection, free_projection in zip(
            old.projected_candidates, new.projected_candidates,
            free.projected_candidates):
        np.testing.assert_array_equal(old_projection, new_projection)
        np.testing.assert_array_equal(old_projection, free_projection)
    np.testing.assert_array_equal(old.risk_s, new.risk_s)
    # This stand-in has only Cartesian tracking and take-up cost. Inspect
    # every candidate, not just different winners: same states/candidates,
    # tracking objective, zero slew/boundary, identical feasibility flags.
    moving = (old_rates[..., 0] > 0) & (old_rates[..., 2] < 0)
    cartesian = ((2.-moving[:, :-1].astype(float))**2).sum(-1)
    cartesian += 6.*(2.-moving[:, -1].astype(float))**2
    coupled = np.flatnonzero(moving.all(-1))[0]
    assert cartesian[0] == pytest.approx(36.)
    assert cartesian[0] == pytest.approx(historical.zero_total_cost)
    assert cartesian[coupled] == pytest.approx(9.)
    assert cartesian[coupled] == pytest.approx(unpriced.selected_total_cost)
    assert old.risk_charge[coupled] > 110.
    assert cartesian[coupled]+old.risk_charge[coupled] > cartesian[0]
    assert cartesian[coupled]+new.risk_charge[coupled] == pytest.approx(9.64)
    assert historical.blocked_candidate_count == bounded.blocked_candidate_count
    assert bounded.blocked_candidate_count == unpriced.blocked_candidate_count
    assert historical.selected_candidate_index == 0
    assert bounded.selected_candidate_index == unpriced.selected_candidate_index > 0
    assert bounded.selected_takeup_risk_s > 13.
    assert bounded.selected_takeup_risk_cost == pytest.approx(.64)
    assert bounded.selected_total_cost == pytest.approx(
        unpriced.selected_total_cost+.64)
    assert bounded.command_logical_velocity[0] > 0
    assert bounded.command_logical_velocity[2] > 0
    np.testing.assert_array_equal(bounded.takeup_joint_position_offset,
                                  unpriced.takeup_joint_position_offset)


@pytest.mark.parametrize('blocked,improvement', [(True, .001), (False, 0.),
                                                (False, .000001)])
def test_bounded_penalty_does_not_force_motion(blocked, improvement):
    result, _, _ = matched_case(blocked=blocked, improvement=improvement)
    assert result.valid
    assert result.selected_candidate_index == 0
    assert not np.any(result.command_logical_velocity)


def test_risk_charge_preserves_short_delay_and_raw_long_delay():
    planner = object.__new__(CatheterMppi)
    planner.config = MppiConfig(takeup_risk_weight=4.)
    raw = np.array([0., .05, .10, 13.405, 27.667])
    np.testing.assert_allclose(planner._takeup_risk_cost(raw, .16),
                               [0., .2, .4, .64, .64])
    assert raw[-1] == 27.667
