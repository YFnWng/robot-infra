from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from catheter_control.backlash import BacklashSnapshot
from catheter_control.engaged_gain import EngagedGainConfig, EngagedGainEstimator
from catheter_control.hardware_contract import (
    MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM, load_hardware_contract)
from catheter_control.mppi import (
    CONTROL_AXES, CatheterMppi, MppiConfig, _synchronize_torch_device,
    _tip_tracking_error_squared)


ROOT = Path(__file__).resolve().parents[2]
LIMITS = ROOT / "automation" / "config" / "catheter_limits.yaml"


class LinearRollout:
    """Tiny differentiable-shape stand-in for the streaming runtime."""

    def predict_sequence(self, state, motor_velocity_sequence, dt_sequence):
        rates = torch.as_tensor(motor_velocity_sequence, dtype=torch.float32)
        displacement = torch.cumsum(rates*float(dt_sequence), dim=1)
        tips = torch.zeros(*rates.shape[:-1], 3)
        tips[..., 0] = -1e-3*displacement[..., 2]
        fractions = torch.tensor([0.0, 0.2, 0.55, 1.0])
        markers = tips[..., None, :]*fractions[None, None, :, None]
        return SimpleNamespace(tip_base_m=tips, markers_base_m=markers)


@pytest.fixture
def contract():
    return load_hardware_contract(LIMITS, "imricor_test")


def config(**changes):
    values = dict(horizon_steps=6, samples=192, planning_deadline_s=1.0,
                  seed=7)
    values.update(changes)
    return MppiConfig(**values)


def test_batch_projection_exactly_matches_scalar_contract(contract):
    rng = np.random.default_rng(4)
    requested = rng.uniform(-50, 50, (9, 6))
    positions = rng.uniform(contract.position_lower,
                            contract.position_upper, (9, 6))
    batch = contract.project_velocity_batch(requested, positions)
    for index in range(9):
        scalar = contract.project_velocity(requested[index], positions[index])
        assert batch.logical_velocity[index] == pytest.approx(
            scalar.logical_velocity)
        assert batch.motor_rpm[index] == pytest.approx(scalar.motor_rpm)
        assert batch.motor_radians_per_second[index] == pytest.approx(
            scalar.motor_radians_per_second)


def test_planner_moves_toward_tip_target_and_pads_unused_axes(contract):
    planner = CatheterMppi(LinearRollout(), contract, config())
    plans = [
        planner.plan(
            object(), [20, 0, 7, 40, 0, 0], [0.002, 0, 0])
        for _ in range(10)]
    assert all(plan.valid and plan.reason == "ok" for plan in plans)
    assert all(plan.command_logical_velocity.shape == (6,) for plan in plans)
    assert any(plan.command_logical_velocity[2] > 0 for plan in plans)
    assert all(np.all(plan.command_logical_velocity[3:] == 0)
               for plan in plans)
    plan = plans[-1]
    assert plan.logical_velocity_sequence.shape == (6, 6)
    assert plan.motor_radians_per_second_sequence.shape == (6, 6)
    assert plan.best_tip_sequence_m.shape == (6, 3)
    assert plan.command_tip_sequence_m.shape == (6, 3)
    assert all(value >= 0.0 for value in (
        plan.sample_projection_ms, plan.rollout_ms,
        plan.cost_weighting_ms, plan.update_projection_ms))


def test_path_corridor_penalizes_lag_but_not_forward_overshoot():
    tips = torch.tensor([[[-.001, .001, 0.0]],
                         [[+.001, .001, 0.0]]])
    target = torch.zeros((1, 3))
    tangent = torch.tensor([[1.0, 0.0, 0.0]])

    point_cost = _tip_tracking_error_squared(tips, target, None)
    path_cost = _tip_tracking_error_squared(tips, target, tangent)

    assert point_cost[:, 0].tolist() == pytest.approx([2.0, 2.0])
    assert path_cost[:, 0].tolist() == pytest.approx([2.0, 1.0])


def test_disabled_rotation_is_not_sampled_or_commanded(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(LinearRollout(), local, config(samples=32))
    planner._nominal[:, 1] = 20.0

    samples = planner._samples(rotation_lock_direction=1)
    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.002, 0, 0],
        direction_lease=np.array([0, 1, 0], dtype=np.int8))

    assert np.all(samples[..., 1] == 0.0)
    assert plan.valid
    assert plan.direction_lease.tolist() == [0, 0, 0]
    assert plan.command_logical_velocity[1] == 0.0
    assert np.all(plan.logical_velocity_sequence[:, 1] == 0.0)
    assert np.all(plan.motor_radians_per_second_sequence[:, 1] == 0.0)


def test_sparse_target_preview_uses_projection_and_zero_rotation(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(
        LinearRollout(), local,
        config(horizon_steps=4, step_s=0.04, samples=32))

    preview = planner.predict_sparse_targets(
        object(), [20, 0, 7, 40, 0, 0], [0.01, 0.02, 0.03],
        [[0.0, 0.0, 2.0]], 25, [1, 0, .5, 0, 0, 0])

    assert preview.target_tip_m.shape == (1, 3)
    assert preview.predicted_tip_displacement_m.shape == (1, 3)
    assert preview.realized_logical_displacement.shape == (1, 6)
    assert preview.endpoint_joint_position.shape == (1, 6)
    assert preview.realized_logical_displacement[0, 1] == 0.0
    assert preview.endpoint_joint_position[0, 1] == 0.0


def test_sparse_target_preview_rejects_rotation(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(LinearRollout(), local, config(samples=32))

    with pytest.raises(ValueError, match="rejects rotation"):
        planner.predict_sparse_targets(
            object(), [20, 0, 7, 40, 0, 0], [0.01, 0.02, 0.03],
            [[0.0, 1.0, 2.0]], 25, [1, 0, .5, 0, 0, 0])


def test_sparse_target_reserve_ignores_inactive_axis_at_boundary(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(
        LinearRollout(), local,
        config(horizon_steps=4, step_s=0.04, samples=32))

    preview = planner.predict_sparse_targets(
        object(), [20, 0, contract.control_position_lower[2], 40, 0, 0],
        [0.01, 0.02, 0.03], [[5.0, 0.0, 0.0]], 25,
        [2.0, 0.0, 0.5, 0.0, 0.0, 0.0])

    assert preview.endpoint_joint_position[0, 2] == pytest.approx(
        contract.control_position_lower[2])
    assert preview.realized_logical_displacement[0, 2] == 0.0


def test_sparse_target_preview_resolves_tolerance_qualified_inactive_axis(
        contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(
        LinearRollout(), local,
        config(horizon_steps=4, step_s=0.04, samples=32))
    quantized_below_lower = (
        contract.position_lower[2]
        - 0.65*contract.feedback_limit_tolerance[2])

    preview = planner.predict_sparse_targets(
        object(), [20, 0, quantized_below_lower, 40, 0, 0],
        [0.01, 0.02, 0.03], [[5.0, 0.0, 0.0]], 25,
        [2.0, 0.0, 0.5, 0.0, 0.0, 0.0])

    assert preview.endpoint_joint_position[0, 2] == pytest.approx(
        contract.position_lower[2])
    assert preview.realized_logical_displacement[0, 2] == 0.0


def test_sparse_target_preview_rejects_feedback_beyond_tolerance(contract):
    minimum = contract.velocity_min.copy()
    maximum = contract.velocity_max.copy()
    minimum[1] = 0.0
    maximum[1] = 0.0
    local = replace(contract, velocity_min=minimum, velocity_max=maximum)
    planner = CatheterMppi(LinearRollout(), local, config(samples=32))
    invalid_below_lower = (
        contract.position_lower[2]
        - 1.01*contract.feedback_limit_tolerance[2])

    with pytest.raises(ValueError, match="feedback-qualified limits"):
        planner.predict_sparse_targets(
            object(), [20, 0, invalid_below_lower, 40, 0, 0],
            [0.01, 0.02, 0.03], [[5.0, 0.0, 0.0]], 25,
            [2.0, 0.0, 0.5, 0.0, 0.0, 0.0])


def test_planner_can_hold_or_recover_inside_hard_limit_beyond_control_margin(
        contract):
    planner = CatheterMppi(LinearRollout(), contract, config(samples=32))
    # The autonomous insertion upper bound is 49 mm, while the independent
    # manager hard bound is 50 mm. This is the state observed in the hardware
    # fault that previously rejected every candidate, including zero.
    position = np.array([49.01, 0, 7, 40, 0, 0], dtype=np.float64)

    plan = planner.plan(object(), position, [0.0, 0.0, 0.0])

    assert plan.valid
    assert plan.reason == "ok"
    assert plan.command_logical_velocity[0] <= 0.0


def test_takeup_offset_farther_beyond_control_margin_remains_blocked(contract):
    planner = CatheterMppi(LinearRollout(), contract, config(samples=2))
    requested = np.zeros((2, planner.config.horizon_steps, 3))
    offsets = np.zeros((2, 6))
    offsets[1, 0] = 0.02

    *_, blocked = planner._project(
        requested, np.array([49.01, 0, 7, 40, 0, 0]),
        initial_position_offset=offsets)

    assert not blocked[0]
    assert blocked[1]


def test_response_instrumentation_does_not_add_a_second_rollout(contract):
    class CountingRollout(LinearRollout):
        def __init__(self):
            self.calls = 0

        def predict_sequence(self, *args):
            self.calls += 1
            return super().predict_sequence(*args)

    backend = CountingRollout()
    planner = CatheterMppi(backend, contract, config(samples=8))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.002, 0, 0])

    assert plan.valid
    assert plan.command_tip_sequence_m.shape == (6, 3)
    assert backend.calls == 1


def test_best_candidate_guard_avoids_harmful_nonconvex_average(
        contract):
    class DisconnectedRollout:
        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 0] > 0.0, .001,
                torch.where(rates[..., 0] < 0.0, -.004, 0.0))
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=2, samples=8, temperature=1000.0,
        tip_weight=1.0, terminal_weight=1.0,
        slew_weight=0.0, boundary_weight=0.0,
        best_candidate_guard=True)
    planner = CatheterMppi(DisconnectedRollout(), contract, cfg)
    requested = np.zeros((8, 2, 3))
    requested[1, :, 0] = 1.0
    requested[2:, :, 0] = -4.0
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0])

    assert plan.valid
    assert plan.best_candidate_selected
    assert plan.scored_candidate_guard_applied
    assert plan.selected_candidate_index >= 0
    assert plan.selected_total_cost <= plan.zero_total_cost
    assert plan.command_logical_velocity[0] > 0.0
    assert plan.weighted_tracking_cost > plan.zero_tracking_cost
    assert plan.best_tracking_cost < plan.zero_tracking_cost
    np.testing.assert_allclose(plan.command_tip_sequence_m[:, 0], .001)


def test_legacy_direction_lease_selects_complete_scored_continue_plan(
        contract):
    class RotationRollout:
        def __init__(self):
            self.batch_sizes = []

        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            self.batch_sizes.append(len(motor_velocity_sequence))
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 1] < 0.0, .001,
                torch.where(rates[..., 1] > 0.0, .0002, 0.0))
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=1, samples=9,
        tip_weight=1.0, terminal_weight=1.0,
        slew_weight=0.0,
        boundary_weight=0.0,
        best_candidate_guard=True, grouped_mode_sampling=False)
    requested = np.zeros((9, 1, 3))
    requested[1, 0, 1] = -20.0
    requested[2, 0, 1] = 20.0
    backend = RotationRollout()
    planner = CatheterMppi(backend, contract, cfg)
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    constrained = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0],
        direction_lease=np.array([0, 1, 0], dtype=np.int8))

    assert constrained.proposed_reversal_direction.tolist() == [0, -1, 0]
    assert constrained.direction_lease_applied
    assert constrained.unrestricted_total_cost < (
        constrained.lease_constrained_total_cost)
    assert not constrained.hold_branch_applied
    assert constrained.proposal_group_count == 1
    assert constrained.selected_reversal_mask == 0
    assert constrained.command_logical_velocity[1] > 0.0
    assert constrained.reversal_axis_cost_improvement[1] > 0.0
    assert constrained.reversal_axis_terminal_error_improvement_mm[1] > 0.0
    # Every proposal mode shares one fixed-size model invocation; no selected
    # plan is altered after scoring.
    assert backend.batch_sizes == [cfg.samples]

    approved = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0],
        direction_lease=np.array([0, 1, 0], dtype=np.int8),
        approved_reversal_direction=np.array([0, -1, 0], dtype=np.int8))
    assert not approved.direction_lease_applied
    assert approved.command_logical_velocity[1] < 0.0
    assert backend.batch_sizes == [cfg.samples, cfg.samples]


def test_grouped_mode_selects_global_complete_plan_without_scheduler_veto(
        contract):
    class RotationRollout:
        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 1] < 0.0, .001,
                torch.where(rates[..., 1] > 0.0, .0002, 0.0))
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=1, samples=10,
        tip_weight=1.0, terminal_weight=1.0,
        slew_weight=0.0,
        boundary_weight=0.0,
        best_candidate_guard=True, grouped_mode_sampling=True)
    requested = np.zeros((10, 1, 3))
    requested[1, 0, 1] = 20.0
    requested[2, 0, 1] = -20.0
    planner = CatheterMppi(RotationRollout(), contract, cfg)
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0],
        direction_lease=np.array([0, 1, 0], dtype=np.int8),
        # Deliberately withhold legacy approval. Grouped mode must compare
        # and execute the complete U/C mode winner without a second veto.
        approved_reversal_direction=np.zeros(3, dtype=np.int8))

    assert plan.proposal_group_count == 2
    assert plan.proposed_reversal_direction.tolist() == [0, -1, 0]
    assert plan.selected_reversal_mask == 1 << 1
    assert not plan.direction_lease_applied
    assert plan.selected_candidate_index == plan.unrestricted_candidate_index
    assert plan.command_logical_velocity[1] < 0.0


def test_grouped_mode_prices_one_risk_adjusted_takeup_transaction(contract):
    class RotationRollout:
        def __init__(self, response_m):
            self.response_m = response_m

        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 1] < 0.0, self.response_m, 0.0)
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=1, samples=10,
        tip_weight=1.0, terminal_weight=1.0, slew_weight=0.0,
        takeup_risk_weight=4.0, takeup_confirmation_time_s=0.10,
        boundary_weight=0.0, best_candidate_guard=True,
        grouped_mode_sampling=True)
    requested = np.zeros((10, 1, 3))
    # Index 2 belongs to the unconstrained half of the round-robin grouped
    # population and reverses a positive response-confirmed rotation lease.
    requested[2, 0, 1] = -20.0
    snapshot = BacklashSnapshot(
        width_rad=np.zeros(3),
        width_positive_rad=np.zeros(3),
        width_negative_rad=np.array([0.0, 4.0, 0.0]),
        remaining_rad=np.zeros(3),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.array([1.0, 0.5, 1.0]),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("UNKNOWN", "ENGAGED", "UNKNOWN"),
        width_negative_lower_rad=np.array([0.0, 2.0, 0.0]),
        width_negative_upper_rad=np.array([0.0, 4.0, 0.0]))

    marginal = CatheterMppi(RotationRollout(.0002), contract, cfg)
    marginal._samples = lambda rotation_lock_direction=0: requested.copy()
    marginal_plan = marginal.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0],
        previous_logical_velocity=np.zeros(6),
        direction_lease=np.array([0, 1, 0], dtype=np.int8),
        transmission_state=snapshot,
        takeup_motor_radians_per_second=np.array([8.0, 40.0, 4.5]))

    # The reverse improves tracking by 0.36 cost units, but its single
    # risk-adjusted transaction costs 4 * (5/40 + 0.10) = 0.9. The 5 rad
    # travel is upper=4 plus (1-confidence)*(upper-lower)=1.
    assert marginal_plan.selected_candidate_index == 0
    assert marginal_plan.selected_reversal_mask == 0
    assert marginal_plan.selected_takeup_risk_cost == 0.0

    material = CatheterMppi(RotationRollout(.0049), contract, cfg)
    material._samples = lambda rotation_lock_direction=0: requested.copy()
    material_plan = material.plan(
        object(), [20, 0, 7, 40, 0, 0], [.005, 0, 0],
        previous_logical_velocity=np.zeros(6),
        direction_lease=np.array([0, 1, 0], dtype=np.int8),
        transmission_state=snapshot,
        takeup_motor_radians_per_second=np.array([8.0, 40.0, 4.5]))

    # The cost is soft: a materially better complete reverse plan still wins.
    # Grouping retains the candidate but compacts the round-robin partitions,
    # so its executable batch index is not its source-population index.
    assert material_plan.selected_candidate_index > 0
    assert material_plan.selected_reversal_mask == 1 << 1
    assert material_plan.selected_switch_count == 1
    assert material_plan.selected_takeup_risk_s == pytest.approx(0.225)
    assert material_plan.selected_takeup_risk_cost == pytest.approx(0.9)
    assert material_plan.command_logical_velocity[1] < 0.0


def test_grouped_candidates_partition_full_population_without_duplication(
        contract):
    cfg = config(horizon_steps=1, samples=16,
                 grouped_mode_sampling=True)
    planner = CatheterMppi(LinearRollout(), contract, cfg)
    requested = np.zeros((16, 1, 3))
    requested[:, 0, 1] = np.arange(1.0, 17.0)
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    grouped, labels, count = planner._grouped_candidates(
        np.array([0, 1, 0], dtype=np.int8))

    assert count == 2
    assert np.bincount(labels, minlength=3).tolist() == [10, 0, 10]
    # Positive rotation is valid in both unconstrained and +continue groups.
    # Excluding the appended deterministic probes, every configured random
    # sample still occurs exactly once.
    np.testing.assert_array_equal(
        np.sort(grouped[~planner._last_tendon_probe_mask, 0, 1]),
        np.arange(1.0, 17.0))


def test_axis2_continue_group_uses_physical_shaft_sign(contract):
    """Axis 2 motor units have the opposite sign from shaft radians/s."""
    class BendRollout:
        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 2] > 0.0, .001,
                torch.where(rates[..., 2] < 0.0, .0002, 0.0))
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=1, samples=9,
        tip_weight=1.0, terminal_weight=1.0,
        slew_weight=0.0,
        boundary_weight=0.0,
        best_candidate_guard=True)
    requested = np.zeros((9, 1, 3))
    # A negative logical bend rate maps to positive physical shaft rotation.
    requested[1, 0, 2] = 5.0
    requested[2, 0, 2] = -5.0
    planner = CatheterMppi(BendRollout(), contract, cfg)
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [.001, 0, 0],
        direction_lease=np.array([0, 0, -1], dtype=np.int8))

    assert plan.proposed_reversal_direction.tolist() == [0, 0, 1]
    assert not plan.hold_branch_applied
    assert plan.selected_reversal_mask == 1 << 2
    projected = contract.project_velocity(
        plan.command_logical_velocity, [20, 0, 7, 40, 0, 0])
    assert projected.motor_radians_per_second[2] > 0.0


def test_reversal_mode_reserves_takeup_travel_before_joint_limit(contract):
    class RotationRollout:
        def predict_sequence(self, state, motor_velocity_sequence, step_s):
            rates = torch.as_tensor(
                motor_velocity_sequence, dtype=torch.float32)
            tips = torch.zeros(*rates.shape[:-1], 3)
            tips[..., 0] = torch.where(
                rates[..., 1] > 0.0, .001,
                torch.where(rates[..., 1] < 0.0, .0002, 0.0))
            return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    cfg = config(
        horizon_steps=1, samples=8, tip_weight=1.0,
        terminal_weight=1.0, slew_weight=0.0, boundary_weight=0.0, best_candidate_guard=True)
    requested = np.zeros((8, 1, 3))
    requested[1, 0, 1] = -20.0
    requested[2, 0, 1] = 20.0
    planner = CatheterMppi(RotationRollout(), contract, cfg)
    planner._samples = lambda rotation_lock_direction=0: requested.copy()
    snapshot = BacklashSnapshot(
        width_rad=np.zeros(3),
        width_positive_rad=np.array([0.0, 2.0, 0.0]),
        width_negative_rad=np.zeros(3),
        remaining_rad=np.zeros(3),
        motion_direction=np.array([0, -1, 0], dtype=np.int8),
        engaged_direction=np.array([0, -1, 0], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("UNKNOWN", "ENGAGED", "UNKNOWN"))

    plan = planner.plan(
        object(), [20, 269, 7, 40, 0, 0], [.001, 0, 0],
        direction_lease=np.array([0, -1, 0], dtype=np.int8),
        approved_reversal_direction=np.array([0, 1, 0], dtype=np.int8),
        transmission_state=snapshot)

    # The +shaft reversal would consume about 8.6 logical degrees before any
    # useful motion, so that entire mode is infeasible near +270 degrees.
    assert np.isinf(plan.mode_best_total_cost[1 << 1])
    assert plan.blocked_candidate_count > 0
    assert plan.selected_reversal_mask == 0
    assert plan.command_logical_velocity[1] <= 0.0


def test_planner_prefers_tip_only_control_rollout(contract):
    class OptimizedRollout(LinearRollout):
        def __init__(self):
            self.control_calls = 0
            self.complete_calls = 0

        def predict_control_sequence(self, *args):
            self.control_calls += 1
            result = super().predict_sequence(*args)
            result.markers_base_m = None
            return result

        def predict_sequence(self, *args):
            self.complete_calls += 1
            return super().predict_sequence(*args)

    backend = OptimizedRollout()
    planner = CatheterMppi(backend, contract, config(samples=8))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.002, 0, 0])

    assert plan.valid
    assert backend.control_calls == 1
    assert backend.complete_calls == 0


def test_cuda_timing_boundary_synchronizes_requested_device(monkeypatch):
    synchronized = []
    monkeypatch.setattr(
        torch.cuda, "synchronize", lambda device: synchronized.append(device))

    _synchronize_torch_device(torch.device("cpu"))
    _synchronize_torch_device(torch.device("cuda:0"))

    assert synchronized == [torch.device("cuda:0")]


def test_position_guard_is_applied_at_every_rollout_step(contract):
    planner = CatheterMppi(LinearRollout(), contract, config())
    plan = planner.plan(
        object(), [20, 0, 14.99, 40, 0, 0], [0.003, 0, 0])
    assert plan.valid
    assert plan.command_logical_velocity[2] == 0.0
    assert np.all(plan.logical_velocity_sequence[:, 2] <= 0.0)


def test_saturation_block_uses_pre_projection_physical_direction(contract):
    """Coupling leakage cannot hide a direction blocked at take-up."""
    planner = CatheterMppi(
        LinearRollout(), contract, config(horizon_steps=2, samples=8))
    boundary = np.array(
        [24, 0, contract.control_position_lower[2], 40, 0, 0],
        dtype=float)
    physical_rates = np.array([5.0, 0.0, 3.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(
            physical_rates))
    requested = np.zeros((2, 2, 3))
    requested[1] = desired[:3]

    logical, _, motor, _, _, blocked = planner._project(
        requested, boundary,
        blocked_motor_direction=np.array([0, 0, 1], dtype=np.int8))

    assert blocked.tolist() == [False, True]
    np.testing.assert_array_equal(logical[1], 0.0)
    np.testing.assert_array_equal(motor[1], 0.0)


def test_coupled_saturation_blocks_only_complete_direction_mode(contract):
    planner = CatheterMppi(
        LinearRollout(), contract, config(horizon_steps=2, samples=8))
    home = np.array([20, 0, 0, 0, 0, 0], dtype=float)
    requested = np.zeros((4, 2, 3))
    # Magnitudes keep the intended logical commands above the hardware
    # minimum-velocity floors, so the classified physical signs are exact.
    modes = (
        np.array([5.0, 0.0, -12.0]),
        np.array([5.0, 0.0, 0.0]),
        np.array([0.0, 0.0, -12.0]),
    )
    for index, physical_rates in enumerate(modes, start=1):
        logical = contract.motor_axis_to_logical_velocity(
            contract.motor_radians_per_second_to_motor_axis_velocity(
                np.r_[physical_rates, 0.0, 0.0, 0.0]))
        requested[index] = logical[:3]

    logical, _, _, _, _, blocked = planner._project(
        requested, home,
        blocked_motor_direction=np.array([1, 0, -1], dtype=np.int8))

    assert blocked.tolist() == [False, True, False, False]
    np.testing.assert_array_equal(logical[1], 0.0)
    assert np.any(logical[2])
    assert np.any(logical[3])


def test_planner_excludes_candidates_in_registered_saturation_direction(
        contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(horizon_steps=2, samples=8, best_candidate_guard=True,
               grouped_mode_sampling=False))
    boundary = np.array(
        [24, 0, contract.control_position_lower[2], 40, 0, 0],
        dtype=float)
    physical_rates = np.array([5.0, 0.0, 3.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(
            physical_rates))
    requested = np.zeros((8, 2, 3))
    requested[1:] = desired[:3]
    planner._samples = lambda rotation_lock_direction=0: requested.copy()

    plan = planner.plan(
        object(), boundary, [.001, 0, 0],
        blocked_motor_direction=np.array([0, 0, 1], dtype=np.int8))

    assert plan.valid
    assert plan.blocked_candidate_count == 7
    assert plan.selected_candidate_index == 0
    np.testing.assert_array_equal(plan.command_logical_velocity, 0.0)


def test_planner_respects_autonomous_insertion_reserve(contract):
    planner = CatheterMppi(LinearRollout(), contract, config())
    requested = np.zeros((planner.config.samples,
                          planner.config.horizon_steps, 3))
    requested[..., 0] = 10.0
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    position[0] = contract.control_position_upper[0]-0.1
    logical, _, _, _, _, _ = planner._project(
        requested, position)
    assert np.all(logical[..., 0] <= 2.0+1e-9)
    assert np.all(logical[:, 1:, 0] == 0.0)


def test_boundary_cost_does_not_create_motion_at_an_on_target_limit(contract):
    planner = CatheterMppi(LinearRollout(), contract, config())
    plan = planner.plan(
        object(), [0, 0, 7, 40, 0, 0], [0, 0, 0])
    assert plan.valid
    assert plan.command_logical_velocity == pytest.approx(np.zeros(6))


def test_zero_and_warm_start_candidates_are_always_present(contract):
    planner = CatheterMppi(LinearRollout(), contract, config(samples=8))
    planner._nominal[:] = [1.0, 2.0, 3.0]
    samples = planner._samples()
    assert samples[0] == pytest.approx(np.zeros((6, 3)))
    assert samples[1] == pytest.approx(np.tile([1.0, 2.0, 3.0], (6, 1)))


def test_causal_backlash_profile_has_signed_samples_beyond_take_up(contract):
    """The long-horizon profile must not rely on lucky Gaussian samples."""
    widths = np.array([7.29565965, 8.04862169, 9.06888263])
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(horizon_steps=24, step_s=.04, samples=12,
               noise_std=(8.0, 40.0, 4.5),
               reversal_backlash_rad=tuple(widths)))
    requested = planner._samples()
    _, _, motor_rates, _, _, _ = planner._project(
        requested, np.array([20, 0, 7, 40, 0, 0], dtype=float))
    # _samples reserves +axis probes at indices 2, 4, and 6.
    for axis, candidate in enumerate((2, 4, 6)):
        travel = np.sum(np.abs(motor_rates[candidate, :, axis]))*.04
        assert travel > widths[axis]

    # Negative signed basis probes are indices 3, 5, and 7. They initially
    # spend samples in take-up, then have nonzero achieved motion in-horizon.
    for axis, candidate in enumerate((3, 5, 7)):
        rates = motor_rates[candidate:candidate+1, :, :3]
        previous = -np.sign(rates[0, 0])
        achieved = planner._apply_rollout_backlash(rates, previous)
        assert achieved[0, 0, axis] == 0.0
        assert np.any(np.abs(achieved[0, :, axis]) > 0.0)


def test_rollout_backlash_blocks_subthreshold_reversal(contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(horizon_steps=3, samples=8,
               reversal_backlash_rad=(1.0, 0.0, 0.0)))
    rates = np.zeros((1, 3, 3))
    rates[0, :, 0] = -5.0  # 0.6 rad total travel at dt=0.04.
    achieved = planner._apply_rollout_backlash(rates, np.ones(3))
    np.testing.assert_array_equal(achieved, np.zeros_like(rates))


def test_signed_actuator_basis_candidates_are_always_present(contract):
    planner = CatheterMppi(LinearRollout(), contract, config(samples=8))

    samples = planner._samples()

    for candidate, (axis, sign) in enumerate(
            ((0, 1), (0, -1), (1, 1), (1, -1),
             (2, 1), (2, -1)), start=2):
        expected = np.zeros((6, 3))
        expected[:, axis] = sign*planner.config.noise_std[axis]
        assert samples[candidate] == pytest.approx(expected)


def test_planner_is_invariant_to_measured_takeup_state(contract):
    class RecordingRollout(LinearRollout):
        def __init__(self):
            self.rates = None

        def predict_sequence(
                self, state, motor_velocity_sequence, dt_sequence):
            self.rates = np.asarray(motor_velocity_sequence).copy()
            return super().predict_sequence(
                state, motor_velocity_sequence, dt_sequence)

    state = BacklashSnapshot(
        width_rad=np.array([0.0, 7.0, 0.0]),
        width_positive_rad=np.array([0.0, 7.0, 0.0]),
        width_negative_rad=np.array([0.0, 7.0, 0.0]),
        remaining_rad=np.array([0.0, 7.0, 0.0]),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "TAKEUP", "UNKNOWN"))
    backend_with_state = RecordingRollout()
    detached_config = config(
        samples=8, transmission_aware_rollout=True,
        rotation_direction_latch=True, takeup_risk_weight=0.0)
    planner_with_state = CatheterMppi(
        backend_with_state, contract, detached_config)
    plan_with_state = planner_with_state.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0],
        transmission_state=state,
        takeup_motor_radians_per_second=[8.0, 40.0, 4.5])
    backend_without_state = RecordingRollout()
    planner_without_state = CatheterMppi(
        backend_without_state, contract, detached_config)
    plan_without_state = planner_without_state.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0],
        transmission_state=None)

    assert plan_with_state.valid and plan_without_state.valid
    assert not plan_with_state.transmission_prediction_applied
    assert not plan_with_state.rotation_direction_latched
    assert not plan_with_state.takeup_direction_latched
    np.testing.assert_allclose(
        backend_with_state.rates, backend_without_state.rates)
    np.testing.assert_allclose(
        plan_with_state.command_logical_velocity,
        plan_without_state.command_logical_velocity)


def test_mppi_uses_effective_dynamics_and_reserves_axis2_takeup(
        contract):
    class SplitRecordingRollout(LinearRollout):
        def __init__(self):
            self.interface_rates = None
            self.raw_rates = None

        def predict_sequence(
                self, state, motor_velocity_sequence, dt_sequence, *,
                raw_motor_velocity_sequence=None):
            self.interface_rates = np.asarray(
                motor_velocity_sequence).copy()
            self.raw_rates = (
                np.asarray(motor_velocity_sequence).copy()
                if raw_motor_velocity_sequence is None else
                np.asarray(raw_motor_velocity_sequence).copy())
            return super().predict_sequence(
                state, motor_velocity_sequence, dt_sequence)

    state = BacklashSnapshot(
        width_rad=np.array([0.0, 0.0, 100.0]),
        width_positive_rad=np.array([0.0, 0.0, 100.0]),
        width_negative_rad=np.array([0.0, 0.0, 100.0]),
        remaining_rad=np.array([0.0, 0.0, 100.0]),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "TAKEUP"))
    backend = SplitRecordingRollout()
    planner = CatheterMppi(
        backend, contract,
        config(samples=8,
               raw_response_during_interface_takeup=(False, False, True)))

    direction = np.array([[0, 0, 1], [0, 0, -1]], dtype=np.int8)
    offsets, travel = planner._takeup_position_offsets(
        direction, np.array([0, 0, 1], dtype=np.int8), state)
    assert np.all(travel[:, 2] > 0.0)
    assert np.any(np.abs(offsets) > 0.0)

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0],
        transmission_state=state,
        takeup_motor_radians_per_second=[8.0, 40.0, 4.5])

    assert plan.valid
    assert not plan.transmission_prediction_applied
    np.testing.assert_allclose(
        backend.interface_rates, backend.raw_rates)


def test_mppi_update_averages_projected_not_raw_samples(contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=2, tip_weight=0.0, terminal_weight=0.0,
               slew_weight=0.0,
               boundary_weight=0.0, best_candidate_guard=False))
    raw = np.zeros((2, 6, 3))
    raw[0, :, 0] = 100.0
    raw[1, :, 0] = -20.0
    planner._samples = lambda: raw.copy()
    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0])
    # Raw averaging would request +40 mm/s and then saturate at +10. The two
    # sampled feasible controls are +10 and -10, whose MPPI update is zero.
    assert plan.command_logical_velocity == pytest.approx(np.zeros(6))
    assert planner.nominal_sequence == pytest.approx(np.zeros((6, 3)))


def test_deadline_miss_returns_six_axis_zero_and_clears_warm_start(contract):
    times = iter([0.0, 0.1, 0.2])
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=8, planning_deadline_s=0.01),
        clock=lambda: next(times))
    planner._nominal[:] = 1.0
    plan = planner.plan(object(), [20, 0, 7, 40, 0, 0], [0, 0, 0])
    assert not plan.valid
    assert plan.reason == "deadline_missed"
    assert plan.command_logical_velocity == pytest.approx(np.zeros(6))
    assert planner.nominal_sequence == pytest.approx(np.zeros((6, 3)))


def test_deadline_can_include_work_before_planner_entry(contract):
    times = iter([0.10, 0.11])
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=8, planning_deadline_s=0.05),
        clock=lambda: next(times))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0],
        deadline_started_s=0.0)

    assert not plan.valid
    assert plan.reason == "deadline_missed"
    assert plan.elapsed_s == pytest.approx(0.11)


def test_deadline_enforcement_includes_caller_commit_wait(contract):
    times = iter([0.01, 0.07, 0.08])
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=8, planning_deadline_s=0.05),
        clock=lambda: next(times))
    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0, 0, 0],
        deadline_started_s=0.0)

    checked = planner.enforce_deadline(plan, 0.0)

    assert not checked.valid
    assert checked.reason == "deadline_missed"
    assert checked.elapsed_s == pytest.approx(0.08)
    assert checked.command_logical_velocity == pytest.approx(np.zeros(6))


def test_nonfinite_rollout_fails_closed(contract):
    class NonfiniteRollout(LinearRollout):
        def predict_sequence(self, *args):
            result = super().predict_sequence(*args)
            result.tip_base_m[0, 0, 0] = torch.nan
            return result

    planner = CatheterMppi(NonfiniteRollout(), contract, config(samples=8))
    plan = planner.plan(object(), [20, 0, 7, 40, 0, 0], [0, 0, 0])
    assert not plan.valid
    assert plan.reason == "invalid_rollout"
    assert plan.command_logical_velocity == pytest.approx(np.zeros(6))


def test_uncertain_engagement_belief_penalizes_reversal(contract):
    planner = CatheterMppi(LinearRollout(), contract, config())
    snapshot = BacklashSnapshot(
        width_rad=np.array([0.0, 2.0, 0.0]),
        width_positive_rad=np.array([0.0, 2.0, 0.0]),
        width_negative_rad=np.array([0.0, 2.0, 0.0]),
        remaining_rad=np.zeros(3),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.array([1.0, 0.25, 1.0]),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("UNKNOWN", "ENGAGED", "UNKNOWN"),
        width_positive_lower_rad=np.array([0.0, 1.0, 0.0]),
        width_positive_upper_rad=np.array([0.0, 3.0, 0.0]),
        width_negative_lower_rad=np.array([0.0, 0.5, 0.0]),
        width_negative_upper_rad=np.array([0.0, 4.0, 0.0]))
    directions = np.array([[0, 1, 0], [0, -1, 0]], dtype=np.int8)
    extra = planner._takeup_low_confidence_extra_travel(
        directions, snapshot)
    assert extra[0] == pytest.approx(np.zeros(3))
    assert extra[1, 1] == pytest.approx(0.75*(4.0-0.5))
    assert np.count_nonzero(extra[1]) == 1


class GainScenarioRollout:
    def __init__(self):
        self.last_gain = None

    def predict_control_sequence(self, state, motor_velocity_sequence,
                                 dt_sequence, *, distal_gain=None):
        rates = torch.as_tensor(motor_velocity_sequence, dtype=torch.float32)
        gain = torch.as_tensor(distal_gain, dtype=torch.float32)
        self.last_gain = gain.detach().cpu().numpy().copy()
        displacement = torch.cumsum(rates*float(dt_sequence), dim=1)
        tips = torch.zeros(*rates.shape[:-1], 3)
        tips[..., 0] = -1e-3*gain[:, None]*displacement[..., 2]
        distal_lambda = gain[:, None]*displacement[..., 2]
        return SimpleNamespace(
            tip_base_m=tips, markers_base_m=None,
            distal_lambda=distal_lambda)


def test_engaged_gain_scenarios_are_complete_rollouts(contract):
    backend = GainScenarioRollout()
    planner = CatheterMppi(
        backend, contract,
        config(samples=32, horizon_steps=4, engaged_gain_scenarios=True,
               engaged_gain_risk_beta=0.5))
    gain = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, prior_mean=(0.6, 0.8), prior_log_std=0.25)).snapshot()
    transmission = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, 1, 1], dtype=np.int8),
        engaged_direction=np.array([1, 1, 1], dtype=np.int8),
        confidence=np.ones(3), confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"), engaged_gain=gain)

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.002, 0, 0],
        transmission_state=transmission)

    assert plan.valid
    assert plan.engaged_gain_scenario_count == 3
    assert backend.last_gain.shape == (
        (32+plan.tendon_probe_candidate_count)*3,)
    assert plan.selected_engaged_gain_scenarios.shape == (3,)
    assert plan.selected_gain_tracking_costs.shape == (3,)
    assert plan.best_tip_sequence_m.shape == (4, 3)


def test_engaged_gain_first_step_cap_blocks_aggressive_candidates(contract):
    backend = GainScenarioRollout()
    planner = CatheterMppi(
        backend, contract,
        config(samples=32, horizon_steps=4, engaged_gain_scenarios=True,
               engaged_gain_maximum_first_step_shift=1e-4))
    gain = EngagedGainEstimator(EngagedGainConfig(
        enabled=True, prior_mean=(1.0, 1.0), prior_log_std=0.5)).snapshot()
    transmission = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, 1, 1], dtype=np.int8),
        engaged_direction=np.array([1, 1, 1], dtype=np.int8),
        confidence=np.ones(3), confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"), engaged_gain=gain)
    root = SimpleNamespace(lambda_value=torch.tensor(0.0))

    plan = planner.plan(
        root, [20, 0, 7, 40, 0, 0], [0.002, 0, 0],
        transmission_state=transmission)

    assert plan.valid
    assert plan.blocked_candidate_count > 0
    assert (plan.selected_maximum_first_step_lambda_shift
            <= planner.config.engaged_gain_maximum_first_step_shift)


def test_point_capture_hold_is_scored_and_persists(contract):
    class Clock:
        value = 0.0

        def __call__(self):
            return self.value

    clock = Clock()
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=64, horizon_steps=4, capture_radius_mm=2.5,
               capture_minimum_terminal_improvement_mm=100.0,
               capture_hold_s=0.30), clock=clock)

    captured = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.001, 0, 0])
    assert captured.valid
    assert captured.hold_branch_applied
    assert captured.selected_candidate_index == 0
    assert captured.command_logical_velocity == pytest.approx(np.zeros(6))
    assert planner.point_capture_hold_active()

    clock.value = 0.10
    held = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.010, 0, 0])
    assert held.hold_branch_applied
    assert held.command_logical_velocity == pytest.approx(np.zeros(6))

    clock.value = 0.31
    assert not planner.point_capture_hold_active()
    released = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.010, 0, 0])
    assert released.valid
    assert not released.hold_branch_applied
    assert released.selected_candidate_index != 0



def test_capture_response_shortfall_releases_hold_and_reduces_estimate(
        contract):
    class Clock:
        value = 0.0

        def __call__(self):
            return self.value

    clock = Clock()
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=64, horizon_steps=4, capture_radius_mm=2.5,
               capture_minimum_terminal_improvement_mm=100.0,
               capture_hold_s=0.30,
               capture_response_minimum_prediction_mm=0.25,
               capture_response_minimum_ratio=0.50), clock=clock)
    captured = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.001, 0, 0],
        observed_tip_base_m=[0.0, 0.0, 0.0])

    assert captured.hold_branch_applied
    released = planner.observe_capture_response(
        [1.0, 0.0, 0.0], [0.1, 0.0, 0.0],
        predicted_progress_mm=0.8, measured_progress_mm=0.05)

    assert released
    assert not planner.point_capture_hold_active()
    diagnostics = planner.capture_diagnostics()
    assert diagnostics["passive_response_scale"] == pytest.approx(0.1)
    assert diagnostics["last_response_ratio"] == pytest.approx(0.0625)
    assert diagnostics["last_response_reason"] == "response_shortfall_release"
    assert diagnostics["release_count"] == 1
    assert diagnostics["rearm_blocked"]

    correction = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.001, 0, 0],
        observed_tip_base_m=[0.0, 0.0, 0.0])
    assert correction.valid
    assert not correction.hold_branch_applied
    assert correction.selected_candidate_index != 0

    planner.note_capture_control_executed(
        correction.command_logical_velocity)
    assert not planner.capture_diagnostics()["rearm_blocked"]


def test_capture_response_consistent_with_forecast_keeps_bounded_hold(contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(samples=64, horizon_steps=4, capture_radius_mm=2.5,
               capture_minimum_terminal_improvement_mm=100.0,
               capture_hold_s=0.30,
               capture_response_minimum_prediction_mm=0.25,
               capture_response_minimum_ratio=0.50))
    planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.001, 0, 0],
        observed_tip_base_m=[0.0, 0.0, 0.0])

    released = planner.observe_capture_response(
        [1.0, 0.0, 0.0], [0.8, 0.0, 0.0],
        predicted_progress_mm=0.8, measured_progress_mm=0.6)

    assert not released
    assert planner.point_capture_hold_active()
    diagnostics = planner.capture_diagnostics()
    assert diagnostics["passive_response_scale"] == pytest.approx(1.0)
    assert diagnostics["last_response_reason"] == "response_consistent"

def test_uncertain_engaged_gain_scales_tendon_candidates_before_rollout(
        contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(engaged_gain_learning_velocity_scale=0.5))
    belief = EngagedGainEstimator(EngagedGainConfig(
        enabled=True)).snapshot()
    transmission = SimpleNamespace(
        engaged_direction=np.array([0, 0, 1], dtype=np.int8),
        engaged_gain=belief)
    directions = np.array([[0, 0, 1], [0, 0, -1], [0, 0, 0]],
                          dtype=np.int8)

    scales = planner._gain_learning_velocity_scales(
        directions, transmission)

    assert scales == pytest.approx([0.5, 0.5, 1.0])


class DelayedTendonRollout:
    """Respond to positive physical tendon only after 200 ms."""

    def __init__(self):
        self.last_dt = None
        self.last_shape = None

    def predict_control_sequence(self, state, motor_velocity_sequence,
                                 dt_sequence, **kwargs):
        rates = torch.as_tensor(motor_velocity_sequence, dtype=torch.float32)
        dt = torch.as_tensor(dt_sequence, dtype=torch.float32)
        if dt.ndim == 0:
            dt = dt.repeat(rates.shape[1])
        self.last_dt = dt.detach().cpu().numpy().copy()
        self.last_shape = tuple(rates.shape)
        elapsed = torch.cumsum(dt, dim=0)
        active = (elapsed > 0.20).to(rates)
        positive_tendon = torch.relu(rates[..., 2])
        response = torch.cumsum(
            positive_tendon*dt[None, :]*active[None, :], dim=1)
        tips = torch.zeros(*rates.shape[:-1], 3)
        tips[..., 0] = 4e-4*response
        return SimpleNamespace(tip_base_m=tips, markers_base_m=None)

    def predict_control_sequence_coarse(
            self, state, motor_velocity_sequence, dt_sequence, **kwargs):
        self.coarse_called = True
        return self.predict_control_sequence(
            state, motor_velocity_sequence, dt_sequence, **kwargs)



def test_point_rollout_uses_exactly_four_coarse_steps_over_08_s(contract):
    backend = DelayedTendonRollout()
    backend.coarse_called = False
    planner = CatheterMppi(
        backend, contract,
        config(horizon_steps=4, step_s=0.04, samples=16,
               point_rollout_step_s=0.20,
               point_rollout_coarse_steps=True,
               point_prediction_tail_steps=0))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.004, 0, 0])

    assert plan.valid
    assert backend.coarse_called
    assert backend.last_shape[1] == 4
    assert backend.last_dt == pytest.approx([0.20]*4)
    assert plan.prediction_horizon_steps == 4
    assert plan.prediction_horizon_s == pytest.approx(0.80)


def test_point_prediction_tail_values_delayed_tendon_without_expanding_command(
        contract):
    short_backend = DelayedTendonRollout()
    short = CatheterMppi(
        short_backend, contract,
        config(horizon_steps=4, step_s=0.04, samples=16,
               point_prediction_tail_steps=0, slew_weight=0.0,
               boundary_weight=0.0, takeup_risk_weight=0.0))
    short_plan = short.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.004, 0, 0])

    tail_backend = DelayedTendonRollout()
    tail = CatheterMppi(
        tail_backend, contract,
        config(horizon_steps=4, step_s=0.04, samples=16,
               point_prediction_tail_steps=7,
               point_prediction_tail_step_s=0.12, slew_weight=0.0,
               boundary_weight=0.0, takeup_risk_weight=0.0))
    tail_plan = tail.plan(
        object(), [20, 0, 7, 40, 0, 0], [0.004, 0, 0])

    assert short_plan.valid and tail_plan.valid
    assert short_plan.selected_candidate_index == 0
    assert tail_plan.selected_candidate_index != 0
    assert tail_plan.command_logical_velocity[2] != 0.0
    assert tail_plan.logical_velocity_sequence.shape == (4, 6)
    assert tail_plan.motor_radians_per_second_sequence.shape == (4, 6)
    assert tail_plan.best_tip_sequence_m.shape == (4, 3)
    assert tail_backend.last_shape[1] == 11
    assert tail_backend.last_dt == pytest.approx(
        [0.04]*4+[0.12]*7)
    assert tail_plan.prediction_horizon_steps == 11
    assert tail_plan.prediction_horizon_s == pytest.approx(1.0)


def test_continuous_path_reference_does_not_use_point_prediction_tail(contract):
    backend = DelayedTendonRollout()
    backend.coarse_called = False
    planner = CatheterMppi(
        backend, contract,
        config(horizon_steps=4, samples=16,
               point_rollout_step_s=0.20,
               point_rollout_coarse_steps=True,
               point_prediction_tail_steps=7,
               point_prediction_tail_step_s=0.12))
    tangent = np.tile([1.0, 0.0, 0.0], (4, 1))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0],
        np.zeros((4, 3)), target_tangent_base=tangent)

    assert plan.valid
    assert not backend.coarse_called
    assert backend.last_shape[1] == 4
    assert backend.last_dt == pytest.approx([0.04]*4)
    assert plan.prediction_horizon_steps == 4
    assert plan.prediction_horizon_s == pytest.approx(0.16)


def test_continuous_path_can_use_exactly_four_coarse_steps_over_08_s(contract):
    backend = DelayedTendonRollout()
    backend.coarse_called = False
    planner = CatheterMppi(
        backend, contract,
        config(horizon_steps=4, step_s=0.20, samples=16,
               path_rollout_coarse_steps=True,
               point_prediction_tail_steps=7))
    tangent = np.tile([1.0, 0.0, 0.0], (4, 1))

    plan = planner.plan(
        object(), [20, 0, 7, 40, 0, 0],
        np.zeros((4, 3)), target_tangent_base=tangent)

    assert plan.valid
    assert backend.coarse_called
    assert backend.last_shape[1] == 4
    assert backend.last_dt == pytest.approx([0.20]*4)
    assert plan.prediction_horizon_steps == 4
    assert plan.prediction_horizon_s == pytest.approx(0.80)


def test_each_grouped_proposal_has_compatible_sustained_tendon_probe(contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(horizon_steps=3, samples=16,
               engaged_gain_learning_velocity_scale=0.5,
               grouped_mode_sampling=True))
    lease = np.array([1, 0, 1], dtype=np.int8)

    grouped, labels, group_count = planner._grouped_candidates(lease)
    probes = planner._last_tendon_probe_mask
    physical = grouped.copy()
    physical[..., 0] -= physical[..., 2]
    physical /= MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[
        None, None, :CONTROL_AXES]

    assert group_count == 4
    assert np.count_nonzero(probes) == 6
    for label in np.unique(labels):
        member = (labels == label) & probes
        assert np.any(member)
        directions = np.sign(physical[member, 0, 2]).astype(int)
        assert np.all(
            np.sign(physical[member, :, 2]).astype(int)
            == directions[:, None])
        if label & (1 << 2):
            assert set(directions) == {1}
        else:
            assert set(directions) == {-1, 1}


def test_prediction_tail_projection_respects_joint_endpoint(contract):
    planner = CatheterMppi(
        LinearRollout(), contract,
        config(horizon_steps=1, samples=8))
    requested = np.zeros((1, 3, 3))
    requested[..., 0] = 8.0
    root = np.array([
        contract.control_position_upper[0]-0.10, 0, 7, 40, 0, 0])
    dt = np.array([0.04, 0.12, 0.12])

    _, realized, _, projection_cost, _, _ = planner._project(
        requested, root, dt_sequence=dt)
    endpoint = root[None]+np.sum(realized*dt[None, :, None], axis=1)

    assert endpoint[0, 0] <= contract.control_position_upper[0]+1e-9
    assert projection_cost[0] > 0.0
