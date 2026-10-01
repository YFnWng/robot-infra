"""Minimal catheter-specific MPPI planner for the Phase-2 streaming model.

This module only plans.  It has no ROS publisher, serial transport, or encoder
zero operation.  The Phase-4 node will own arming and command heartbeats.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from itertools import product
from threading import Lock
from time import monotonic, perf_counter
from typing import Any, Callable

import numpy as np
import torch

from .hardware_contract import (
    HardwareContract, MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM, N_AXES)


CONTROL_AXES = 3


def _finite_vector(name: str, value, size: int) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _synchronize_torch_device(device: torch.device) -> None:
    """Make asynchronous accelerator work visible to timers and deadlines."""
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def _tip_tracking_error_squared(
        tips: torch.Tensor, target: torch.Tensor,
        target_tangent: torch.Tensor | None) -> torch.Tensor:
    """Squared point error or forward-path-corridor error in millimetres."""
    target_view = target.reshape(
        (1,)*(tips.ndim-target.ndim)+tuple(target.shape))
    error_mm = (tips-target_view)*1000.0
    if target_tangent is None:
        return error_mm.square().sum(-1)
    along_mm = torch.sum(error_mm*target_tangent[None], dim=-1)
    cross_mm = error_mm-along_mm[..., None]*target_tangent[None]
    lag_mm = torch.relu(-along_mm)
    return cross_mm.square().sum(-1)+lag_mm.square()


@dataclass(frozen=True)
class MppiConfig:
    """Small defaults intended for a low-latency camera-loop demo."""

    horizon_steps: int = 4
    step_s: float = 0.040
    # Optional point-target-only integration grid. A positive value replaces
    # step_s for point rollouts but not for path tracking. Coarse mode requires
    # a backend route that performs exactly one model update per control step.
    point_rollout_step_s: float = 0.0
    point_rollout_coarse_steps: bool = False
    # Continuous-path analogue of point_rollout_coarse_steps. When enabled,
    # each MPPI control interval is one backend model update; the backend must
    # not subdivide step_s into its native fine integration interval.
    path_rollout_coarse_steps: bool = False

    # Point targets can require more model time than the short optimized
    # sequence exposes. The tail is prediction-only: it holds the last
    # optimized control, is projected through the same joint limits, and is
    # never returned as an executable command sequence. Path tracking keeps
    # its supplied finite reference horizon and therefore does not use it.
    point_prediction_tail_steps: int = 0
    point_prediction_tail_step_s: float = 0.120
    samples: int = 32
    temperature: float = 1.0
    noise_std: tuple[float, float, float] = (4.0, 20.0, 2.0)
    noise_correlation: float = 0.65
    exploration_fraction: float = 0.15
    reversal_backlash_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    tip_weight: float = 1.0
    terminal_weight: float = 6.0
    slew_weight: float = 0.04
    takeup_risk_weight: float = 4.0
    takeup_confirmation_time_s: float = 0.10
    # Robust engaged-response scenarios are evaluated as complete rollouts.
    # Scenario zero is the posterior mean; lower/upper credible gains are used
    # only in the risk aggregate and never to edit a selected control plan.
    engaged_gain_scenarios: bool = False
    engaged_gain_risk_beta: float = 0.50
    engaged_gain_cvar_alpha: float = 0.67
    engaged_gain_maximum_first_step_shift: float = 0.0
    engaged_gain_learning_velocity_scale: float = 1.0
    # Near a point target, require a sampled nonzero plan to improve the
    # predicted terminal error by this margin over deterministic hold. Once
    # hold wins, keep it active long enough to observe delayed plant motion.
    capture_radius_mm: float = 0.0
    capture_minimum_terminal_improvement_mm: float = 0.0
    # A capture hold is a bounded experiment, not a model-only latch. Compare
    # its horizon-aligned measured response with the zero forecast. A weak
    # response releases hold and reduces the passive-motion scale used by
    # subsequent capture decisions.
    capture_response_minimum_prediction_mm: float = 0.25
    capture_response_minimum_ratio: float = 0.50
    capture_hold_s: float = 0.0
    # True means the raw shaft still has a modeled response while the visible
    # interface coordinate is inside its play.  The deployed v175/v171 split
    # uses this for motor 2: raw motion drives frozen v171 tendon history while
    # v175 play filters only the interface-Jacobian input.
    raw_response_during_interface_takeup: tuple[bool, bool, bool] = (
        False, False, False)
    transmission_aware_rollout: bool = False
    rotation_direction_latch: bool = False
    best_candidate_guard: bool = True
    grouped_mode_sampling: bool = True
    takeup_limit_reserve_scale: float = 1.0
    boundary_weight: float = 0.10
    boundary_margin_fraction: float = 0.10
    planning_deadline_s: float = 0.060
    seed: int = 0

    def __post_init__(self) -> None:
        if self.horizon_steps < 1 or self.samples < 2:
            raise ValueError("MPPI needs at least one step and two samples")
        if (not np.isfinite(self.point_rollout_step_s)
                or self.point_rollout_step_s < 0.0):
            raise ValueError("point_rollout_step_s must be nonnegative")
        if self.point_prediction_tail_steps < 0:
            raise ValueError(
                "point_prediction_tail_steps must be nonnegative")
        positive = (self.step_s, self.temperature, self.planning_deadline_s,
                    self.boundary_margin_fraction,
                    self.takeup_limit_reserve_scale,
                    self.point_prediction_tail_step_s)
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError(
                "step, temperature, boundary margin, and deadline must be "
                "positive")
        noise = np.asarray(self.noise_std, dtype=np.float64)
        if noise.shape != (CONTROL_AXES,) or np.any(noise <= 0.0):
            raise ValueError("noise_std must contain three positive values")
        if not 0.0 <= self.noise_correlation < 1.0:
            raise ValueError("noise_correlation must be in [0,1)")
        if not 0.0 <= self.exploration_fraction <= 1.0:
            raise ValueError("exploration_fraction must be in [0,1]")
        backlash = np.asarray(self.reversal_backlash_rad, dtype=np.float64)
        if (backlash.shape != (CONTROL_AXES,)
                or not np.isfinite(backlash).all()
                or np.any(backlash < 0.0)):
            raise ValueError(
                "reversal_backlash_rad must contain three nonnegative values")
        raw_response = np.asarray(
            self.raw_response_during_interface_takeup, dtype=bool)
        if raw_response.shape != (CONTROL_AXES,):
            raise ValueError(
                "raw_response_during_interface_takeup must contain three "
                "booleans")
        if (not np.isfinite(self.takeup_confirmation_time_s)
                or self.takeup_confirmation_time_s < 0.0):
            raise ValueError(
                "takeup_confirmation_time_s must be finite and nonnegative")
        if (not np.isfinite(self.engaged_gain_maximum_first_step_shift)
                or self.engaged_gain_maximum_first_step_shift < 0.0):
            raise ValueError(
                "engaged_gain_maximum_first_step_shift must be nonnegative")
        if (not np.isfinite(self.engaged_gain_learning_velocity_scale)
                or not 0.0 < self.engaged_gain_learning_velocity_scale <= 1.0):
            raise ValueError(
                "engaged_gain_learning_velocity_scale must be in (0,1]")
        capture = (self.capture_radius_mm,
                   self.capture_minimum_terminal_improvement_mm,
                   self.capture_hold_s,
                   self.capture_response_minimum_prediction_mm)
        if any(not np.isfinite(value) or value < 0.0 for value in capture):
            raise ValueError("capture parameters must be nonnegative")
        if (not np.isfinite(self.capture_response_minimum_ratio)
                or not 0.0 <= self.capture_response_minimum_ratio <= 1.0):
            raise ValueError(
                "capture_response_minimum_ratio must be in [0,1]")
        if (not np.isfinite(self.engaged_gain_risk_beta)
                or self.engaged_gain_risk_beta < 0.0):
            raise ValueError(
                "engaged_gain_risk_beta must be finite and nonnegative")
        if (not np.isfinite(self.engaged_gain_cvar_alpha)
                or not 0.0 <= self.engaged_gain_cvar_alpha < 1.0):
            raise ValueError(
                "engaged_gain_cvar_alpha must be in [0,1)")
        weights = (self.tip_weight, self.terminal_weight, self.slew_weight,
                   self.takeup_risk_weight, self.boundary_weight)
        if any(not np.isfinite(value) or value < 0.0 for value in weights):
            raise ValueError("cost weights must be finite and nonnegative")


@dataclass(frozen=True)
class MppiPlan:
    """One planner result; only ``command_logical_velocity`` is executable."""

    command_logical_velocity: np.ndarray
    logical_velocity_sequence: np.ndarray
    motor_radians_per_second_sequence: np.ndarray
    best_tip_sequence_m: np.ndarray | None
    best_cost: float
    effective_samples: float
    elapsed_s: float
    valid: bool
    reason: str
    sample_projection_ms: float = 0.0
    rollout_ms: float = 0.0
    cost_weighting_ms: float = 0.0
    update_projection_ms: float = 0.0
    command_tip_sequence_m: np.ndarray | None = None
    compensated_motor_radians_per_second_sequence: np.ndarray | None = None
    transmitted_motor_radians_per_second_sequence: np.ndarray | None = None
    transmission_prediction_applied: bool = False
    rotation_direction_latched: bool = False
    takeup_direction_latched: bool = False
    best_candidate_selected: bool = False
    weighted_tracking_cost: float = float("nan")
    zero_tracking_cost: float = float("nan")
    best_tracking_cost: float = float("nan")
    selected_candidate_index: int = -1
    selected_total_cost: float = float("nan")
    zero_total_cost: float = float("nan")
    scored_candidate_guard_applied: bool = False
    blocked_motor_direction: np.ndarray | None = None
    blocked_candidate_count: int = 0
    direction_lease: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.int8))
    approved_reversal_direction: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.int8))
    proposed_reversal_direction: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.int8))
    direction_lease_applied: bool = False
    unrestricted_candidate_index: int = -1
    lease_constrained_candidate_index: int = -1
    unrestricted_total_cost: float = float("nan")
    lease_constrained_total_cost: float = float("nan")
    reversal_axis_cost_improvement: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.float64))
    reversal_axis_terminal_error_improvement_mm: np.ndarray = field(
        default_factory=lambda: np.zeros(CONTROL_AXES, dtype=np.float64))
    hold_branch_applied: bool = False
    hold_branch_terminal_error_mm: float = float("nan")
    zero_terminal_error_mm: float = float("nan")
    raw_zero_terminal_error_mm: float = float("nan")
    capture_passive_response_scale: float = 1.0
    proposal_group_count: int = 1
    selected_reversal_mask: int = 0
    unrestricted_reversal_mask: int = 0
    mode_best_total_cost: np.ndarray = field(
        default_factory=lambda: np.full(1 << CONTROL_AXES, np.inf))
    takeup_joint_position_offset: np.ndarray = field(
        default_factory=lambda: np.zeros(N_AXES, dtype=np.float64))
    selected_takeup_risk_s: float = 0.0
    selected_takeup_risk_cost: float = 0.0
    selected_switch_count: int = 0
    engaged_gain_scenario_count: int = 1
    selected_engaged_gain_scenarios: np.ndarray = field(
        default_factory=lambda: np.ones(1, dtype=np.float64))
    selected_gain_tracking_costs: np.ndarray = field(
        default_factory=lambda: np.full(1, np.nan, dtype=np.float64))
    selected_maximum_first_step_lambda_shift: float = 0.0
    selected_gain_learning_velocity_scale: float = 1.0
    prediction_horizon_steps: int = 0
    prediction_horizon_s: float = 0.0
    tendon_probe_candidate_count: int = 0
    selected_tendon_probe: bool = False


@dataclass(frozen=True)
class SparseTargetPrediction:
    """Read-only forward-model preview for guarded sparse experiments."""

    target_tip_m: np.ndarray
    predicted_tip_displacement_m: np.ndarray
    realized_logical_displacement: np.ndarray
    endpoint_joint_position: np.ndarray
    takeup_joint_position_offset: np.ndarray


class CatheterMppi:
    """Batched MPPI over three catheter joints with six-axis safety projection.

    ``rollout_backend`` must provide ``predict_sequence(state, motor_rates,
    dt_sequence)`` with the interface of ``StreamingCatheterRuntime``.
    Logical commands use the production manager units: mm/s, deg/s, mm/s.
    """

    def __init__(self, rollout_backend: Any, contract: HardwareContract,
                 config: MppiConfig | None = None,
                 *, clock: Callable[[], float] = monotonic):
        self.backend = rollout_backend
        self.contract = contract
        self.config = config or MppiConfig()
        self._clock = clock
        self._rng = np.random.default_rng(self.config.seed)
        self._nominal = np.zeros(
            (self.config.horizon_steps, CONTROL_AXES), dtype=np.float64)
        self._capture_lock = Lock()
        self._capture_hold_until_s = float("-inf")
        self._capture_rearm_blocked = False
        self._capture_passive_response_scale = 1.0
        self._capture_last_response_ratio = float("nan")
        self._capture_last_response_reason = "unobserved"
        self._capture_release_count = 0
        self._last_tendon_probe_mask = np.zeros(
            self.config.samples, dtype=bool)

    def reset(self) -> None:
        """Clear warm-start state without touching hardware or model state."""
        self._nominal.fill(0.0)
        with self._capture_lock:
            self._capture_hold_until_s = float("-inf")
            self._capture_rearm_blocked = False

    def reset_capture_target(self) -> None:
        """End target-local hold state while retaining learned passive scale."""
        with self._capture_lock:
            self._capture_hold_until_s = float("-inf")
            self._capture_rearm_blocked = False

    @property
    def capture_passive_response_scale(self) -> float:
        with self._capture_lock:
            return float(self._capture_passive_response_scale)

    def capture_diagnostics(self) -> dict[str, object]:
        with self._capture_lock:
            return {
                "passive_response_scale": float(
                    self._capture_passive_response_scale),
                "last_response_ratio": float(
                    self._capture_last_response_ratio),
                "last_response_reason": self._capture_last_response_reason,
                "release_count": int(self._capture_release_count),
                "rearm_blocked": bool(self._capture_rearm_blocked),
            }

    def observe_capture_response(
            self, predicted_delta_mm, measured_delta_mm, *,
            predicted_progress_mm: float | None = None,
            measured_progress_mm: float | None = None) -> bool:
        """Update passive-motion belief and release a disproven hold.

        Returns true only when the observation disproves the active capture
        hypothesis. Directional displacement and target-directed progress are
        both ratios, so lateral motion or motion away from the target cannot
        masquerade as the predicted passive approach.
        """
        predicted = _finite_vector(
            "predicted_delta_mm", predicted_delta_mm, 3)
        measured = _finite_vector("measured_delta_mm", measured_delta_mm, 3)
        predicted_norm = float(np.linalg.vector_norm(predicted))
        minimum = self.config.capture_response_minimum_prediction_mm
        ratios = []
        if predicted_norm >= minimum:
            projected = float(np.dot(measured, predicted))/max(
                predicted_norm*predicted_norm, 1e-12)
            ratios.append(max(0.0, projected))
        if (predicted_progress_mm is not None
                and measured_progress_mm is not None):
            predicted_progress = float(predicted_progress_mm)
            measured_progress = float(measured_progress_mm)
            if (np.isfinite(predicted_progress)
                    and np.isfinite(measured_progress)
                    and predicted_progress >= minimum):
                ratios.append(max(0.0, measured_progress/predicted_progress))
        if not ratios:
            with self._capture_lock:
                self._capture_last_response_reason = (
                    "prediction_below_evidence_floor")
            return False
        ratio = min(ratios)
        with self._capture_lock:
            self._capture_last_response_ratio = ratio
            if ratio >= self.config.capture_response_minimum_ratio:
                self._capture_last_response_reason = "response_consistent"
                self._capture_rearm_blocked = False
                return False
            # This estimate is deliberately one-sided and conservative. A
            # disproven passive approach may reduce future reliance on the
            # zero forecast; model prediction alone cannot increase it.
            self._capture_passive_response_scale = min(
                self._capture_passive_response_scale,
                max(0.1, ratio))
            self._capture_hold_until_s = float("-inf")
            self._capture_rearm_blocked = True
            self._capture_release_count += 1
            self._capture_last_response_reason = "response_shortfall_release"
            return True

    def note_capture_control_executed(self, command_logical_velocity) -> None:
        """Permit later capture only after useful nonzero control executes."""
        command = _finite_vector(
            "command_logical_velocity", command_logical_velocity, N_AXES)
        if np.any(np.abs(command[:CONTROL_AXES]) > 1e-12):
            with self._capture_lock:
                self._capture_rearm_blocked = False

    def point_capture_hold_active(self) -> bool:
        """Return whether a previously selected point hold is still active.

        The ROS wrapper uses this query to heartbeat zero without launching a
        redundant GPU rollout. A full rollout is still required to select the
        hold initially and again after the bounded hold expires.
        """
        with self._capture_lock:
            return bool(
                self.config.capture_radius_mm > 0.0
                and self.config.capture_hold_s > 0.0
                and self._clock() < self._capture_hold_until_s)

    def predict_sparse_targets(
            self, root_state: Any, joint_position, observed_tip_m,
            logical_displacements, rollout_steps: int,
            minimum_endpoint_reserve, *, transmission_state=None
            ) -> SparseTargetPrediction:
        """Preview model-generated targets without changing planner state.

        Inputs are post-engagement logical displacements in controller units.
        Candidate projection reserves the conservative upper take-up
        bound before integrating useful motion. Rollouts are exclusively in
        effective post-engagement coordinates; no immediate-engagement or
        partial-take-up response is synthesized inside MPPI.
        """
        measured_position = _finite_vector(
            "joint_position", joint_position, N_AXES)
        # The manager accepts a small configured feedback excursion beyond an
        # exact hard boundary to accommodate encoder quantization.  Target
        # preview must use the same qualification rule, then project from the
        # exact command boundary.  Otherwise an unchanged inactive axis can
        # reject every otherwise-safe candidate (for example bend=-0.0013 mm
        # at its 0 mm lower limit).
        position = self.contract.resolve_feedback_position(measured_position)
        observed = _finite_vector("observed_tip_m", observed_tip_m, 3)
        reserve = _finite_vector(
            "minimum_endpoint_reserve", minimum_endpoint_reserve, N_AXES)
        if np.any(reserve < 0.0):
            raise ValueError("minimum endpoint reserve must be nonnegative")
        displacement = np.asarray(logical_displacements, dtype=np.float64)
        if (displacement.ndim != 2
                or displacement.shape[1:] != (CONTROL_AXES,)
                or not 1 <= len(displacement) <= 16
                or not np.isfinite(displacement).all()):
            raise ValueError(
                "logical displacements must have shape (1..16,3)")
        if np.any(np.linalg.norm(displacement, axis=1) <= 0.0):
            raise ValueError("logical displacements must be nonzero")
        if self.contract.velocity_max[1] != 0.0:
            raise ValueError("sparse no-rotation preview requires axis 1 disabled")
        if np.any(displacement[:, 1] != 0.0):
            raise ValueError("sparse no-rotation preview rejects rotation")
        steps = int(rollout_steps)
        if steps < 1 or steps > 250:
            raise ValueError("rollout_steps must be in [1,250]")
        duration_s = steps*self.config.step_s
        requested = np.broadcast_to(
            displacement[:, None, :]/duration_s,
            (len(displacement), steps, CONTROL_AXES)).copy()
        first_direction, internal_reversal = self._first_physical_direction(
            requested)
        if np.any(internal_reversal):
            raise ValueError("constant sparse preview unexpectedly reverses")
        takeup_offset, _ = self._takeup_position_offsets(
            first_direction, np.zeros(CONTROL_AXES, dtype=np.int8),
            transmission_state)
        (logical, realized, motor_rates, _, _, blocked) = self._project(
            requested, position,
            np.zeros(CONTROL_AXES, dtype=np.int8), takeup_offset)
        if np.any(blocked):
            indices = np.flatnonzero(blocked).tolist()
            raise ValueError(
                f"sparse candidates blocked by projected limits: {indices}")
        if (np.any(logical[..., 1] != 0.0)
                or np.any(realized[..., 1] != 0.0)
                or np.any(motor_rates[..., 1] != 0.0)):
            raise ValueError("projection produced nonzero rotation")

        endpoint = (
            position[None]+takeup_offset
            + np.sum(realized, axis=1)*self.config.step_s)
        outside_hard = (
            (endpoint < self.contract.position_lower[None])
            | (endpoint > self.contract.position_upper[None]))
        travel = (
            takeup_offset
            + np.sum(realized, axis=1)*self.config.step_s)
        active_axis = np.abs(travel) > 1.0e-12
        reserve_lower = self.contract.control_position_lower+reserve
        reserve_upper = self.contract.control_position_upper-reserve
        outside_reserve = active_axis & (
            (endpoint < reserve_lower[None])
            | (endpoint > reserve_upper[None]))
        outside = np.any(outside_hard | outside_reserve, axis=1)
        if np.any(outside):
            details = []
            for candidate in np.flatnonzero(outside):
                axes = np.flatnonzero(
                    outside_hard[candidate] | outside_reserve[candidate])
                details.append(
                    f"{candidate}:axes={axes.tolist()},"
                    f"endpoint={np.round(endpoint[candidate], 4).tolist()}")
            raise ValueError(
                "sparse candidate endpoint lacks joint reserve: "
                + ";".join(details))

        raw_motor_rates = motor_rates[..., :CONTROL_AXES]
        interface_motor_rates = raw_motor_rates
        transmission_prediction_applied = False
        predict = getattr(
            self.backend, "predict_control_sequence",
            self.backend.predict_sequence)
        kwargs = ({"raw_motor_velocity_sequence": raw_motor_rates}
                  if transmission_prediction_applied else {})
        prediction = predict(
            root_state, interface_motor_rates, self.config.step_s, **kwargs)
        zero_rates = np.zeros_like(raw_motor_rates)
        baseline = predict(root_state, zero_rates, self.config.step_s)
        tips = torch.as_tensor(prediction.tip_base_m)
        zero_tips = torch.as_tensor(baseline.tip_base_m)
        _synchronize_torch_device(tips.device)
        delta = (tips[..., -1, :]-zero_tips[..., -1, :]).detach().cpu().numpy()
        targets = observed[None]+delta
        realized_displacement = np.sum(
            realized, axis=1)*self.config.step_s
        if (not np.isfinite(targets).all()
                or not np.isfinite(realized_displacement).all()):
            raise ValueError("sparse target preview produced nonfinite output")
        if len(np.unique(np.round(targets, decimals=10), axis=0)) != len(
                targets):
            raise ValueError("sparse target preview produced duplicate targets")
        return SparseTargetPrediction(
            target_tip_m=targets,
            predicted_tip_displacement_m=delta,
            realized_logical_displacement=realized_displacement,
            endpoint_joint_position=endpoint,
            takeup_joint_position_offset=takeup_offset)

    @property
    def nominal_sequence(self) -> np.ndarray:
        return self._nominal.copy()

    def _samples(self, rotation_lock_direction: int = 0) -> np.ndarray:
        cfg = self.config
        noise = self._rng.standard_normal(
            (cfg.samples, cfg.horizon_steps, CONTROL_AXES))
        scale = np.sqrt(1.0 - cfg.noise_correlation**2)
        for step in range(1, cfg.horizon_steps):
            noise[:, step] = (cfg.noise_correlation*noise[:, step-1]
                              + scale*noise[:, step])
        noise *= np.asarray(cfg.noise_std)[None, None, :]
        requested = self._nominal[None] + noise
        explore = int(round(cfg.samples*cfg.exploration_fraction))
        if explore:
            requested[-explore:] = noise[-explore:]
        # These deterministic candidates make hold and warm-start observable.
        requested[0] = 0.0
        requested[1] = self._nominal
        # Reserve constant signed actuator-basis probes when the sample budget
        # permits it. They guarantee that insertion, rotation, and bending are
        # represented independently even if a small random batch is unlucky.
        if cfg.samples >= 8:
            basis_scale = np.asarray(cfg.noise_std, dtype=np.float64)
            for candidate, (axis, sign) in enumerate(
                    ((0, 1.0), (0, -1.0),
                     (1, 1.0), (1, -1.0),
                     (2, 1.0), (2, -1.0)), start=2):
                requested[candidate] = 0.0
                requested[candidate, :, axis] = sign*basis_scale[axis]
        if rotation_lock_direction and self.contract.velocity_max[1] > 0.0:
            direction = float(np.sign(rotation_lock_direction))
            floor = float(self.contract.velocity_min[1])
            requested[..., 1] = direction*np.maximum(
                np.abs(requested[..., 1]), floor)
        # Disabled controller axes must never enter a sampled plan.  Applying
        # this after deterministic probes and direction latching also clears
        # stale warm-start values from a controller restarted with a stricter
        # local contract.
        disabled = self.contract.velocity_max[:CONTROL_AXES] <= 0.0
        requested[..., disabled] = 0.0
        return requested

    def _rotation_lock_direction(self, transmission_state) -> int:
        if (not self.config.rotation_direction_latch
                or transmission_state is None):
            return 0
        phase = tuple(transmission_state.phase)
        direction = np.asarray(
            transmission_state.motion_direction, dtype=np.int8)
        if len(phase) != CONTROL_AXES or direction.shape != (CONTROL_AXES,):
            raise ValueError("invalid transmission state")
        if phase[1] == "FAILED":
            raise ValueError("rotation transmission state failed")
        return int(direction[1]) if phase[1] == "TAKEUP" else 0

    def _project(self, requested_three: np.ndarray,
                 root_joint_position: np.ndarray,
                 blocked_motor_direction: np.ndarray | None = None,
                 initial_position_offset: np.ndarray | None = None,
                 dt_sequence: np.ndarray | None = None):
        count, horizon, _ = requested_three.shape
        step_dt = (np.full(horizon, self.config.step_s, dtype=np.float64)
                   if dt_sequence is None else
                   np.asarray(dt_sequence, dtype=np.float64))
        if (step_dt.shape != (horizon,)
                or not np.isfinite(step_dt).all()
                or np.any(step_dt <= 0.0)):
            raise ValueError(
                "dt_sequence must contain one positive value per step")
        offset = (np.zeros((count, N_AXES), dtype=np.float64)
                  if initial_position_offset is None else
                  np.asarray(initial_position_offset, dtype=np.float64))
        if (offset.shape != (count, N_AXES)
                or not np.isfinite(offset).all()):
            raise ValueError(
                "initial_position_offset must have shape (K,6)")
        positions = np.broadcast_to(
            root_joint_position, (count, N_AXES)).copy()+offset
        logical = np.zeros((count, horizon, N_AXES), dtype=np.float64)
        motor_rates = np.zeros_like(logical)
        realized_logical = np.zeros_like(logical)
        boundary_cost = np.zeros(count, dtype=np.float64)
        projection_cost = np.zeros(count, dtype=np.float64)
        blocked_candidate = np.zeros(count, dtype=bool)
        blocked_direction = (
            np.zeros(CONTROL_AXES, dtype=np.int8)
            if blocked_motor_direction is None else
            np.asarray(blocked_motor_direction, dtype=np.int8))
        if (blocked_direction.shape != (CONTROL_AXES,)
                or np.any(np.abs(blocked_direction) > 1)):
            raise ValueError(
                "blocked_motor_direction must contain three signs")
        velocity_scale = np.maximum(self.contract.velocity_max, 1e-9)
        lower = self.contract.control_position_lower
        upper = self.contract.control_position_upper
        span = upper-lower
        margin = self.config.boundary_margin_fraction*span
        root_clearance = np.minimum(
            root_joint_position-lower, upper-root_joint_position)
        root_boundary = np.maximum(
            0.0, (margin-root_clearance)/margin)
        # Feedback may legitimately sit just beyond the controller's
        # conservative margin while remaining inside the manager hard limit
        # (for example because of encoder quantization or stopping distance).
        # In that state hold and inward recovery must remain feasible.  Only
        # reject a candidate here when its estimated take-up travel moves the
        # shaft farther outside the conservative envelope; projection below
        # will erase outward useful motion and admit inward motion.
        root_positions = np.broadcast_to(
            root_joint_position, (count, N_AXES))
        takeup_worsens_lower = (
            (positions < lower) & (positions < root_positions))
        takeup_worsens_upper = (
            (positions > upper) & (positions > root_positions))
        blocked_candidate |= np.any(
            takeup_worsens_lower | takeup_worsens_upper, axis=1)
        for step in range(horizon):
            requested = np.zeros((count, N_AXES), dtype=np.float64)
            requested[:, :CONTROL_AXES] = requested_three[:, step]
            # Preserve the candidate's intended physical shaft direction
            # before position projection.  At a coupled boundary the final
            # contract may erase the saturated shaft while leaving motion on
            # another shaft (for example bend compensation leaking into
            # insertion).  Looking only at the realized motor command would
            # therefore miss exactly the direction registered at saturation.
            intended_logical = np.clip(
                requested, -self.contract.velocity_max,
                self.contract.velocity_max)
            below_floor = (
                (np.abs(intended_logical) > 0.0)
                & (np.abs(intended_logical)
                   < self.contract.velocity_min[None]))
            intended_logical = np.where(
                below_floor,
                np.copysign(
                    self.contract.velocity_min[None], intended_logical),
                intended_logical)
            intended_motor_axis = intended_logical.copy()
            intended_motor_axis[:, 0] -= intended_motor_axis[:, 2]
            intended_motor_axis[:, 5] += intended_motor_axis[:, 4]
            projected = self.contract.project_velocity_batch(
                requested, positions)
            step_logical = projected.logical_velocity.copy()
            step_motor = projected.motor_radians_per_second.copy()
            step_realized = projected.realized_logical_velocity.copy()
            # A coarse prediction-only step can cross a position boundary
            # even when its start lies inside the contract. Uniformly shorten
            # the complete coupled action to the available travel; if that
            # would put any active logical axis below its minimum velocity,
            # use hold for this step. This is prediction projection, not an
            # executable post-plan edit.
            travel = step_realized*step_dt[step]
            fraction = np.ones(count, dtype=np.float64)
            positive = travel > 0.0
            negative = travel < 0.0
            available_upper = np.maximum(0.0, upper-positions)
            available_lower = np.maximum(0.0, positions-lower)
            axis_fraction = np.ones_like(travel)
            axis_fraction[positive] = np.minimum(
                1.0, available_upper[positive]/travel[positive])
            axis_fraction[negative] = np.minimum(
                1.0, available_lower[negative]/(-travel[negative]))
            fraction = np.min(axis_fraction, axis=1)
            if np.any(fraction < 1.0):
                step_logical *= fraction[:, None]
                step_motor *= fraction[:, None]
                step_realized *= fraction[:, None]
                subminimum = (
                    (np.abs(step_logical) > 0.0)
                    & (np.abs(step_logical)
                       < self.contract.velocity_min[None]))
                stop = np.any(subminimum, axis=1)
                step_logical[stop] = 0.0
                step_motor[stop] = 0.0
                step_realized[stop] = 0.0
            if np.any(blocked_direction):
                intended_physical_rate = (
                    intended_motor_axis[:, :CONTROL_AXES]
                    / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[
                        None, :CONTROL_AXES])
                physical_sign = np.sign(
                    intended_physical_rate).astype(np.int8)
                blocked_axes = blocked_direction != 0
                # One nonzero sign is an independent-axis block. Multiple
                # signs encode an infeasible coupled take-up mode and must
                # only reject candidates that reproduce the complete mode.
                blocked_step = np.all(
                    physical_sign[:, blocked_axes]
                    == blocked_direction[blocked_axes][None], axis=1)
                # A direction saturated by take-up is infeasible at this
                # encoder position. Replace the complete coupled step with
                # zero rather than clipping one shaft and synthesizing a
                # different action. Candidate zero remains feasible.
                step_logical[blocked_step] = 0.0
                step_motor[blocked_step] = 0.0
                step_realized[blocked_step] = 0.0
                blocked_candidate |= blocked_step
            logical[:, step] = step_logical
            motor_rates[:, step] = step_motor
            realized_logical[:, step] = step_realized
            difference = ((requested-step_realized)/velocity_scale)
            if np.any(blocked_direction):
                difference[blocked_step] = (
                    requested[blocked_step]/velocity_scale)
            projection_cost += np.square(difference).sum(-1)
            positions += step_realized*step_dt[step]
            clearance = np.minimum(
                positions-lower, upper-positions)
            violation = np.maximum(
                0.0, (margin-clearance)/margin - root_boundary)
            boundary_cost += np.square(violation).sum(-1)
        return (logical, realized_logical, motor_rates, projection_cost,
                boundary_cost, blocked_candidate)

    @staticmethod
    def _first_physical_direction(requested_three: np.ndarray):
        """Classify a complete candidate without changing its trajectory."""
        motor_axis = np.asarray(requested_three, dtype=np.float64).copy()
        motor_axis[..., 0] -= motor_axis[..., 2]
        physical = (motor_axis
                    / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[
                        None, None, :CONTROL_AXES])
        sign = np.sign(physical).astype(np.int8)
        moving = sign != 0
        first_index = np.argmax(moving, axis=1)
        first = np.take_along_axis(
            sign, first_index[:, None, :], axis=1)[:, 0]
        first[~np.any(moving, axis=1)] = 0
        internal_reversal = np.any(sign > 0, axis=1) & np.any(sign < 0, axis=1)
        return first, internal_reversal

    def _grouped_candidates(self, lease_direction: np.ndarray):
        """Build U/C proposal groups with deterministic tendon probes.

        Every grouped proposal that can move the tendon receives a sustained
        probe. Unconstrained proposals receive both physical directions;
        continue proposals receive the leased physical direction. Probes are
        appended rather than substituted, preserving deterministic hold and
        the configured random population.
        """
        raw = self._samples()
        if not self.config.grouped_mode_sampling:
            self._last_tendon_probe_mask = np.zeros(len(raw), dtype=bool)
            return raw, np.zeros(len(raw), dtype=np.uint8), 1
        active = np.flatnonzero(
            (lease_direction != 0)
            & (self.contract.velocity_max[:CONTROL_AXES] > 0.0))
        masks = (list(product((0, 1), repeat=len(active)))
                 if len(active) else [()])
        groups, labels, probe_masks = [], [], []
        tendon_enabled = self.contract.velocity_max[2] > 0.0
        # LEARNING candidates are scaled after proposal construction. Choose
        # a pre-scale probe that still reaches the physical minimum velocity.
        learning_scale = self.config.engaged_gain_learning_velocity_scale
        probe_velocity = min(
            float(self.contract.velocity_max[2]),
            max(float(self.config.noise_std[2]),
                float(self.contract.velocity_min[2])/learning_scale))
        unit_sign = float(np.sign(
            MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[2]))
        for group_index, choices in enumerate(masks):
            # Partition the complete random population, then append a small
            # deterministic coverage set. This keeps total stochastic samples
            # fixed while ensuring no U/C branch loses tendon exploration due
            # to round-robin assignment.
            indices = np.arange(group_index, len(raw), len(masks))
            if not len(indices):
                continue
            candidate = raw[indices].copy()
            local_probe = np.zeros(len(candidate), dtype=bool)
            choice_by_axis = {
                int(axis): int(choice)
                for choice, axis in zip(choices, active)}
            physical_probe_directions = []
            if tendon_enabled:
                if choice_by_axis.get(2, 0):
                    physical_probe_directions = [
                        int(np.sign(lease_direction[2]))]
                else:
                    physical_probe_directions = [1, -1]
            if physical_probe_directions:
                probes = np.zeros(
                    (len(physical_probe_directions),
                     self.config.horizon_steps, CONTROL_AXES),
                    dtype=np.float64)
                for probe_index, physical_direction in enumerate(
                        physical_probe_directions):
                    # Axis-2 hardware conversion may reverse sign, so define
                    # coverage in physical-shaft coordinates explicitly.
                    probes[probe_index, :, 2] = (
                        physical_direction*unit_sign*probe_velocity)
                candidate = np.concatenate((candidate, probes), axis=0)
                local_probe = np.concatenate((
                    local_probe,
                    np.ones(len(probes), dtype=bool)))
            motor_axis = candidate.copy()
            motor_axis[..., 0] -= motor_axis[..., 2]
            code = 0
            for choice, axis in zip(choices, active):
                if not choice:  # unconstrained
                    continue
                code |= 1 << int(axis)
                physical = (motor_axis[..., axis]
                            / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[axis])
                opposite = ((np.sign(physical) != 0)
                            & (np.sign(physical) != lease_direction[axis]))
                motor_axis[..., axis][opposite] = 0.0
            candidate = motor_axis
            candidate[..., 0] += candidate[..., 2]
            groups.append(candidate)
            labels.extend([code]*len(candidate))
            probe_masks.append(local_probe)
        grouped = np.concatenate(groups, axis=0)
        self._last_tendon_probe_mask = np.concatenate(probe_masks)
        return (grouped, np.asarray(labels, dtype=np.uint8), len(masks))

    def _takeup_position_offsets(
            self, first_direction, lease_direction, transmission_state):
        """Reserve estimated raw joint travel before useful MPPI motion.

        The MPPI samples describe post-engagement motion.  Reaching that state
        still consumes encoder travel, so every reversing mode is checked from
        the position expected after its direction-dependent gap is exhausted.
        """
        count = len(first_direction)
        travel = np.zeros((count, CONTROL_AXES), dtype=np.float64)
        motion = np.asarray(lease_direction, dtype=np.int8)
        positive = np.asarray(
            self.config.reversal_backlash_rad, dtype=np.float64)
        negative = positive.copy()
        remaining = np.zeros(CONTROL_AXES, dtype=np.float64)
        phase = ("ENGAGED",)*CONTROL_AXES
        if transmission_state is not None:
            point_positive = np.asarray(
                transmission_state.width_positive_rad, dtype=np.float64)
            point_negative = np.asarray(
                transmission_state.width_negative_rad, dtype=np.float64)
            point_remaining = np.asarray(
                transmission_state.remaining_rad, dtype=np.float64)
            positive_upper = np.asarray(
                getattr(transmission_state, "width_positive_upper_rad",
                        point_positive), dtype=np.float64)
            negative_upper = np.asarray(
                getattr(transmission_state, "width_negative_upper_rad",
                        point_negative), dtype=np.float64)
            remaining_upper = np.asarray(
                getattr(transmission_state, "remaining_upper_rad",
                        point_remaining), dtype=np.float64)
            # Backward-compatible snapshots without interval evidence use the
            # old point estimate as a degenerate interval.
            positive = np.where(
                (positive_upper <= 0.0) & (point_positive > 0.0),
                point_positive, positive_upper)
            negative = np.where(
                (negative_upper <= 0.0) & (point_negative > 0.0),
                point_negative, negative_upper)
            remaining = np.where(
                (remaining_upper <= 0.0) & (point_remaining > 0.0),
                point_remaining, remaining_upper)
            motion = np.asarray(
                transmission_state.motion_direction, dtype=np.int8)
            phase = tuple(transmission_state.phase)
        for axis in range(CONTROL_AXES):
            direction = first_direction[:, axis]
            selected = np.where(direction > 0, positive[axis], negative[axis])
            changed = ((direction != 0)
                       & ((motion[axis] == 0) | (direction != motion[axis])))
            active = ((direction == motion[axis])
                      & np.isin(phase[axis], ("TAKEUP", "PROVISIONAL")))
            travel[:, axis] = np.where(
                changed, selected, np.where(active, remaining[axis], 0.0))
            travel[:, axis] *= direction
        travel *= self.config.takeup_limit_reserve_scale
        motor_axis_delta = np.zeros((count, N_AXES), dtype=np.float64)
        motor_axis_delta[:, :CONTROL_AXES] = (
            travel/(2.0*np.pi)
            * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[
                None, :CONTROL_AXES]*60.0)
        logical = motor_axis_delta.copy()
        logical[:, 0] += logical[:, 2]
        logical[:, 5] -= logical[:, 4]
        return logical, np.abs(travel)

    def _takeup_low_confidence_extra_travel(
            self, first_direction, transmission_state):
        """Extra conservative travel from interval width and confidence.

        The upper interval bound is already reserved by the take-up position
        offsets. Low confidence expands that bound by up to one additional
        interval width. This remains a physical travel quantity, not a second
        independently weighted cost.
        """
        count = len(first_direction)
        extra = np.zeros((count, CONTROL_AXES), dtype=np.float64)
        if transmission_state is None:
            return extra
        motion = np.asarray(
            transmission_state.motion_direction, dtype=np.int8)
        confidence = np.asarray(
            transmission_state.confidence, dtype=np.float64)
        point_pos = np.asarray(
            transmission_state.width_positive_rad, dtype=np.float64)
        point_neg = np.asarray(
            transmission_state.width_negative_rad, dtype=np.float64)
        point_rem = np.asarray(
            transmission_state.remaining_rad, dtype=np.float64)
        pos_lo = np.asarray(
            transmission_state.width_positive_lower_rad, dtype=np.float64)
        pos_hi = np.asarray(
            transmission_state.width_positive_upper_rad, dtype=np.float64)
        neg_lo = np.asarray(
            transmission_state.width_negative_lower_rad, dtype=np.float64)
        neg_hi = np.asarray(
            transmission_state.width_negative_upper_rad, dtype=np.float64)
        rem_lo = np.asarray(
            transmission_state.remaining_lower_rad, dtype=np.float64)
        rem_hi = np.asarray(
            transmission_state.remaining_upper_rad, dtype=np.float64)
        pos_lo = np.where((pos_hi <= 0.0) & (point_pos > 0.0),
                          point_pos, pos_lo)
        pos_hi = np.where((pos_hi <= 0.0) & (point_pos > 0.0),
                          point_pos, pos_hi)
        neg_lo = np.where((neg_hi <= 0.0) & (point_neg > 0.0),
                          point_neg, neg_lo)
        neg_hi = np.where((neg_hi <= 0.0) & (point_neg > 0.0),
                          point_neg, neg_hi)
        rem_lo = np.where((rem_hi <= 0.0) & (point_rem > 0.0),
                          point_rem, rem_lo)
        rem_hi = np.where((rem_hi <= 0.0) & (point_rem > 0.0),
                          point_rem, rem_hi)
        phase = tuple(transmission_state.phase)
        for axis in range(CONTROL_AXES):
            direction = first_direction[:, axis]
            lower = np.where(direction > 0, pos_lo[axis], neg_lo[axis])
            upper = np.where(direction > 0, pos_hi[axis], neg_hi[axis])
            changed = ((direction != 0)
                       & ((motion[axis] == 0)
                          | (direction != motion[axis])))
            active = ((direction == motion[axis])
                      & np.isin(phase[axis], ("TAKEUP", "PROVISIONAL")))
            lower = np.where(active, rem_lo[axis], lower)
            upper = np.where(active, rem_hi[axis], upper)
            relevant = changed | active
            interval_width = np.maximum(0.0, upper-lower)
            extra[:, axis] = np.where(
                relevant,
                np.clip(1.0-confidence[axis], 0.0, 1.0)*interval_width,
                0.0)
        return extra*self.config.takeup_limit_reserve_scale

    def _gain_learning_velocity_scales(
            self, first_direction, transmission_state):
        """Conservatively scale tendon candidates until gain is learned.

        This changes candidates before rollout and scoring; it never edits the
        selected plan after optimization. Only the configured tendon axis is
        affected, and a direction returns to full speed once its posterior is
        CONFIDENT.
        """
        result = np.ones(len(first_direction), dtype=np.float64)
        configured = self.config.engaged_gain_learning_velocity_scale
        if configured >= 1.0 or transmission_state is None:
            return result
        belief = getattr(transmission_state, "engaged_gain", None)
        if belief is None or not bool(getattr(belief, "enabled", False)):
            return result
        axis = int(belief.tendon_axis)
        if not 0 <= axis < CONTROL_AXES:
            raise ValueError("invalid engaged gain tendon axis")
        for candidate in range(len(first_direction)):
            direction = int(first_direction[candidate, axis])
            if direction == 0:
                continue
            index = 1 if direction > 0 else 0
            if belief.status[axis][index] != "CONFIDENT":
                result[candidate] = configured
        return result

    def _engaged_gain_scenarios(self, first_direction, transmission_state):
        """Return candidate-wise mean/lower/upper engaged gain scenarios."""
        count = len(first_direction)
        unit = np.ones((count, 1), dtype=np.float64)
        if not self.config.engaged_gain_scenarios or transmission_state is None:
            return unit
        belief = getattr(transmission_state, "engaged_gain", None)
        if belief is None or not bool(getattr(belief, "enabled", False)):
            return unit
        axis = int(belief.tendon_axis)
        if not 0 <= axis < CONTROL_AXES:
            raise ValueError("invalid engaged gain tendon axis")
        result = np.empty((count, 3), dtype=np.float64)
        engaged_direction = np.asarray(
            transmission_state.engaged_direction, dtype=np.int8)
        active_direction = np.asarray(
            belief.active_direction, dtype=np.int8)
        for candidate in range(count):
            direction = int(first_direction[candidate, axis])
            if direction == 0:
                direction = int(engaged_direction[axis])
            if direction == 0:
                direction = int(active_direction[axis])
            result[candidate] = belief.scenarios(axis, direction)
        if not np.isfinite(result).all() or np.any(result <= 0.0):
            raise ValueError("invalid engaged gain scenarios")
        return result

    def _apply_rollout_backlash(self, motor_rates: np.ndarray,
                                previous_motor_rate: np.ndarray) -> np.ndarray:
        """Consume reversal take-up before passing samples to the model.

        This is candidate-local state: every rollout starts from the sign of
        the last commanded motor rate, and its own future reversals consume a
        fresh per-axis backlash width. The executable command remains the
        original projected rate; only predicted achieved motion is filtered.
        """
        rates = np.asarray(motor_rates, dtype=np.float64)
        previous = np.asarray(previous_motor_rate, dtype=np.float64)
        if (rates.ndim != 3 or rates.shape[-1] != CONTROL_AXES
                or previous.shape != (CONTROL_AXES,)):
            raise ValueError("invalid motor-rate shape for backlash rollout")
        width = np.asarray(
            self.config.reversal_backlash_rad, dtype=np.float64)
        if not np.any(width > 0.0):
            return rates
        achieved = rates.copy()
        remaining = np.zeros((len(rates), CONTROL_AXES), dtype=np.float64)
        last_sign = np.broadcast_to(
            np.sign(previous), remaining.shape).copy()
        for step in range(rates.shape[1]):
            requested = rates[:, step]
            sign = np.sign(requested)
            reversal = ((sign != 0.0) & (last_sign != 0.0)
                        & (sign != last_sign))
            remaining[reversal] = np.broadcast_to(
                width, remaining.shape)[reversal]
            travel = np.abs(requested)*self.config.step_s
            blocked = np.minimum(remaining, travel)
            remaining -= blocked
            fraction = np.divide(
                blocked, travel, out=np.zeros_like(blocked),
                where=travel > 0.0)
            achieved[:, step] *= 1.0-fraction
            last_sign[sign != 0.0] = sign[sign != 0.0]
        return achieved

    def _safe_result(self, started: float, reason: str, *,
                     phase_timings_ms: tuple[float, float, float, float] =
                     (0.0, 0.0, 0.0, 0.0)) -> MppiPlan:
        zero = np.zeros((self.config.horizon_steps, N_AXES), dtype=np.float64)
        self._nominal.fill(0.0)
        return MppiPlan(
            zero[0].copy(), zero, zero.copy(), None, float("inf"), 0.0,
            max(0.0, self._clock() - started), False, reason,
            *phase_timings_ms,
            compensated_motor_radians_per_second_sequence=zero.copy(),
            transmitted_motor_radians_per_second_sequence=zero.copy())

    def enforce_deadline(self, plan: MppiPlan, started: float) -> MppiPlan:
        """Include caller-side snapshot and commit wait in the plan budget."""
        elapsed = max(float(plan.elapsed_s), self._clock()-float(started))
        if plan.valid and elapsed > self.config.planning_deadline_s:
            return replace(
                self._safe_result(started, "deadline_missed"),
                sample_projection_ms=plan.sample_projection_ms,
                rollout_ms=plan.rollout_ms,
                cost_weighting_ms=plan.cost_weighting_ms,
                update_projection_ms=plan.update_projection_ms)
        if plan.valid:
            return replace(plan, elapsed_s=elapsed)
        return plan

    def plan(self, root_state: Any, joint_position,
             target_tip_base_m, *, target_markers_base_m=None,
             target_tangent_base=None,
             observed_tip_base_m=None,
             previous_logical_velocity=None,
             deadline_started_s=None, transmission_state=None,
             takeup_motor_radians_per_second=None,
             blocked_motor_direction=None, direction_lease=None,
             approved_reversal_direction=None) -> MppiPlan:
        """Plan one action and shift warm start for the next camera frame."""
        started = (self._clock() if deadline_started_s is None
                   else float(deadline_started_s))
        position = _finite_vector("joint_position", joint_position, N_AXES)
        target_tip = np.asarray(target_tip_base_m, dtype=np.float64)
        horizon = self.config.horizon_steps
        if target_tip.shape == (3,):
            target_tip = np.broadcast_to(target_tip, (horizon, 3)).copy()
        if (target_tip.shape != (horizon, 3)
                or not np.all(np.isfinite(target_tip))):
            raise ValueError("target_tip_base_m must have shape (3,) or (H,3)")
        observed_tip = (
            None if observed_tip_base_m is None else
            _finite_vector("observed_tip_base_m", observed_tip_base_m, 3))
        target_tangent = None
        if target_tangent_base is not None:
            target_tangent = np.asarray(
                target_tangent_base, dtype=np.float64)
            if target_tangent.shape == (3,):
                target_tangent = np.broadcast_to(
                    target_tangent, (horizon, 3)).copy()
            if (target_tangent.shape != (horizon, 3)
                    or not np.all(np.isfinite(target_tangent))):
                raise ValueError(
                    "target_tangent_base must have shape (3,) or (H,3)")
            tangent_norm = np.linalg.norm(target_tangent, axis=1)
            if np.any(tangent_norm <= 1e-12):
                raise ValueError("target_tangent_base must be nonzero")
            target_tangent /= tangent_norm[:, None]
        previous = (np.zeros(N_AXES) if previous_logical_velocity is None
                    else _finite_vector("previous_logical_velocity",
                                        previous_logical_velocity, N_AXES))
        marker_target = None
        marker_mask = None
        if target_markers_base_m is not None:
            marker_target = np.asarray(
                target_markers_base_m, dtype=np.float64)
            if marker_target.shape != (4, 3):
                raise ValueError("target_markers_base_m must have shape (4,3)")
            # The base marker is excluded; finite intermediate markers
            # constrain shape while the separate target controls the tip.
            marker_mask = np.isfinite(marker_target).all(-1)
            marker_mask[0] = False
            marker_mask[3] = False
        # Only point targets receive the model-only terminal tail. Point and
        # continuous-path targets may independently opt into exact coarse
        # model updates; a marker target would need a corresponding future
        # shape reference before it can use that route.
        point_target = target_tangent is None and marker_target is None
        path_target = target_tangent is not None and marker_target is None
        tail_steps = (
            self.config.point_prediction_tail_steps
            if point_target else 0)
        head_step_s = (
            self.config.point_rollout_step_s
            if point_target and self.config.point_rollout_step_s > 0.0
            else self.config.step_s)
        prediction_horizon = horizon+tail_steps
        rollout_dt = np.concatenate((
            np.full(horizon, head_step_s, dtype=np.float64),
            np.full(tail_steps, self.config.point_prediction_tail_step_s,
                    dtype=np.float64)))
        prediction_target = (
            target_tip if not tail_steps else
            np.concatenate((
                target_tip,
                np.repeat(target_tip[-1:, :], tail_steps, axis=0)), axis=0))
        # Preserve the established scalar-dt backend contract when the tail is
        # disabled; nonuniform dt is needed only for the coarsened tail.
        rollout_backend_dt = (
            rollout_dt if tail_steps else head_step_s)

        phase_started = perf_counter()
        blocked_direction = (
            np.zeros(CONTROL_AXES, dtype=np.int8)
            if blocked_motor_direction is None else
            np.asarray(blocked_motor_direction, dtype=np.int8))
        if (blocked_direction.shape != (CONTROL_AXES,)
                or np.any(np.abs(blocked_direction) > 1)):
            raise ValueError(
                "blocked_motor_direction must contain three signs")
        lease_direction = (
            np.zeros(CONTROL_AXES, dtype=np.int8)
            if direction_lease is None else
            np.asarray(direction_lease, dtype=np.int8))
        approved_direction = (
            np.zeros(CONTROL_AXES, dtype=np.int8)
            if approved_reversal_direction is None else
            np.asarray(approved_reversal_direction, dtype=np.int8))
        if (lease_direction.shape != (CONTROL_AXES,)
                or np.any(np.abs(lease_direction) > 1)):
            raise ValueError("direction_lease must contain three signs")
        if (approved_direction.shape != (CONTROL_AXES,)
                or np.any(np.abs(approved_direction) > 1)):
            raise ValueError(
                "approved_reversal_direction must contain three signs")
        # Transmission history can retain a lease or saturation direction
        # from before an axis was disabled in the controller profile. Such an
        # axis is neither a proposal mode nor an executable reversal.
        disabled_axes = self.contract.velocity_max[:CONTROL_AXES] <= 0.0
        blocked_direction = blocked_direction.copy()
        lease_direction = lease_direction.copy()
        approved_direction = approved_direction.copy()
        blocked_direction[disabled_axes] = 0
        lease_direction[disabled_axes] = 0
        approved_direction[disabled_axes] = 0
        # MPPI operates exclusively in useful, post-take-up command space.
        # Stratify the one rollout batch into complete U/C mode proposals:
        # each leased shaft is either unconstrained or restricted to continue
        # (zero is included). No selected plan is edited after scoring.
        requested, proposal_group, proposal_group_count = (
            self._grouped_candidates(lease_direction))
        candidate_count = len(requested)
        first_requested_direction, internal_reversal = (
            self._first_physical_direction(requested))
        gain_learning_velocity_scales = (
            self._gain_learning_velocity_scales(
                first_requested_direction, transmission_state))
        if np.any(gain_learning_velocity_scales < 1.0):
            requested = requested.copy()
            tendon_axis = int(transmission_state.engaged_gain.tendon_axis)
            if tendon_axis == 2:
                # Logical bend is mechanically coupled into motor axis 0.
                # Scale the physical tendon shaft while preserving every
                # other physical-shaft proposal constraint, then map back.
                motor_axis = requested.copy()
                motor_axis[..., 0] -= motor_axis[..., 2]
                motor_axis[..., 2] *= (
                    gain_learning_velocity_scales[:, None])
                requested = motor_axis
                requested[..., 0] += requested[..., 2]
            else:
                requested[:, :, tendon_axis] *= (
                    gain_learning_velocity_scales[:, None])
        reversal_direction = np.where(
            (lease_direction[None] != 0)
            & (first_requested_direction != 0)
            & (first_requested_direction != lease_direction[None]),
            first_requested_direction, 0).astype(np.int8)
        reversal_mask = np.zeros(candidate_count, dtype=np.uint8)
        for axis in range(CONTROL_AXES):
            reversal_mask |= ((reversal_direction[:, axis] != 0).astype(
                np.uint8) << axis)
        takeup_position_offset, takeup_travel_rad = (
            self._takeup_position_offsets(
                first_requested_direction, lease_direction,
                transmission_state))
        rollout_requested = requested
        if tail_steps:
            rollout_requested = np.concatenate((
                requested,
                np.repeat(requested[:, -1:, :], tail_steps, axis=1)),
                axis=1)
        (rollout_logical, rollout_realized_logical, motor_rates,
         projection_cost, boundary_cost, blocked_candidate) = self._project(
             rollout_requested, position, blocked_direction,
             takeup_position_offset, rollout_dt)
        # The prediction tail affects scoring and feasibility only. Keep the
        # optimized sequence as the sole executable/warm-start sequence.
        logical = rollout_logical[:, :horizon]
        realized_logical = rollout_realized_logical[:, :horizon]
        blocked_candidate |= np.any(internal_reversal, axis=1)
        previous_motor_rate = self.contract.project_velocity(
            previous, position).motor_radians_per_second[:CONTROL_AXES]
        # MPPI predicts only effective post-engagement dynamics. Physical
        # take-up is an atomic low-speed transaction outside the planner.
        transmission_prediction_applied = False
        raw_rollout_motor_rates = motor_rates[..., :CONTROL_AXES]
        rollout_motor_rates = raw_rollout_motor_rates
        takeup_risk_s = np.zeros(candidate_count, dtype=np.float64)
        if takeup_motor_radians_per_second is not None:
            takeup_rate = _finite_vector(
                "takeup_motor_radians_per_second",
                takeup_motor_radians_per_second, CONTROL_AXES)
            extra_travel = self._takeup_low_confidence_extra_travel(
                first_requested_direction, transmission_state)
            risk_travel = takeup_travel_rad+extra_travel
            takeup_risk_s = np.max(
                risk_travel/np.maximum(np.abs(takeup_rate), 1e-12),
                axis=1)
            transaction = np.any(takeup_travel_rad > 0.0, axis=1)
            takeup_risk_s += np.where(
                transaction, self.config.takeup_confirmation_time_s, 0.0)
        gain_scenarios = self._engaged_gain_scenarios(
            first_requested_direction, transmission_state)
        gain_scenario_count = gain_scenarios.shape[1]
        if gain_scenario_count > 1 and marker_target is not None:
            return self._safe_result(
                started, "engaged_gain_shape_target_unsupported")
        scenario_rollout_motor_rates = (
            rollout_motor_rates if gain_scenario_count == 1 else
            np.repeat(rollout_motor_rates[:, None],
                      gain_scenario_count, axis=1).reshape(
                          candidate_count*gain_scenario_count,
                          prediction_horizon, CONTROL_AXES))
        scenario_gain = (None if gain_scenario_count == 1 else
                         gain_scenarios.reshape(-1))
        sample_projection_ms = 1e3*(perf_counter()-phase_started)
        phase_started = perf_counter()
        try:
            # The deployed v171 runtime provides a tip-only route that avoids
            # marker and stacked-centerline work. Preserve the generic backend
            # contract used by validation fakes and shape-aware objectives.
            coarse_rollout = (
                (point_target and self.config.point_rollout_coarse_steps)
                or (path_target and self.config.path_rollout_coarse_steps))
            control_predict_name = (
                "predict_control_sequence_coarse" if coarse_rollout
                else "predict_control_sequence")
            control_predict = getattr(self.backend, control_predict_name, None)
            if coarse_rollout and control_predict is None:
                raise RuntimeError("backend lacks coarse rollout support")
            if control_predict is not None and marker_target is None:
                rollout_kwargs = {}
                if transmission_prediction_applied:
                    rollout_kwargs["raw_motor_velocity_sequence"] = (
                        raw_rollout_motor_rates)
                if scenario_gain is not None:
                    rollout_kwargs["distal_gain"] = scenario_gain
                prediction = control_predict(
                    root_state, scenario_rollout_motor_rates,
                    rollout_backend_dt, **rollout_kwargs)
            else:
                if transmission_prediction_applied:
                    prediction = self.backend.predict_sequence(
                        root_state, rollout_motor_rates, rollout_backend_dt,
                        raw_motor_velocity_sequence=(
                            raw_rollout_motor_rates))
                else:
                    prediction = self.backend.predict_sequence(
                        root_state, rollout_motor_rates, rollout_backend_dt)
            tips = torch.as_tensor(prediction.tip_base_m)
            predicted_lambda = getattr(prediction, "distal_lambda", None)
            if predicted_lambda is not None:
                predicted_lambda = torch.as_tensor(predicted_lambda)
            if gain_scenario_count > 1:
                tips = tips.reshape(
                    candidate_count, gain_scenario_count,
                    prediction_horizon, 3)
                if predicted_lambda is not None:
                    predicted_lambda = predicted_lambda.reshape(
                        candidate_count, gain_scenario_count,
                        prediction_horizon)
            markers = (None if prediction.markers_base_m is None else
                       torch.as_tensor(prediction.markers_base_m))
            # CUDA kernel submission is asynchronous. Synchronizing here makes
            # rollout_ms and the callback-entry deadline describe completed
            # model work rather than queue-submission latency.
            _synchronize_torch_device(tips.device)
        except Exception as error:  # A rollout failure must fail closed.
            name = type(error).__name__
            return self._safe_result(
                started, f"rollout_error:{name}",
                phase_timings_ms=(sample_projection_ms,
                                  1e3*(perf_counter()-phase_started),
                                  0.0, 0.0))
        rollout_ms = 1e3*(perf_counter()-phase_started)
        invalid_markers = bool(
            marker_target is not None
            and (markers is None
                 or tuple(markers.shape) != (
                     candidate_count, prediction_horizon, 4, 3)
                 or not bool(torch.isfinite(markers).all())))
        expected_tip_shape = ((candidate_count, prediction_horizon, 3)
                              if gain_scenario_count == 1 else
                              (candidate_count, gain_scenario_count,
                               prediction_horizon, 3))
        if (tuple(tips.shape) != expected_tip_shape
                or not bool(torch.isfinite(tips).all())
                or invalid_markers):
            return self._safe_result(
                started, "invalid_rollout",
                phase_timings_ms=(sample_projection_ms, rollout_ms,
                                  0.0, 0.0))

        first_step_lambda_shift = np.zeros(
            candidate_count, dtype=np.float64)
        if self.config.engaged_gain_maximum_first_step_shift > 0.0:
            if predicted_lambda is None:
                return self._safe_result(
                    started, "engaged_gain_lambda_prediction_missing",
                    phase_timings_ms=(sample_projection_ms, rollout_ms,
                                      0.0, 0.0))
            root_lambda = torch.as_tensor(
                root_state.lambda_value, device=tips.device, dtype=tips.dtype)
            first_lambda = (predicted_lambda[:, 0] if gain_scenario_count == 1
                            else predicted_lambda[:, :, 0])
            shift_t = torch.abs(first_lambda-root_lambda)
            if gain_scenario_count > 1:
                shift_t = shift_t.max(-1).values
            first_step_lambda_shift = shift_t.detach().cpu().numpy()
            blocked_candidate |= (
                first_step_lambda_shift
                > self.config.engaged_gain_maximum_first_step_shift)

        phase_started = perf_counter()
        device, dtype = tips.device, tips.dtype
        target = torch.as_tensor(
            prediction_target, device=device, dtype=dtype)
        mean_tips = tips if gain_scenario_count == 1 else tips[:, 0]
        tangent_t = (None if target_tangent is None else torch.as_tensor(
            target_tangent, device=device, dtype=dtype))
        # A continuous path is a forward corridor, not a sequence of
        # mandatory point captures. Penalize cross-track displacement and lag
        # behind the local reference plane, while allowing forward motion to
        # advance to the next reference. Endpoint capture omits tangents and
        # retains the exact Euclidean point objective.
        tracking_error_squared = _tip_tracking_error_squared(
            tips, target, tangent_t)
        step_weights = torch.ones(
            prediction_horizon, device=device, dtype=dtype)
        step_weights[-1] *= self.config.terminal_weight
        scenario_tracking_costs = self.config.tip_weight*(
            tracking_error_squared*step_weights).sum(-1)
        if gain_scenario_count == 1:
            tracking_costs = scenario_tracking_costs
        else:
            scenario_mean = scenario_tracking_costs.mean(-1)
            tail_count = max(1, int(np.ceil(
                (1.0-self.config.engaged_gain_cvar_alpha)
                * gain_scenario_count)))
            scenario_cvar = torch.topk(
                scenario_tracking_costs, tail_count, dim=-1).values.mean(-1)
            tracking_costs = (scenario_mean
                              + self.config.engaged_gain_risk_beta
                              * scenario_cvar)
        costs = tracking_costs.clone()
        logical_t = torch.as_tensor(
            realized_logical, device=device, dtype=dtype)
        safe_velocity_max = np.where(
            self.contract.velocity_max > 0.0,
            self.contract.velocity_max, 1.0)
        scale = torch.as_tensor(
            safe_velocity_max, device=device, dtype=dtype)
        normalized = logical_t/scale
        first = normalized[:, :1]-torch.as_tensor(
            previous/safe_velocity_max,
            device=device, dtype=dtype)[None, None]
        delta = torch.cat((first, normalized[:, 1:]-normalized[:, :-1]), 1)
        costs += self.config.slew_weight*delta.square().sum((-1, -2))
        # MPPI samples useful post-take-up motion, but the physical plant must
        # still pay to change a response-confirmed shaft direction.  Use the
        # direction lease when one exists: the immediately previous command
        # is deliberately zero during a take-up hold and therefore cannot
        # represent transmission history.  Before a lease exists, retain the
        # previous physical command as the best available direction memory.
        first_motor_rate = motor_rates[:, 0, :CONTROL_AXES]
        previous_reversal = (
            first_motor_rate*previous_motor_rate[None] < 0.0)
        leased_axis = lease_direction != 0
        switch_event = previous_reversal
        if np.any(leased_axis):
            switch_event[:, leased_axis] = (
                reversal_direction[:, leased_axis] != 0)
        switch_count = switch_event.sum(-1).astype(np.int32)
        costs += torch.as_tensor(
            self.config.boundary_weight*boundary_cost
            + self.config.takeup_risk_weight*takeup_risk_s,
            device=device, dtype=dtype)
        blocked_candidate_t = torch.as_tensor(
            blocked_candidate, device=device, dtype=torch.bool)
        unrestricted_eligible_t = ~blocked_candidate_t
        unrestricted_costs = torch.where(
            unrestricted_eligible_t, costs,
            torch.full_like(costs, torch.inf))
        costs_finite_t = (
            torch.isfinite(costs).all()
            & torch.any(unrestricted_eligible_t))
        unrestricted_best_t = torch.argmin(unrestricted_costs)
        unrestricted_candidate_index = int(
            unrestricted_best_t.detach().cpu())
        proposed_reversal_direction = reversal_direction[
            unrestricted_candidate_index].copy()
        unrestricted_reversal_mask = int(
            reversal_mask[unrestricted_candidate_index])

        mode_best_total_cost = np.full(
            1 << CONTROL_AXES, np.inf, dtype=np.float64)
        mode_best_index = np.full(1 << CONTROL_AXES, -1, dtype=np.int64)
        for mode in range(1 << CONTROL_AXES):
            member = ((reversal_mask == mode) & ~blocked_candidate)
            if not np.any(member):
                continue
            member_t = torch.as_tensor(member, device=device, dtype=torch.bool)
            index_t = torch.argmin(torch.where(
                member_t, costs, torch.full_like(costs, torch.inf)))
            index = int(index_t.detach().cpu())
            mode_best_index[mode] = index
            mode_best_total_cost[mode] = float(costs[index_t].detach().cpu())

        no_reversal_index = int(mode_best_index[0])
        if no_reversal_index < 0:
            return self._safe_result(
                started, "no_nonreversing_candidate",
                phase_timings_ms=(sample_projection_ms, rollout_ms,
                                  1e3*(perf_counter()-phase_started), 0.0))
        reversal_axis_cost_improvement = np.zeros(
            CONTROL_AXES, dtype=np.float64)
        reversal_axis_terminal_improvement = np.zeros(
            CONTROL_AXES, dtype=np.float64)
        hold_branch_applied = False
        raw_zero_terminal_error = float(torch.linalg.vector_norm(
            (mean_tips[0, -1]-target[-1])*1000.0).detach().cpu())
        with self._capture_lock:
            capture_response_scale = self._capture_passive_response_scale
        zero_terminal_error = raw_zero_terminal_error
        if observed_tip is not None and target_tangent is None:
            raw_zero_tip = mean_tips[0, -1].detach().cpu().numpy()
            adjusted_zero_tip = observed_tip + capture_response_scale*(
                raw_zero_tip-observed_tip)
            zero_terminal_error = float(np.linalg.vector_norm(
                (adjusted_zero_tip-prediction_target[-1])*1000.0))
        unrestricted_terminal = float(torch.linalg.vector_norm(
            (mean_tips[unrestricted_best_t, -1]-target[-1])*1000.0
        ).detach().cpu())
        complete_mode_cost_improvement = max(
            0.0, float((costs[no_reversal_index]
                        - costs[unrestricted_best_t]).detach().cpu()))
        no_reversal_terminal = float(torch.linalg.vector_norm(
            (mean_tips[no_reversal_index, -1]-target[-1])*1000.0
        ).detach().cpu())
        for axis in np.flatnonzero(proposed_reversal_direction):
            reversal_axis_cost_improvement[axis] = (
                complete_mode_cost_improvement)
            reversal_axis_terminal_improvement[axis] = (
                no_reversal_terminal-unrestricted_terminal)

        # Only independently sampled, complete plans are executable. Grouped
        # U/C sampling already compares the best complete plan from every
        # continuation combination with the unconstrained alternatives. Do
        # not apply the legacy response-clocked reversal approval afterward:
        # that second decision rule can veto the optimizer's winner forever
        # and reduce the executed command to hold. The transaction arbiter
        # still realizes and confirms any selected take-up before useful MPPI
        # motion, while ``blocked_candidate`` continues to enforce physical
        # limits including the estimated take-up travel.
        if self.config.grouped_mode_sampling:
            eligible = ~blocked_candidate
        else:
            approved_candidate = np.all(
                (reversal_direction == 0)
                | (reversal_direction == approved_direction[None]), axis=1)
            eligible = (~blocked_candidate) & approved_candidate
        # Point capture is a model-selected branch, not a post-plan command
        # edit. Near the target, deterministic hold wins unless a sampled plan
        # provides a meaningful predicted terminal improvement. Once selected,
        # retain hold briefly so delayed plant response can be observed before
        # applying more load in either direction. Continuous-path corridors do
        # not use this point-capture rule.
        hold_branch_terminal_error = float("nan")
        capture_enabled = (
            target_tangent is None and self.config.capture_radius_mm > 0.0)
        now_s = (self._clock() if capture_enabled else float("-inf"))
        with self._capture_lock:
            hold_active = (
                capture_enabled and now_s < self._capture_hold_until_s)
            capture_rearm_blocked = self._capture_rearm_blocked
        if (capture_enabled and not hold_active and not capture_rearm_blocked
                and np.any(eligible)):
            provisional = torch.where(
                torch.as_tensor(eligible, device=device, dtype=torch.bool),
                costs, torch.full_like(costs, torch.inf))
            provisional_index = int(torch.argmin(provisional).detach().cpu())
            provisional_terminal = float(torch.linalg.vector_norm(
                (mean_tips[provisional_index, -1]-target[-1])*1000.0
            ).detach().cpu())
            terminal_improvement = (
                zero_terminal_error-provisional_terminal)
            if (zero_terminal_error <= self.config.capture_radius_mm
                    and terminal_improvement
                    < self.config.capture_minimum_terminal_improvement_mm):
                with self._capture_lock:
                    if not self._capture_rearm_blocked:
                        hold_active = True
                        self._capture_rearm_blocked = True
                        self._capture_last_response_reason = (
                            "awaiting_capture_response")
                        self._capture_hold_until_s = max(
                            self._capture_hold_until_s,
                            now_s+self.config.capture_hold_s)
        if hold_active:
            eligible = np.zeros(candidate_count, dtype=bool)
            eligible[0] = not blocked_candidate[0]
            hold_branch_applied = True
            hold_branch_terminal_error = zero_terminal_error
        eligible_t = torch.as_tensor(eligible, device=device, dtype=torch.bool)
        selection_costs = torch.where(
            eligible_t, costs, torch.full_like(costs, torch.inf))
        constrained_candidate_index = int(
            torch.argmin(selection_costs).detach().cpu())
        selected_reversal_mask = int(
            reversal_mask[constrained_candidate_index])
        direction_lease_applied = (
            constrained_candidate_index != unrestricted_candidate_index)
        minimum = selection_costs.min()
        # The learned model's displacement over 240 ms varies strongly with
        # operating point. Normalize the rollout spread so one temperature is
        # usable both near a slow distal equilibrium and during insertion.
        eligible_costs = costs[eligible_t]
        cost_scale = eligible_costs.std(unbiased=False).clamp_min(1e-6)
        logits = -(costs-minimum)/(self.config.temperature*cost_scale)
        logits = torch.where(
            eligible_t, logits, torch.full_like(logits, -torch.inf))
        weights = torch.softmax(logits, dim=0)
        # A second exact rollout of the weighted command added a 14 ms median
        # compute path and caused real 60 ms deadline faults. The weighted
        # feasible mean is retained only as the next sampling distribution's
        # nominal. When the execution guard is enabled, the actual command is
        # one already-scored candidate (including deterministic candidate 0).
        weighted_tips = torch.sum(
            weights[:, None, None]*mean_tips, dim=0)
        # As in the reference controller's g(v) path, update the nominal from
        # the feasible controls that were actually evaluated, never from the
        # raw Gaussian requests. Averaging raw saturated requests biases the
        # nominal and can reintroduce a limit-violating direction.
        weighted_feasible_t = torch.sum(
            weights[:, None, None]
            * logical_t[..., :CONTROL_AXES], dim=0)
        best_t = torch.argmin(selection_costs)
        best_tips_t = mean_tips[best_t]
        best_cost_t = costs[best_t]
        weighted_tracking_cost_t = torch.sum(weights*tracking_costs)
        zero_tracking_cost_t = tracking_costs[0]
        best_tracking_cost_t = tracking_costs[best_t]
        # Candidate zero is deterministic. Under guarded execution, choose
        # the minimum-cost candidate from the evaluated batch instead of an
        # unscored convex average. Since zero participates in the same total
        # objective, this cannot select a sampled command costlier than hold.
        guard_t = torch.as_tensor(
            (self.config.best_candidate_guard
             or (self.config.grouped_mode_sampling
                 and bool(np.any(lease_direction)))),
            device=device, dtype=torch.bool)
        updated_feasible_t = torch.where(
            guard_t, logical_t[best_t, :, :CONTROL_AXES],
            weighted_feasible_t)
        command_tips = torch.where(
            guard_t, best_tips_t, weighted_tips)
        effective_samples_t = 1.0/torch.sum(weights.square())
        # Complete all accelerator work once, then transfer only the compact
        # result needed by the CPU safety projection and ROS diagnostics.
        _synchronize_torch_device(device)
        costs_finite = bool(costs_finite_t.detach().cpu())
        updated_feasible = (
            updated_feasible_t.detach().cpu().numpy().copy())
        command_tips_np = (
            command_tips[:horizon].detach().cpu().numpy().copy())
        best_tips_np = (
            best_tips_t[:horizon].detach().cpu().numpy().copy())
        best_cost = float(best_cost_t.detach().cpu())
        scored_candidate_guard_applied = bool(guard_t.detach().cpu())
        blocked_candidate_count = int(
            blocked_candidate_t.sum().detach().cpu())
        lease_constrained_candidate_index = constrained_candidate_index
        unrestricted_total_cost = float(
            costs[unrestricted_best_t].detach().cpu())
        lease_constrained_total_cost = float(
            costs[constrained_candidate_index].detach().cpu())
        selected_candidate_index = (
            int(best_t.detach().cpu())
            if scored_candidate_guard_applied else -1)
        best_candidate_selected = bool(
            scored_candidate_guard_applied and selected_candidate_index != 0)
        selected_total_cost = float(
            (best_cost_t if scored_candidate_guard_applied else torch.sum(
                weights*costs)).detach().cpu())
        zero_total_cost = float(costs[0].detach().cpu())
        weighted_tracking_cost = float(
            weighted_tracking_cost_t.detach().cpu())
        zero_tracking_cost = float(zero_tracking_cost_t.detach().cpu())
        best_tracking_cost = float(best_tracking_cost_t.detach().cpu())
        effective_samples = float(effective_samples_t.detach().cpu())
        cost_weighting_ms = 1e3*(perf_counter()-phase_started)
        if not costs_finite:
            return self._safe_result(
                started, "invalid_cost",
                phase_timings_ms=(sample_projection_ms, rollout_ms,
                                  cost_weighting_ms, 0.0))
        phase_started = perf_counter()
        # The minimum-speed rule makes the feasible set non-convex around
        # zero. Do not snap a small weighted residual into physical motion.
        deadband = self.contract.velocity_min[:CONTROL_AXES]
        executable_request = np.where(
            np.abs(updated_feasible) < deadband[None], 0.0,
            updated_feasible)
        updated_logical, _, updated_motor, _, _, _ = self._project(
            executable_request[None], position, blocked_direction)
        takeup_direction_latched = False
        compensated_motor = updated_motor[0].copy()
        transmitted_motor = updated_motor[0].copy()
        update_projection_ms = 1e3*(perf_counter()-phase_started)

        elapsed = max(0.0, self._clock() - started)
        if elapsed > self.config.planning_deadline_s:
            return self._safe_result(
                started, "deadline_missed",
                phase_timings_ms=(sample_projection_ms, rollout_ms,
                                  cost_weighting_ms,
                                  update_projection_ms))

        warm = np.clip(
            updated_feasible,
            -self.contract.velocity_max[:CONTROL_AXES],
            self.contract.velocity_max[:CONTROL_AXES])
        self._nominal[:-1] = warm[1:]
        self._nominal[-1] = warm[-1]
        return MppiPlan(
            command_logical_velocity=updated_logical[0, 0].copy(),
            logical_velocity_sequence=updated_logical[0].copy(),
            motor_radians_per_second_sequence=updated_motor[0].copy(),
            best_tip_sequence_m=best_tips_np,
            best_cost=best_cost,
            effective_samples=effective_samples,
            elapsed_s=elapsed, valid=True, reason="ok",
            sample_projection_ms=sample_projection_ms,
            rollout_ms=rollout_ms,
            cost_weighting_ms=cost_weighting_ms,
            update_projection_ms=update_projection_ms,
            command_tip_sequence_m=command_tips_np,
            compensated_motor_radians_per_second_sequence=(
                compensated_motor),
            transmitted_motor_radians_per_second_sequence=(
                transmitted_motor),
            transmission_prediction_applied=(
                transmission_prediction_applied),
            rotation_direction_latched=False,
            takeup_direction_latched=takeup_direction_latched,
            best_candidate_selected=best_candidate_selected,
            weighted_tracking_cost=weighted_tracking_cost,
            zero_tracking_cost=zero_tracking_cost,
            best_tracking_cost=best_tracking_cost,
            selected_candidate_index=selected_candidate_index,
            selected_total_cost=selected_total_cost,
            zero_total_cost=zero_total_cost,
            scored_candidate_guard_applied=scored_candidate_guard_applied,
            blocked_motor_direction=blocked_direction.copy(),
            blocked_candidate_count=blocked_candidate_count,
            direction_lease=lease_direction.copy(),
            approved_reversal_direction=approved_direction.copy(),
            proposed_reversal_direction=(
                proposed_reversal_direction.copy()),
            direction_lease_applied=direction_lease_applied,
            unrestricted_candidate_index=unrestricted_candidate_index,
            lease_constrained_candidate_index=(
                lease_constrained_candidate_index),
            unrestricted_total_cost=unrestricted_total_cost,
            lease_constrained_total_cost=lease_constrained_total_cost,
            reversal_axis_cost_improvement=(
                reversal_axis_cost_improvement.copy()),
            reversal_axis_terminal_error_improvement_mm=(
                reversal_axis_terminal_improvement.copy()),
            hold_branch_applied=hold_branch_applied,
            hold_branch_terminal_error_mm=hold_branch_terminal_error,
            zero_terminal_error_mm=zero_terminal_error,
            raw_zero_terminal_error_mm=raw_zero_terminal_error,
            capture_passive_response_scale=capture_response_scale,
            proposal_group_count=proposal_group_count,
            selected_reversal_mask=selected_reversal_mask,
            unrestricted_reversal_mask=unrestricted_reversal_mask,
            mode_best_total_cost=mode_best_total_cost.copy(),
            takeup_joint_position_offset=(
                takeup_position_offset[int(best_t.detach().cpu())].copy()),
            selected_takeup_risk_s=float(
                takeup_risk_s[int(best_t.detach().cpu())]),
            selected_takeup_risk_cost=float(
                self.config.takeup_risk_weight
                * takeup_risk_s[int(best_t.detach().cpu())]),
            selected_switch_count=int(
                switch_count[int(best_t.detach().cpu())]),
            engaged_gain_scenario_count=gain_scenario_count,
            selected_engaged_gain_scenarios=(
                gain_scenarios[int(best_t.detach().cpu())].copy()),
            selected_gain_tracking_costs=(
                scenario_tracking_costs[best_t].detach().cpu().numpy()
                .reshape(-1).copy()),
            selected_maximum_first_step_lambda_shift=float(
                first_step_lambda_shift[int(best_t.detach().cpu())]),
            selected_gain_learning_velocity_scale=float(
                gain_learning_velocity_scales[
                    int(best_t.detach().cpu())]),
            prediction_horizon_steps=prediction_horizon,
            prediction_horizon_s=float(np.sum(rollout_dt)),
            tendon_probe_candidate_count=int(
                np.count_nonzero(self._last_tendon_probe_mask)),
            selected_tendon_probe=bool(
                self._last_tendon_probe_mask[
                    int(best_t.detach().cpu())])
        )
