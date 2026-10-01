"""Response-clocked arbitration for costly backlash direction reversals."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


CONTROL_AXES = 3


def _directions(name, value):
    result = np.asarray(value, dtype=np.int8)
    if result.shape != (CONTROL_AXES,) or np.any(np.abs(result) > 1):
        raise ValueError(f"{name} must contain three direction signs")
    return result


@dataclass(frozen=True)
class ReversalSchedulerConfig:
    enabled: bool = True
    required_plans: int = 3
    minimum_absolute_cost_improvement: float = 5.0
    minimum_fractional_cost_improvement: float = 0.0
    minimum_terminal_error_improvement_mm: float = 0.25
    minimum_accepted_observations: int = 3
    cooldown_s: float = 1.0

    def __post_init__(self):
        if self.required_plans < 1:
            raise ValueError("required_plans must be positive")
        if self.minimum_accepted_observations < 0:
            raise ValueError(
                "minimum_accepted_observations must be nonnegative")
        finite_nonnegative = (
            self.minimum_absolute_cost_improvement,
            self.minimum_fractional_cost_improvement,
            self.minimum_terminal_error_improvement_mm,
            self.cooldown_s,
        )
        if any(not np.isfinite(value) or value < 0.0
               for value in finite_nonnegative):
            raise ValueError("reversal scheduler thresholds must be finite "
                             "and nonnegative")


@dataclass(frozen=True)
class ReversalSchedulerSnapshot:
    lease_direction: np.ndarray
    pending_direction: np.ndarray
    pending_count: np.ndarray
    approved_direction: np.ndarray
    observations_since_lease: np.ndarray
    lease_age_s: np.ndarray
    absolute_cost_improvement: float = 0.0
    fractional_cost_improvement: float = 0.0
    terminal_error_improvement_mm: np.ndarray | None = None
    reason: str = "no_reversal_requested"


class ReversalDirectionScheduler:
    """Require stable evidence before opening an opposite take-up macro.

    A direction lease is acquired from observation-confirmed backlash state.
    MPPI may continue to evaluate reversing candidates, but only a previously
    approved direction is executable.  This class owns intent persistence;
    the backlash estimator remains the authority on physical engagement.
    """

    def __init__(self, config: ReversalSchedulerConfig):
        self.config = config
        self.lease_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.pending_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.pending_count = np.zeros(CONTROL_AXES, dtype=np.int32)
        self.approved_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.lease_observation = np.zeros(CONTROL_AXES, dtype=np.int64)
        self.lease_time_s = np.zeros(CONTROL_AXES, dtype=np.float64)
        self.last_absolute_improvement = 0.0
        self.last_fractional_improvement = 0.0
        self.last_terminal_improvement_mm = np.zeros(
            CONTROL_AXES, dtype=np.float64)
        self.reason = "disabled" if not config.enabled else (
            "no_reversal_requested")

    def reset(self):
        self.lease_direction.fill(0)
        self.reset_intent()
        self.lease_observation.fill(0)
        self.lease_time_s.fill(0.0)

    def reset_intent(self):
        self.pending_direction.fill(0)
        self.pending_count.fill(0)
        self.approved_direction.fill(0)
        self.last_absolute_improvement = 0.0
        self.last_fractional_improvement = 0.0
        self.last_terminal_improvement_mm.fill(0.0)
        self.reason = "disabled" if not self.config.enabled else (
            "no_reversal_requested")

    def synchronize(self, phase, engaged_direction, accepted_observations,
                    now_s):
        """Acquire leases only from response-confirmed estimator state."""
        phases = tuple(str(value) for value in phase)
        direction = _directions("engaged_direction", engaged_direction)
        if len(phases) != CONTROL_AXES:
            raise ValueError("phase must contain three entries")
        observation = int(accepted_observations)
        now = float(now_s)
        if observation < 0 or not np.isfinite(now):
            raise ValueError("invalid scheduler observation/time")
        if not self.config.enabled:
            return self.snapshot(observation, now)
        for axis in range(CONTROL_AXES):
            confirmed = phases[axis] in ("PROVISIONAL", "ENGAGED")
            if not confirmed or direction[axis] == 0:
                continue
            if self.lease_direction[axis] != direction[axis]:
                self.lease_direction[axis] = direction[axis]
                self.lease_observation[axis] = observation
                self.lease_time_s[axis] = now
                self.pending_direction[axis] = 0
                self.pending_count[axis] = 0
                self.approved_direction[axis] = 0
        return self.snapshot(observation, now)

    def observe_plan(self, proposed_reversal_direction,
                     axis_cost_improvement,
                     axis_terminal_error_improvement_mm,
                     accepted_observations, now_s):
        """Update intent from independently scored complete proposal modes.

        Grouped MPPI reports the same whole-mode improvement on every shaft
        participating in the proposed reversal mask. This scheduler adds only
        response-clocked persistence/cooldown authorization; it never edits a
        selected control sequence.
        """
        proposed = _directions(
            "proposed_reversal_direction", proposed_reversal_direction)
        observation = int(accepted_observations)
        now = float(now_s)
        cost_improvement = np.asarray(
            axis_cost_improvement, dtype=np.float64)
        terminal_improvement = np.asarray(
            axis_terminal_error_improvement_mm, dtype=np.float64)
        if (observation < 0 or not np.isfinite(now)
                or cost_improvement.shape != (CONTROL_AXES,)
                or terminal_improvement.shape != (CONTROL_AXES,)
                or not np.isfinite(cost_improvement).all()
                or not np.isfinite(terminal_improvement).all()):
            raise ValueError("invalid reversal scheduler plan evidence")
        if not self.config.enabled:
            return self.snapshot(observation, now)

        positive_cost = np.maximum(0.0, cost_improvement)
        self.last_absolute_improvement = float(np.max(positive_cost))
        self.last_fractional_improvement = 0.0
        self.last_terminal_improvement_mm = terminal_improvement.copy()
        margin_ok = (
            (positive_cost >= self.config.minimum_absolute_cost_improvement)
            & (terminal_improvement
               >= self.config.minimum_terminal_error_improvement_mm))

        for axis in range(CONTROL_AXES):
            requested = int(proposed[axis])
            if (requested == 0
                    or self.lease_direction[axis] == 0
                    or requested == self.lease_direction[axis]):
                self.pending_direction[axis] = 0
                self.pending_count[axis] = 0
                self.approved_direction[axis] = 0
                continue
            if self.pending_direction[axis] == requested:
                self.pending_count[axis] += 1
            else:
                self.pending_direction[axis] = requested
                self.pending_count[axis] = 1
                self.approved_direction[axis] = 0
            observations_ready = (
                observation-self.lease_observation[axis]
                >= self.config.minimum_accepted_observations)
            cooldown_ready = (
                now-self.lease_time_s[axis] >= self.config.cooldown_s)
            persistence_ready = (
                self.pending_count[axis] >= self.config.required_plans)
            if (margin_ok[axis] and observations_ready and cooldown_ready
                    and persistence_ready):
                self.approved_direction[axis] = requested

        proposed_mask = proposed != 0
        if not np.any(proposed_mask):
            self.reason = "no_reversal_requested"
        elif np.any(proposed_mask & ~margin_ok):
            self.reason = "reversal_cost_margin"
        elif np.any(
                (observation-self.lease_observation)[proposed_mask]
                < self.config.minimum_accepted_observations):
            self.reason = "reversal_observation_holdoff"
        elif np.any(
                (now-self.lease_time_s)[proposed_mask]
                < self.config.cooldown_s):
            self.reason = "reversal_cooldown"
        elif np.any(
                self.pending_count[proposed_mask]
                < self.config.required_plans):
            self.reason = "reversal_persistence"
        elif np.all(
                self.approved_direction[proposed_mask]
                == proposed[proposed_mask]):
            self.reason = "reversal_approved"
        else:
            self.reason = "reversal_constrained"
        return self.snapshot(observation, now)

    def mark_transaction_started(self, transaction_direction):
        """Consume approvals used to begin one bounded physical transaction."""
        direction = _directions(
            "transaction_direction", transaction_direction)
        started = (
            (self.approved_direction != 0)
            & (direction == self.approved_direction))
        self.approved_direction[started] = 0
        self.pending_direction[started] = 0
        self.pending_count[started] = 0
        if np.any(started):
            self.reason = "approved_reversal_transaction_started"

    def snapshot(self, accepted_observations=0, now_s=0.0):
        observation = int(accepted_observations)
        now = float(now_s)
        observations_since_lease = np.maximum(
            0, observation-self.lease_observation).astype(np.int64)
        lease_age_s = np.maximum(0.0, now-self.lease_time_s)
        inactive = self.lease_direction == 0
        observations_since_lease[inactive] = 0
        lease_age_s[inactive] = 0.0
        return ReversalSchedulerSnapshot(
            lease_direction=self.lease_direction.copy(),
            pending_direction=self.pending_direction.copy(),
            pending_count=self.pending_count.copy(),
            approved_direction=self.approved_direction.copy(),
            observations_since_lease=observations_since_lease,
            lease_age_s=lease_age_s,
            absolute_cost_improvement=self.last_absolute_improvement,
            fractional_cost_improvement=self.last_fractional_improvement,
            terminal_error_improvement_mm=(
                self.last_terminal_improvement_mm.copy()),
            reason=self.reason)
