"""Causal directional engaged-gain belief for distal tendon response."""
from __future__ import annotations

from dataclasses import dataclass
import copy
import math

import numpy as np


CONTROL_AXES = 3
DIRECTIONS = 2


@dataclass(frozen=True)
class EngagedGainConfig:
    """Configuration for one slow, positive, direction-specific gain."""

    enabled: bool = False
    tendon_axis: int = 2
    minimum_gain: float = 0.10
    maximum_gain: float = 2.0
    prior_mean: tuple[float, float] = (1.0, 1.0)
    prior_log_std: float = 0.70
    reversal_log_std: float = 0.80
    process_log_std_sqrt_s: float = 0.05
    observation_std: float = 0.08
    minimum_nominal_increment: float = 0.02
    huber_sigma: float = 3.0
    maximum_normalized_innovation: float = 8.0
    contradiction_log_std: float = 0.70
    confidence_log_width: float = 0.50
    minimum_updates: int = 2
    credible_sigma: float = 1.645

    def __post_init__(self):
        if not 0 <= self.tendon_axis < CONTROL_AXES:
            raise ValueError("tendon_axis must select a controlled shaft")
        if (not np.isfinite(self.minimum_gain)
                or not np.isfinite(self.maximum_gain)
                or self.minimum_gain <= 0.0
                or self.maximum_gain <= self.minimum_gain):
            raise ValueError("invalid engaged gain support")
        prior = np.asarray(self.prior_mean, dtype=np.float64)
        if (prior.shape != (DIRECTIONS,) or not np.isfinite(prior).all()
                or np.any(prior < self.minimum_gain)
                or np.any(prior > self.maximum_gain)):
            raise ValueError("prior_mean must contain two supported gains")
        positive = (
            self.prior_log_std, self.reversal_log_std,
            self.process_log_std_sqrt_s, self.observation_std,
            self.minimum_nominal_increment, self.huber_sigma,
            self.maximum_normalized_innovation, self.contradiction_log_std,
            self.confidence_log_width,
            self.credible_sigma)
        if not all(np.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("engaged gain scales must be finite and positive")
        if self.minimum_updates < 1:
            raise ValueError("minimum_updates must be positive")


@dataclass(frozen=True)
class EngagedGainSnapshot:
    """Immutable planner-facing directional gain posterior."""

    tendon_axis: int
    enabled: bool
    mean: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    log_variance: np.ndarray
    update_count: np.ndarray
    last_update_timestamp_ns: np.ndarray
    normalized_innovation: np.ndarray
    status: tuple[tuple[str, str], ...]
    active_direction: np.ndarray
    last_reason: tuple[str, str, str]

    def scenarios(self, axis: int, direction: int) -> np.ndarray:
        index = 1 if int(direction) > 0 else 0
        if int(direction) == 0:
            values = np.r_[
                self.mean[axis], self.lower[axis], self.upper[axis]]
            return np.asarray([
                float(np.mean(self.mean[axis])),
                float(np.min(values)),
                float(np.max(values))], dtype=np.float64)
        return np.asarray([
            self.mean[axis, index],
            self.lower[axis, index],
            self.upper[axis, index]], dtype=np.float64)


class EngagedGainEstimator:
    """Robust log-domain belief updated only by confirmed engaged response."""

    def __init__(self, config: EngagedGainConfig):
        self.config = config
        prior = np.log(np.asarray(config.prior_mean, dtype=np.float64))
        self.log_mean = np.broadcast_to(
            prior[None], (CONTROL_AXES, DIRECTIONS)).copy()
        self.log_variance = np.full(
            (CONTROL_AXES, DIRECTIONS), config.prior_log_std**2)
        self.update_count = np.zeros(
            (CONTROL_AXES, DIRECTIONS), dtype=np.int32)
        self.last_update_timestamp_ns = np.zeros(
            (CONTROL_AXES, DIRECTIONS), dtype=np.int64)
        self.normalized_innovation = np.zeros(
            (CONTROL_AXES, DIRECTIONS), dtype=np.float64)
        self.active_direction = np.zeros(CONTROL_AXES, dtype=np.int8)
        self.last_nominal = np.full(CONTROL_AXES, np.nan)
        self.last_observed = np.full(CONTROL_AXES, np.nan)
        self.last_observation_timestamp_ns = np.zeros(
            CONTROL_AXES, dtype=np.int64)
        self.last_reason = ["disabled" if not config.enabled else "unobserved"
                            for _ in range(CONTROL_AXES)]

    @staticmethod
    def _index(direction: int) -> int:
        value = int(np.sign(direction))
        if value == 0:
            raise ValueError("engaged gain direction must be nonzero")
        return 1 if value > 0 else 0

    def clone_state(self):
        return copy.deepcopy(self.__dict__)

    def restore_state(self, checkpoint):
        if not isinstance(checkpoint, dict):
            raise ValueError("engaged gain checkpoint must be a dictionary")
        config = self.config
        self.__dict__.clear()
        self.__dict__.update(copy.deepcopy(checkpoint))
        self.config = config

    def start_takeup(self, axis: int, direction: int):
        axis = int(axis)
        index = self._index(direction)
        self.active_direction[axis] = int(np.sign(direction))
        self.last_nominal[axis] = np.nan
        self.last_observed[axis] = np.nan
        self.last_observation_timestamp_ns[axis] = 0
        self.log_variance[axis, index] = max(
            self.log_variance[axis, index],
            self.config.reversal_log_std**2)
        self.last_reason[axis] = "awaiting_engagement"

    def confirm(self, axis: int, direction: int, nominal_lambda,
                observed_lambda, timestamp_ns):
        axis = int(axis)
        self._index(direction)
        nominal = float(nominal_lambda)
        observed = float(observed_lambda)
        if not np.isfinite(nominal) or not np.isfinite(observed):
            self.last_reason[axis] = "nonfinite_anchor"
            return
        self.active_direction[axis] = int(np.sign(direction))
        self.last_nominal[axis] = nominal
        self.last_observed[axis] = observed
        self.last_observation_timestamp_ns[axis] = int(timestamp_ns or 0)
        self.last_reason[axis] = "engaged_anchor"

    def _reject_and_reanchor(self, axis, index, nominal, observed,
                             timestamp, reason):
        """Forget false confidence after evidence contradicts the branch.

        Reanchoring prevents every later observation from being compared with
        the same stale pre-transient point. The directional mean is retained,
        but the posterior must earn confidence again from fresh local motion.
        """
        self.log_variance[axis, index] = max(
            self.log_variance[axis, index],
            self.config.contradiction_log_std**2)
        self.update_count[axis, index] = 0
        self.last_nominal[axis] = nominal
        self.last_observed[axis] = observed
        self.last_observation_timestamp_ns[axis] = timestamp
        self.last_reason[axis] = reason

    def observe(self, axis: int, direction: int, nominal_lambda,
                observed_lambda, timestamp_ns, *, engaged: bool):
        """Update from posterior versus pre-correction nominal lambda change."""
        axis = int(axis)
        if not self.config.enabled or axis != self.config.tendon_axis:
            self.last_reason[axis] = "disabled"
            return False
        direction = int(np.sign(direction))
        if not engaged or direction == 0:
            self.last_reason[axis] = "not_engaged"
            return False
        nominal = float(nominal_lambda)
        observed = float(observed_lambda)
        timestamp = int(timestamp_ns or 0)
        if not np.isfinite(nominal) or not np.isfinite(observed):
            self.last_reason[axis] = "nonfinite_observation"
            return False
        if (self.active_direction[axis] != direction
                or not np.isfinite(self.last_nominal[axis])
                or not np.isfinite(self.last_observed[axis])):
            self.confirm(axis, direction, nominal, observed, timestamp)
            return False

        index = self._index(direction)
        previous_timestamp = self.last_observation_timestamp_ns[axis]
        dt_s = (0.0 if previous_timestamp <= 0 or timestamp <= previous_timestamp
                else (timestamp-previous_timestamp)*1e-9)
        self.log_variance[axis, index] += (
            self.config.process_log_std_sqrt_s**2*dt_s)

        phi = nominal-self.last_nominal[axis]
        response = observed-self.last_observed[axis]
        if abs(phi) < self.config.minimum_nominal_increment:
            self.last_reason[axis] = "insufficient_nominal_excitation"
            return False

        mean = self.log_mean[axis, index]
        variance = self.log_variance[axis, index]
        predicted = math.exp(mean)*phi
        jacobian = predicted
        innovation = response-predicted
        innovation_variance = (
            jacobian*jacobian*variance+self.config.observation_std**2)
        sigma = math.sqrt(max(innovation_variance, 1e-12))
        normalized = innovation/sigma
        self.normalized_innovation[axis, index] = normalized

        # Opposite signed, resolved response is not allowed to pull a positive
        # gain through zero. Preserve the branch and increase uncertainty.
        if (response*phi < 0.0
                and abs(response) >= self.config.observation_std):
            self._reject_and_reanchor(
                axis, index, nominal, observed, timestamp,
                "contradictory_response")
            return False

        if abs(normalized) > self.config.maximum_normalized_innovation:
            self._reject_and_reanchor(
                axis, index, nominal, observed, timestamp,
                "innovation_rejected")
            return False

        robust = min(1.0, self.config.huber_sigma/max(abs(normalized), 1e-12))
        gain = variance*jacobian/max(innovation_variance, 1e-12)
        updated_mean = mean+gain*robust*innovation
        updated_variance = max(
            1e-8, (1.0-gain*jacobian)*variance)
        support = np.log([
            self.config.minimum_gain, self.config.maximum_gain])
        self.log_mean[axis, index] = float(np.clip(
            updated_mean, support[0], support[1]))
        self.log_variance[axis, index] = float(updated_variance)
        self.update_count[axis, index] += 1
        self.last_update_timestamp_ns[axis, index] = timestamp
        self.last_nominal[axis] = nominal
        self.last_observed[axis] = observed
        self.last_observation_timestamp_ns[axis] = timestamp
        self.last_reason[axis] = "updated"
        return True

    def snapshot(self) -> EngagedGainSnapshot:
        sigma = np.sqrt(np.maximum(self.log_variance, 0.0))
        mean = np.exp(self.log_mean)
        lower = np.exp(self.log_mean-self.config.credible_sigma*sigma)
        upper = np.exp(self.log_mean+self.config.credible_sigma*sigma)
        lower = np.clip(
            lower, self.config.minimum_gain, self.config.maximum_gain)
        upper = np.clip(
            upper, self.config.minimum_gain, self.config.maximum_gain)
        status = []
        for axis in range(CONTROL_AXES):
            row = []
            for direction in range(DIRECTIONS):
                if not self.config.enabled or axis != self.config.tendon_axis:
                    value = "DISABLED"
                elif (abs(self.normalized_innovation[axis, direction])
                      > self.config.maximum_normalized_innovation):
                    value = "DEGRADED"
                elif (self.update_count[axis, direction]
                      < self.config.minimum_updates):
                    value = ("UNOBSERVED"
                             if self.update_count[axis, direction] == 0
                             else "LEARNING")
                elif (math.log(upper[axis, direction])
                      - math.log(lower[axis, direction])
                      <= self.config.confidence_log_width):
                    value = "CONFIDENT"
                else:
                    value = "LEARNING"
                row.append(value)
            status.append(tuple(row))
        return EngagedGainSnapshot(
            tendon_axis=self.config.tendon_axis,
            enabled=self.config.enabled,
            mean=mean.copy(), lower=lower.copy(), upper=upper.copy(),
            log_variance=self.log_variance.copy(),
            update_count=self.update_count.copy(),
            last_update_timestamp_ns=self.last_update_timestamp_ns.copy(),
            normalized_innovation=self.normalized_innovation.copy(),
            status=tuple(status),
            active_direction=self.active_direction.copy(),
            last_reason=tuple(self.last_reason))
