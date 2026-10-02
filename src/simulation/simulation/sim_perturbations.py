"""Deterministic robustness perturbations for the isolated simulator."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


def _vector(name, value, length, *, nonnegative=False):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} must be nonnegative")
    return result.copy()


def _probability(name, value):
    result = float(value)
    if not np.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be within [0,1]")
    return result


@dataclass(frozen=True)
class MarkerPacket:
    release_timestamp_ns: int
    observation_timestamp_ns: int
    points_m: np.ndarray
    outlier_injected: bool


@dataclass(frozen=True)
class MarkerSensorConfig:
    seed: int = 0
    noise_std_m: float = 0.0
    common_bias_m: tuple = (0.0, 0.0, 0.0)
    marker_bias_m: tuple = (0.0,)*12
    latency_s: float = 0.0
    timestamp_jitter_s: float = 0.0
    dropout_probability: float = 0.0
    outlier_probability: float = 0.0
    outlier_magnitude_m: float = 0.0


class MarkerSensorModel:
    """Seeded marker corruption and causal release queue."""

    def __init__(self, config: MarkerSensorConfig):
        self.config = config
        self.noise_std_m = float(config.noise_std_m)
        self.common_bias_m = _vector(
            "common_bias_m", config.common_bias_m, 3)
        self.marker_bias_m = _vector(
            "marker_bias_m", config.marker_bias_m, 12).reshape(4, 3)
        self.latency_s = float(config.latency_s)
        self.timestamp_jitter_s = float(config.timestamp_jitter_s)
        self.dropout_probability = _probability(
            "dropout_probability", config.dropout_probability)
        self.outlier_probability = _probability(
            "outlier_probability", config.outlier_probability)
        self.outlier_magnitude_m = float(config.outlier_magnitude_m)
        values = (self.noise_std_m, self.latency_s,
                  self.timestamp_jitter_s, self.outlier_magnitude_m)
        if any(not np.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("sensor magnitudes and times must be nonnegative")
        self.rng = np.random.default_rng(int(config.seed))
        self._queue = deque()
        self.accepted_packets = 0
        self.dropped_packets = 0
        self.outlier_packets = 0

    def push(self, timestamp_ns, points_m):
        timestamp_ns = int(timestamp_ns)
        points = np.asarray(points_m, dtype=np.float64)
        if (timestamp_ns <= 0 or points.shape != (4, 3)
                or not np.all(np.isfinite(points))):
            raise ValueError("marker sample must be finite (4,3) at positive time")
        if self.rng.random() < self.dropout_probability:
            self.dropped_packets += 1
            return False
        observed = points+self.common_bias_m+self.marker_bias_m
        if self.noise_std_m:
            observed = observed+self.rng.normal(
                0.0, self.noise_std_m, observed.shape)
        outlier = self.rng.random() < self.outlier_probability
        if outlier and self.outlier_magnitude_m:
            marker = int(self.rng.integers(0, 4))
            direction = self.rng.normal(size=3)
            direction /= max(np.linalg.norm(direction), 1e-12)
            observed[marker] += self.outlier_magnitude_m*direction
            self.outlier_packets += 1
        jitter_ns = int(round(self.rng.normal(
            0.0, self.timestamp_jitter_s*1e9)))
        release_ns = timestamp_ns+int(round(self.latency_s*1e9))
        packet = MarkerPacket(
            release_ns, max(1, timestamp_ns+jitter_ns), observed, outlier)
        self._queue.append(packet)
        self.accepted_packets += 1
        return True

    def pop_ready(self, now_ns):
        ready = []
        while (self._queue
               and self._queue[0].release_timestamp_ns <= int(now_ns)):
            ready.append(self._queue.popleft())
        return ready

    @property
    def pending_packets(self):
        return len(self._queue)


@dataclass(frozen=True)
class ActuatorConfig:
    gain: tuple = (1.0,)*6
    deadband_rad_s: tuple = (0.0,)*6
    time_constant_s: tuple = (0.0,)*6
    command_delay_s: float = 0.0
    reversal_backlash_rad: tuple = (0.0,)*6
    reversal_backlash_positive_rad: tuple | None = None
    reversal_backlash_negative_rad: tuple | None = None
    initial_backlash_unengaged: bool = False


class ActuatorPerturbation:
    """Stateful achieved-motor-rate perturbation applied after projection."""

    def __init__(self, config: ActuatorConfig, step_s: float):
        self.gain = _vector("gain", config.gain, 6, nonnegative=True)
        self.deadband = _vector(
            "deadband_rad_s", config.deadband_rad_s, 6,
            nonnegative=True)
        self.time_constant = _vector(
            "time_constant_s", config.time_constant_s, 6,
            nonnegative=True)
        symmetric_backlash = _vector(
            "reversal_backlash_rad", config.reversal_backlash_rad, 6,
            nonnegative=True)
        if ((config.reversal_backlash_positive_rad is None)
                != (config.reversal_backlash_negative_rad is None)):
            raise ValueError(
                "positive and negative actuator backlash must be supplied "
                "together")
        positive_backlash = (
            None if config.reversal_backlash_positive_rad is None else
            _vector("reversal_backlash_positive_rad",
                    config.reversal_backlash_positive_rad, 6,
                    nonnegative=True))
        negative_backlash = (
            None if config.reversal_backlash_negative_rad is None else
            _vector("reversal_backlash_negative_rad",
                    config.reversal_backlash_negative_rad, 6,
                    nonnegative=True))
        directional_present = (
            positive_backlash is not None
            and np.any(np.r_[positive_backlash, negative_backlash] > 0.0))
        self.backlash_positive = (
            positive_backlash if directional_present
            else symmetric_backlash.copy())
        self.backlash_negative = (
            negative_backlash if directional_present
            else symmetric_backlash.copy())
        self.step_s = float(step_s)
        self.initial_backlash_unengaged = bool(
            config.initial_backlash_unengaged)
        delay = float(config.command_delay_s)
        if (not np.isfinite(self.step_s) or self.step_s <= 0.0
                or not np.isfinite(delay) or delay < 0.0):
            raise ValueError("actuator step and delay are invalid")
        self.delay_steps = int(round(delay/self.step_s))
        self._delay = deque(
            [np.zeros(6)]*self.delay_steps, maxlen=self.delay_steps+1)
        self._filtered = np.zeros(6)
        self._engaged_sign = np.zeros(6, dtype=np.int8)
        self._takeup_sign = np.zeros(6, dtype=np.int8)
        self._takeup_progress = np.zeros(6)

    def reset(self):
        """Reset both dynamic state and the simulated mechanical memory."""
        self._delay = deque(
            [np.zeros(6)]*self.delay_steps, maxlen=self.delay_steps+1)
        self._filtered.fill(0.0)
        self._engaged_sign.fill(0)
        self._takeup_sign.fill(0)
        self._takeup_progress.fill(0.0)

    def halt(self):
        """Stop motion immediately without erasing transmission position.

        A zero command or watchdog stop clears queued and filtered velocity,
        but it does not physically recenter a backlash gap.  Keeping the
        engagement and partial take-up state makes a later same-direction
        command resume from the actual simulated mechanical state.
        """
        self._delay = deque(
            [np.zeros(6)]*self.delay_steps, maxlen=self.delay_steps+1)
        self._filtered.fill(0.0)

    def _backlash_width(self, axis, direction):
        return float(
            self.backlash_positive[axis] if direction > 0
            else self.backlash_negative[axis])

    def _apply_backlash_axis(self, axis, rate, dt):
        direction = int(np.sign(rate))
        if direction == 0:
            return 0.0
        travel = abs(float(rate))*dt
        engaged = int(self._engaged_sign[axis])
        takeup = int(self._takeup_sign[axis])

        if engaged == 0:
            if not self.initial_backlash_unengaged:
                self._engaged_sign[axis] = direction
                return float(rate)
            if takeup != direction:
                # With no known contact side, a reversal cannot be localized
                # causally. Restart the conservative directional budget.
                self._takeup_sign[axis] = direction
                self._takeup_progress[axis] = 0.0
            width = self._backlash_width(axis, direction)
            required = max(0.0, width-self._takeup_progress[axis])
            blocked = min(required, travel)
            self._takeup_progress[axis] += blocked
            residual = travel-blocked
            if self._takeup_progress[axis] >= width-1e-12:
                self._engaged_sign[axis] = direction
                self._takeup_sign[axis] = 0
                self._takeup_progress[axis] = 0.0
            return direction*residual/dt

        if takeup == 0:
            if direction == engaged:
                return float(rate)
            self._takeup_sign[axis] = direction
            self._takeup_progress[axis] = 0.0
            takeup = direction

        if direction == takeup:
            width = self._backlash_width(axis, direction)
            required = max(0.0, width-self._takeup_progress[axis])
            blocked = min(required, travel)
            self._takeup_progress[axis] += blocked
            residual = travel-blocked
            if self._takeup_progress[axis] >= width-1e-12:
                self._engaged_sign[axis] = direction
                self._takeup_sign[axis] = 0
                self._takeup_progress[axis] = 0.0
            return direction*residual/dt

        # The command returned toward the last contacted side before reaching
        # the opposite side. Unwind only the partial gap travel already used;
        # do not charge another complete directional width.
        blocked = min(self._takeup_progress[axis], travel)
        self._takeup_progress[axis] -= blocked
        residual = travel-blocked
        if self._takeup_progress[axis] <= 1e-12:
            self._takeup_sign[axis] = 0
            self._takeup_progress[axis] = 0.0
        return direction*residual/dt

    def apply_components(self, projected_motor_rate, dt_s=None):
        """Return actual shaft rate and downstream transmitted rate.

        Delay, gain, deadband, and lag act before the encoder and therefore
        affect the measured motor shaft.  Backlash is downstream of that
        encoder: it can block interface motion without freezing ENC feedback.
        """
        requested = _vector(
            "projected_motor_rate", projected_motor_rate, 6)
        dt = self.step_s if dt_s is None else float(dt_s)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("actuator dt must be positive")
        self._delay.append(requested.copy())
        delayed = (self._delay.popleft() if self.delay_steps
                   else requested)
        target = delayed*self.gain
        target[np.abs(target) < self.deadband] = 0.0
        alpha = np.ones(6)
        positive_tau = self.time_constant > 0.0
        alpha[positive_tau] = 1.0-np.exp(
            -dt/self.time_constant[positive_tau])
        self._filtered += alpha*(target-self._filtered)
        shaft = self._filtered.copy()
        transmitted = np.asarray([
            self._apply_backlash_axis(axis, rate, dt)
            for axis, rate in enumerate(shaft)], dtype=np.float64)
        return shaft, transmitted

    def apply(self, projected_motor_rate, dt_s=None):
        """Backward-compatible downstream rate returned to existing callers."""
        return self.apply_components(projected_motor_rate, dt_s)[1]


@dataclass(frozen=True)
class JacobianConfig:
    angular_column_gain: tuple = (1.0, 1.0, 1.0)
    linear_column_gain: tuple = (1.0, 1.0, 1.0)


class JacobianPerturbation:
    def __init__(self, config: JacobianConfig):
        self.angular_gain = _vector(
            "angular_column_gain", config.angular_column_gain, 3)
        self.linear_gain = _vector(
            "linear_column_gain", config.linear_column_gain, 3)

    def apply(self, jacobian):
        result = np.asarray(jacobian, dtype=np.float64).copy()
        if result.shape != (6, 3) or not np.all(np.isfinite(result)):
            raise ValueError("Jacobian must be a finite 6x3 matrix")
        result[:3] *= self.angular_gain[None, :]
        result[3:] *= self.linear_gain[None, :]
        return result


def set_physical_jacobian(model, physical_jacobian):
    """Set a copied AdaptiveForwardJacobian while preserving its scales."""
    physical = np.asarray(physical_jacobian, dtype=np.float64)
    if physical.shape != model.jacobian.shape or not np.isfinite(physical).all():
        raise ValueError("physical Jacobian has the wrong shape or values")
    model.J_normalized = (
        physical*model.action_scale[None, :]/model.state_scale[:, None])
    return model
