"""Small, unit-explicit tracking diagnostics for closed-loop control."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


def tip_tracking_error_mm(target_tip_m, observed_tip_m):
    """Return target-minus-observation error vector and norm in millimetres."""
    target = np.asarray(target_tip_m, dtype=np.float64)
    observed = np.asarray(observed_tip_m, dtype=np.float64)
    if (target.shape != (3,) or observed.shape != (3,)
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(observed))):
        raise ValueError("tip positions must be finite three-vectors")
    error = 1e3*(target-observed)
    return error, float(np.linalg.vector_norm(error))


def _tip(name: str, value) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite three-vector")
    return result.copy()


def _optional_vector(name: str, value, size: int) -> np.ndarray | None:
    if value is None:
        return None
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return result.copy()


@dataclass(frozen=True)
class TipForecast:
    """One command rollout awaiting a horizon-aligned camera observation."""

    start_timestamp_ns: int
    start_observation_timestamp_ns: int
    due_timestamp_ns: int
    start_tip_m: np.ndarray
    predicted_terminal_tip_m: np.ndarray
    command_logical_velocity: np.ndarray | None = None
    motor_radians_per_second: np.ndarray | None = None
    capture_hold: bool = False
    joint_position: np.ndarray | None = None
    model_motor_angle_rad: np.ndarray | None = None
    interface_pose: np.ndarray | None = None
    interface_jacobian: np.ndarray | None = None
    target_tip_m: np.ndarray | None = None
    model_start_tip_m: np.ndarray | None = None


@dataclass(frozen=True)
class TipForecastResult:
    """Prediction-versus-observation response, with distances in mm."""

    start_timestamp_ns: int
    start_observation_timestamp_ns: int
    due_timestamp_ns: int
    observation_timestamp_ns: int
    predicted_delta_mm: np.ndarray
    measured_delta_mm: np.ndarray
    endpoint_error_mm: np.ndarray
    endpoint_error_norm_mm: float
    direction_cosine: float | None
    start_tip_m: np.ndarray | None = None
    predicted_terminal_tip_m: np.ndarray | None = None
    observed_tip_m: np.ndarray | None = None
    command_logical_velocity: np.ndarray | None = None
    motor_radians_per_second: np.ndarray | None = None
    joint_position: np.ndarray | None = None
    model_motor_angle_rad: np.ndarray | None = None
    capture_hold: bool = False
    interface_pose: np.ndarray | None = None
    interface_jacobian: np.ndarray | None = None
    target_tip_m: np.ndarray | None = None
    comparison_valid: bool = False
    comparison_reason: str = "unaligned_observations"
    source_observation_timestamp_ns: int = 0
    source_start_observation_timestamp_ns: int = 0
    model_start_tip_m: np.ndarray | None = None
    root_readout_error_mm: np.ndarray | None = None

    @property
    def start_observation_skew_ms(self) -> float:
        return 1e-6*float(
            self.start_timestamp_ns-self.start_observation_timestamp_ns)


class TipForecastMonitor:
    """Align forecasts with bracketed camera samples and published commands.

    Planning can be faster than the camera. If several forecasts mature before
    one observation, only the newest due forecast is evaluated. Interpolation
    is diagnostic, never an estimator observation or a new control target.
    A 100 ms bracket/heartbeat gap is the maximum supported diagnostic window;
    unsupported comparisons cannot drive passive capture response feedback.
    """

    def __init__(self, max_pending: int = 32):
        if int(max_pending) < 1:
            raise ValueError("max_pending must be positive")
        self._pending = deque(maxlen=int(max_pending))
        self._observations = deque(maxlen=128)
        self._commands = deque(maxlen=128)

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def clear(self) -> None:
        self._pending.clear()
        self._observations.clear()
        self._commands.clear()

    def note_command(self, timestamp_ns: int, velocity) -> None:
        """Record published intent, not motor-applied acknowledgement."""
        stamp = int(timestamp_ns)
        command = _optional_vector("velocity", velocity, 6)
        if stamp <= 0 or command is None:
            raise ValueError("command requires a positive timestamp and velocity")
        if self._commands and stamp < self._commands[-1][0]:
            self._commands.clear()  # clock discontinuity: coverage is unknown
        self._commands.append((stamp, command))

    def _interpolate_tip(self, timestamp_ns):
        previous = None
        for stamp, tip in self._observations:
            if stamp == timestamp_ns:
                return tip.copy()
            if stamp > timestamp_ns:
                if previous is None or stamp-previous[0] > 100_000_000:
                    return None
                fraction = (timestamp_ns-previous[0])/(stamp-previous[0])
                return previous[1]+fraction*(tip-previous[1])
            previous = (stamp, tip)
        return None  # no extrapolation through missing camera samples

    def _command_reason(self, forecast):
        if forecast.command_logical_velocity is None:
            return "missing_command_metadata"
        prior = [(t, v) for t, v in self._commands
                 if t <= forecast.start_timestamp_ns]
        if not prior:
            return "missing_published_command_at_root"
        commands = [prior[-1]] + [(t, v) for t, v in self._commands
            if forecast.start_timestamp_ns < t <= forecast.due_timestamp_ns]
        after = next((t for t, _ in self._commands
                      if t >= forecast.due_timestamp_ns), None)
        times = [t for t, _ in commands]
        if (after is None or max(np.diff(times+[after]), default=0)
                > 100_000_000):
            return "published_command_coverage_gap"
        if any(not np.allclose(v, forecast.command_logical_velocity,
                               rtol=0.0, atol=1e-12) for _, v in commands):
            return "published_command_changed"
        return "aligned_published_command"

    def add(self, start_timestamp_ns: int,
            start_observation_timestamp_ns: int, horizon_s: float,
            start_tip_m, predicted_terminal_tip_m, *,
            command_logical_velocity=None,
            motor_radians_per_second=None,
            joint_position=None,
            model_motor_angle_rad=None,
            interface_pose=None,
            interface_jacobian=None,
            target_tip_m=None,
            model_start_tip_m=None,
            capture_hold: bool = False) -> None:
        start = int(start_timestamp_ns)
        observation = int(start_observation_timestamp_ns)
        horizon = float(horizon_s)
        if start <= 0 or observation <= 0:
            raise ValueError("forecast timestamps must be positive")
        if not np.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("forecast horizon must be positive")
        due = start+int(round(1e9*horizon))
        anchor = _tip("start_tip_m", start_tip_m)
        if not self._observations or observation > self._observations[-1][0]:
            self._observations.append((observation, anchor))
        self._pending.append(TipForecast(
            start_timestamp_ns=start,
            start_observation_timestamp_ns=observation,
            due_timestamp_ns=due,
            start_tip_m=anchor,
            predicted_terminal_tip_m=_tip(
                "predicted_terminal_tip_m", predicted_terminal_tip_m),
            command_logical_velocity=_optional_vector(
                "command_logical_velocity", command_logical_velocity, 6),
            motor_radians_per_second=_optional_vector(
                "motor_radians_per_second", motor_radians_per_second, 6),
            joint_position=_optional_vector(
                "joint_position", joint_position, 6),
            model_motor_angle_rad=_optional_vector(
                "model_motor_angle_rad", model_motor_angle_rad, 3),
            interface_pose=_optional_vector(
                "interface_pose", interface_pose, 16),
            interface_jacobian=_optional_vector(
                "interface_jacobian", interface_jacobian, 18),
            target_tip_m=_optional_vector(
                "target_tip_m", target_tip_m, 3),
            model_start_tip_m=_optional_vector(
                "model_start_tip_m", model_start_tip_m, 3),
            capture_hold=bool(capture_hold)))

    def observe(self, timestamp_ns: int,
                observed_tip_m) -> TipForecastResult | None:
        timestamp = int(timestamp_ns)
        if timestamp <= 0:
            raise ValueError("observation timestamp must be positive")
        observed = _tip("observed_tip_m", observed_tip_m)
        if self._observations and timestamp < self._observations[-1][0]:
            return None
        if self._observations and timestamp == self._observations[-1][0]:
            self._observations[-1] = (timestamp, observed)
        else:
            self._observations.append((timestamp, observed))
        matured = None
        while self._pending and self._pending[0].due_timestamp_ns <= timestamp:
            matured = self._pending.popleft()
        if matured is None:
            return None

        aligned_start = self._interpolate_tip(matured.start_timestamp_ns)
        aligned_end = self._interpolate_tip(matured.due_timestamp_ns)
        aligned = aligned_start is not None and aligned_end is not None
        reason = (self._command_reason(matured) if aligned
                  else "unaligned_observations")
        if aligned and matured.model_start_tip_m is None:
            reason = "missing_model_root_readout"
        valid = aligned and reason == "aligned_published_command"
        # Unsupported rows retain raw values for inspection but cannot be
        # mistaken for a valid fixed-input direction comparison.
        start_tip = aligned_start if aligned else matured.start_tip_m
        end_tip = aligned_end if aligned else observed
        model_start = (matured.model_start_tip_m
                       if matured.model_start_tip_m is not None else start_tip)
        predicted_delta = 1e3*(matured.predicted_terminal_tip_m-model_start)
        measured_delta = 1e3*(end_tip-start_tip)
        endpoint_error = 1e3*(
            end_tip-matured.predicted_terminal_tip_m)
        predicted_norm = float(np.linalg.vector_norm(predicted_delta))
        measured_norm = float(np.linalg.vector_norm(measured_delta))
        direction_cosine = None
        if valid and predicted_norm > 1e-9 and measured_norm > 1e-9:
            direction_cosine = float(np.clip(
                np.dot(predicted_delta, measured_delta)
                /(predicted_norm*measured_norm), -1.0, 1.0))
        return TipForecastResult(
            start_timestamp_ns=matured.start_timestamp_ns,
            start_observation_timestamp_ns=(
                matured.start_timestamp_ns if aligned
                else matured.start_observation_timestamp_ns),
            due_timestamp_ns=matured.due_timestamp_ns,
            observation_timestamp_ns=(matured.due_timestamp_ns if aligned else timestamp),
            predicted_delta_mm=predicted_delta,
            measured_delta_mm=measured_delta,
            endpoint_error_mm=endpoint_error,
            endpoint_error_norm_mm=float(
                np.linalg.vector_norm(endpoint_error)),
            direction_cosine=direction_cosine,
            start_tip_m=start_tip.copy(),
            predicted_terminal_tip_m=(
                matured.predicted_terminal_tip_m.copy()),
            observed_tip_m=end_tip.copy(),
            command_logical_velocity=matured.command_logical_velocity,
            motor_radians_per_second=matured.motor_radians_per_second,
            joint_position=matured.joint_position,
            model_motor_angle_rad=matured.model_motor_angle_rad,
            interface_pose=matured.interface_pose,
            interface_jacobian=matured.interface_jacobian,
            target_tip_m=matured.target_tip_m,
            capture_hold=matured.capture_hold,
            comparison_valid=valid, comparison_reason=reason,
            source_observation_timestamp_ns=timestamp,
            source_start_observation_timestamp_ns=matured.start_observation_timestamp_ns,
            model_start_tip_m=matured.model_start_tip_m,
            root_readout_error_mm=(1e3*(model_start-start_tip)
                                   if aligned and matured.model_start_tip_m is not None
                                   else None))
