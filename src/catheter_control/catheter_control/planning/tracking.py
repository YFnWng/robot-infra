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

    @property
    def start_observation_skew_ms(self) -> float:
        return 1e-6*float(
            self.start_timestamp_ns-self.start_observation_timestamp_ns)


class TipForecastMonitor:
    """Match command forecasts to the first accepted camera sample after due.

    Planning can be faster than the camera. If several forecasts mature before
    one observation, only the newest due forecast is evaluated; older ones
    would have a larger and less meaningful timestamp mismatch.
    """

    def __init__(self, max_pending: int = 32):
        if int(max_pending) < 1:
            raise ValueError("max_pending must be positive")
        self._pending = deque(maxlen=int(max_pending))

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def clear(self) -> None:
        self._pending.clear()

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
            capture_hold: bool = False) -> None:
        start = int(start_timestamp_ns)
        observation = int(start_observation_timestamp_ns)
        horizon = float(horizon_s)
        if start <= 0 or observation <= 0:
            raise ValueError("forecast timestamps must be positive")
        if not np.isfinite(horizon) or horizon <= 0.0:
            raise ValueError("forecast horizon must be positive")
        due = start+int(round(1e9*horizon))
        self._pending.append(TipForecast(
            start_timestamp_ns=start,
            start_observation_timestamp_ns=observation,
            due_timestamp_ns=due,
            start_tip_m=_tip("start_tip_m", start_tip_m),
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
            capture_hold=bool(capture_hold)))

    def observe(self, timestamp_ns: int,
                observed_tip_m) -> TipForecastResult | None:
        timestamp = int(timestamp_ns)
        if timestamp <= 0:
            raise ValueError("observation timestamp must be positive")
        observed = _tip("observed_tip_m", observed_tip_m)
        matured = None
        while self._pending and self._pending[0].due_timestamp_ns <= timestamp:
            matured = self._pending.popleft()
        if matured is None:
            return None

        predicted_delta = 1e3*(
            matured.predicted_terminal_tip_m-matured.start_tip_m)
        measured_delta = 1e3*(observed-matured.start_tip_m)
        endpoint_error = 1e3*(
            observed-matured.predicted_terminal_tip_m)
        predicted_norm = float(np.linalg.vector_norm(predicted_delta))
        measured_norm = float(np.linalg.vector_norm(measured_delta))
        direction_cosine = None
        if predicted_norm > 1e-9 and measured_norm > 1e-9:
            direction_cosine = float(np.clip(
                np.dot(predicted_delta, measured_delta)
                /(predicted_norm*measured_norm), -1.0, 1.0))
        return TipForecastResult(
            start_timestamp_ns=matured.start_timestamp_ns,
            start_observation_timestamp_ns=(
                matured.start_observation_timestamp_ns),
            due_timestamp_ns=matured.due_timestamp_ns,
            observation_timestamp_ns=timestamp,
            predicted_delta_mm=predicted_delta,
            measured_delta_mm=measured_delta,
            endpoint_error_mm=endpoint_error,
            endpoint_error_norm_mm=float(
                np.linalg.vector_norm(endpoint_error)),
            direction_cosine=direction_cosine,
            start_tip_m=matured.start_tip_m.copy(),
            predicted_terminal_tip_m=(
                matured.predicted_terminal_tip_m.copy()),
            observed_tip_m=observed.copy(),
            command_logical_velocity=matured.command_logical_velocity,
            motor_radians_per_second=matured.motor_radians_per_second,
            joint_position=matured.joint_position,
            model_motor_angle_rad=matured.model_motor_angle_rad,
            interface_pose=matured.interface_pose,
            interface_jacobian=matured.interface_jacobian,
            target_tip_m=matured.target_tip_m,
            capture_hold=matured.capture_hold)
