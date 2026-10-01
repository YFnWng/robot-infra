"""ROS-independent exact-model plant for catheter MPPI simulation."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..safety.hardware_contract import (
    ENCODER_RADIANS_PER_COUNT, HardwareContract,
    MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM, N_AXES)


def _vector(name: str, value) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (N_AXES,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    return result


def position_servo_velocity(current, target, speed, tolerance,
                            step_s: float) -> np.ndarray:
    """Resolve one bounded velocity step for a simulated POS transaction.

    The real firmware executes an absolute position transaction at the six
    supplied speed magnitudes. The model-in-loop device has a velocity-driven
    plant, so it reproduces that contract with a stop-within-tolerance servo.
    Encoder quantization is still applied by the plant itself.
    """
    position = _vector("current position", current)
    destination = _vector("position target", target)
    maximum = _vector("position speed", speed)
    deadband = _vector("position tolerance", tolerance)
    if np.any(maximum < 0.0):
        raise ValueError("position speed must be nonnegative")
    if np.any(deadband <= 0.0):
        raise ValueError("position tolerance must be positive")
    if not np.isfinite(step_s) or step_s <= 0.0:
        raise ValueError("step_s must be finite and strictly positive")
    error = destination-position
    moving = np.abs(error) > deadband
    if np.any(moving & (maximum <= 0.0)):
        axes = np.flatnonzero(moving & (maximum <= 0.0)).tolist()
        raise ValueError(
            f"nonzero position error has zero speed on axes {axes}")
    velocity = np.zeros(N_AXES, dtype=np.float64)
    velocity[moving] = (
        np.sign(error[moving])
        * np.minimum(maximum[moving], np.abs(error[moving])/float(step_s)))
    return velocity


@dataclass(frozen=True)
class ActuatorSnapshot:
    """Count-derived feedback from the lightweight simulated device."""

    timestamp_ns: int
    encoder_counts: np.ndarray
    joint_position: np.ndarray
    motor_angle_rad: np.ndarray
    requested_logical_velocity: np.ndarray
    projected_logical_velocity: np.ndarray
    realized_logical_velocity: np.ndarray
    motor_radians_per_second: np.ndarray
    transmitted_encoder_counts: np.ndarray
    transmitted_motor_angle_rad: np.ndarray
    transmitted_motor_radians_per_second: np.ndarray


class SimulatedActuatorPlant:
    """Firmware-side integration with no learned-model dependency."""

    def __init__(self, contract: HardwareContract, *, timestamp_ns: int,
                 step_s: float = 0.01, initial_encoder_counts=None,
                 actuator_perturbation=None):
        if not np.isfinite(step_s) or step_s <= 0.0:
            raise ValueError("step_s must be finite and strictly positive")
        if int(timestamp_ns) <= 0:
            raise ValueError("timestamp_ns must be positive")
        counts = _vector(
            "initial_encoder_counts",
            np.zeros(N_AXES) if initial_encoder_counts is None
            else initial_encoder_counts)
        rounded = np.rint(counts)
        if not np.array_equal(counts, rounded):
            raise ValueError("initial_encoder_counts must be integer-valued")
        if not contract.model_encoder_counts_are_valid(rounded):
            raise ValueError("initial encoder counts are outside model bounds")
        position = contract.encoder_counts_to_logical_position(rounded)
        if not contract.position_is_valid(position):
            raise ValueError("initial encoder counts imply an invalid position")
        self.contract = contract
        self.actuator_perturbation = actuator_perturbation
        self.step_s = float(step_s)
        self.step_ns = int(round(self.step_s*1e9))
        self.timestamp_ns = int(timestamp_ns)
        self.encoder_counts = rounded.astype(np.float64)
        self.motor_angle_rad = (
            self.encoder_counts*ENCODER_RADIANS_PER_COUNT)
        self.transmitted_motor_angle_rad = self.motor_angle_rad.copy()
        self.joint_position = position
        self.requested_velocity = np.zeros(N_AXES, dtype=np.float64)
        self.last_projection = contract.project_velocity(
            self.requested_velocity, self.joint_position)

    def set_command(self, logical_velocity) -> None:
        self.requested_velocity = _vector(
            "logical_velocity", logical_velocity).copy()

    def stop(self) -> None:
        self.requested_velocity.fill(0.0)
        if self.actuator_perturbation is not None:
            self.actuator_perturbation.halt()
        self.last_projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)

    def reset_transmission(self) -> None:
        """Reset simulation-only hidden transmission memory at this shaft pose.

        Shaft encoder counts and logical joint position are preserved.  The
        transmitted coordinate is re-registered to that measured shaft pose,
        and all actuator delay/backlash memory is returned to its configured
        initial condition.  This has no hardware analogue.
        """
        self.requested_velocity.fill(0.0)
        self.transmitted_motor_angle_rad = self.motor_angle_rad.copy()
        if self.actuator_perturbation is not None:
            self.actuator_perturbation.reset()
        self.last_projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)

    def step(self, timestamp_ns: int | None = None) -> ActuatorSnapshot:
        target_ns = (self.timestamp_ns+self.step_ns
                     if timestamp_ns is None else int(timestamp_ns))
        if target_ns <= self.timestamp_ns:
            raise ValueError("plant timestamps must be strictly increasing")
        dt = (target_ns-self.timestamp_ns)*1e-9
        projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)
        shaft_rate = projection.motor_radians_per_second
        transmitted_rate = shaft_rate
        if self.actuator_perturbation is not None:
            shaft_rate, transmitted_rate = (
                self.actuator_perturbation.apply_components(shaft_rate, dt))
        next_angle = self.contract.integrate_motor_angles(
            self.motor_angle_rad, shaft_rate, dt)
        next_transmitted_angle = self.contract.integrate_motor_angles(
            self.transmitted_motor_angle_rad, transmitted_rate, dt)
        next_counts = np.rint(
            next_angle/ENCODER_RADIANS_PER_COUNT).astype(np.float64)
        next_transmitted_counts = np.rint(
            next_transmitted_angle/ENCODER_RADIANS_PER_COUNT).astype(np.float64)
        next_position = self.contract.encoder_counts_to_logical_position(
            next_counts)
        if (not self.contract.model_encoder_counts_are_valid(next_counts)
                or not self.contract.position_is_valid(next_position)):
            self.stop()
            raise RuntimeError("simulated feedback left configured limits")
        self.timestamp_ns = target_ns
        self.encoder_counts = next_counts
        self.motor_angle_rad = next_angle
        self.transmitted_motor_angle_rad = next_transmitted_angle
        self.joint_position = next_position
        self.last_projection = projection
        motor_axis_rate = (
            transmitted_rate/(2.0*np.pi/60.0)
            * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM)
        realized_logical = self.contract.motor_axis_to_logical_velocity(
            motor_axis_rate)
        return ActuatorSnapshot(
            target_ns, next_counts.copy(), next_position.copy(),
            next_angle.copy(), self.requested_velocity.copy(),
            projection.realized_logical_velocity.copy(),
            realized_logical.copy(), shaft_rate.copy(),
            next_transmitted_counts.copy(), next_transmitted_angle.copy(),
            transmitted_rate.copy())


@dataclass(frozen=True)
class PlantSnapshot:
    """Immutable outputs from one simulated firmware/model tick."""

    timestamp_ns: int
    encoder_counts: np.ndarray
    joint_position: np.ndarray
    motor_angle_rad: np.ndarray
    markers_base_m: np.ndarray
    tip_base_m: np.ndarray
    requested_logical_velocity: np.ndarray
    projected_logical_velocity: np.ndarray
    realized_logical_velocity: np.ndarray
    motor_radians_per_second: np.ndarray
    transmitted_encoder_counts: np.ndarray
    transmitted_motor_angle_rad: np.ndarray
    transmitted_motor_radians_per_second: np.ndarray


class ModelInLoopPlant:
    """Advance an independent v171 runtime from simulated encoder counts.

    The continuous motor angle accumulates sub-count motion. Feedback and the
    learned runtime see integer counts, matching the Teensy encoder boundary.
    POS is reconstructed from those same counts instead of being integrated
    separately.
    """

    def __init__(self, runtime, contract: HardwareContract, *,
                 timestamp_ns: int, step_s: float = 0.01,
                 initial_encoder_counts=None, actuator_perturbation=None):
        if not np.isfinite(step_s) or step_s <= 0.0:
            raise ValueError("step_s must be finite and strictly positive")
        if int(timestamp_ns) <= 0:
            raise ValueError("timestamp_ns must be positive")
        counts = _vector(
            "initial_encoder_counts",
            np.zeros(N_AXES) if initial_encoder_counts is None
            else initial_encoder_counts)
        rounded = np.rint(counts)
        if not np.array_equal(counts, rounded):
            raise ValueError("initial_encoder_counts must be integer-valued")
        if not contract.model_encoder_counts_are_valid(rounded):
            raise ValueError("initial encoder counts are outside model bounds")
        position = contract.encoder_counts_to_logical_position(rounded)
        if not contract.position_is_valid(position):
            raise ValueError("initial encoder counts imply an invalid position")

        self.runtime = runtime
        self.contract = contract
        self.actuator_perturbation = actuator_perturbation
        self.step_s = float(step_s)
        self.step_ns = int(round(self.step_s * 1e9))
        if self.step_ns <= 0:
            raise ValueError("step_s is below nanosecond resolution")
        self.timestamp_ns = int(timestamp_ns)
        self.encoder_counts = rounded.astype(np.float64)
        self.motor_angle_rad = (
            self.encoder_counts * ENCODER_RADIANS_PER_COUNT)
        self.transmitted_motor_angle_rad = self.motor_angle_rad.copy()
        self.joint_position = position
        self.requested_velocity = np.zeros(N_AXES, dtype=np.float64)
        self.last_projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)
        self.runtime.initialize(self.timestamp_ns, self.encoder_counts)
        self._markers = np.asarray(
            self.runtime.current_markers().detach().cpu(),
            dtype=np.float64)

    def set_command(self, logical_velocity) -> None:
        """Set the requested logical velocity held until replaced or stopped."""
        self.requested_velocity = _vector(
            "logical_velocity", logical_velocity).copy()

    def stop(self) -> None:
        """Immediately clear the held velocity command."""
        self.requested_velocity.fill(0.0)
        if self.actuator_perturbation is not None:
            self.actuator_perturbation.halt()
        self.last_projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)

    def step(self, timestamp_ns: int | None = None, *,
             advance_model: bool = True) -> PlantSnapshot:
        """Advance one fixed step and return paired truth/feedback outputs."""
        target_ns = (self.timestamp_ns + self.step_ns
                     if timestamp_ns is None else int(timestamp_ns))
        if target_ns <= self.timestamp_ns:
            raise ValueError("plant timestamps must be strictly increasing")
        dt = (target_ns - self.timestamp_ns) * 1e-9
        projection = self.contract.project_velocity(
            self.requested_velocity, self.joint_position)
        shaft_rate = projection.motor_radians_per_second
        transmitted_rate = shaft_rate
        if self.actuator_perturbation is not None:
            shaft_rate, transmitted_rate = (
                self.actuator_perturbation.apply_components(shaft_rate, dt))
        next_angle = self.contract.integrate_motor_angles(
            self.motor_angle_rad, shaft_rate, dt)
        next_transmitted_angle = self.contract.integrate_motor_angles(
            self.transmitted_motor_angle_rad, transmitted_rate, dt)
        next_counts = np.rint(
            next_angle / ENCODER_RADIANS_PER_COUNT).astype(np.float64)
        next_transmitted_counts = np.rint(
            next_transmitted_angle / ENCODER_RADIANS_PER_COUNT).astype(
                np.float64)
        if not self.contract.model_encoder_counts_are_valid(next_counts):
            self.stop()
            raise RuntimeError("simulated encoder feedback left model bounds")
        next_position = self.contract.encoder_counts_to_logical_position(
            next_counts)
        if not self.contract.position_is_valid(next_position):
            self.stop()
            raise RuntimeError("simulated position left hard limits")

        self.timestamp_ns = target_ns
        self.motor_angle_rad = next_angle
        self.transmitted_motor_angle_rad = next_transmitted_angle
        self.encoder_counts = next_counts
        self.joint_position = next_position
        self.last_projection = projection
        if advance_model:
            self.runtime.advance_encoder(target_ns, next_transmitted_counts)
            self._markers = np.asarray(
                self.runtime.current_markers().detach().cpu(),
                dtype=np.float64)
        markers = self._markers
        if markers.shape != (4, 3) or not np.all(np.isfinite(markers)):
            self.stop()
            raise RuntimeError("learned plant returned invalid markers")
        motor_axis_rate = (
            transmitted_rate/(2.0*np.pi/60.0)
            * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM)
        realized_logical = self.contract.motor_axis_to_logical_velocity(
            motor_axis_rate)
        return PlantSnapshot(
            timestamp_ns=target_ns,
            encoder_counts=next_counts.copy(),
            joint_position=next_position.copy(),
            motor_angle_rad=next_angle.copy(),
            markers_base_m=markers.copy(),
            tip_base_m=markers[-1].copy(),
            requested_logical_velocity=self.requested_velocity.copy(),
            projected_logical_velocity=(
                projection.realized_logical_velocity.copy()),
            realized_logical_velocity=realized_logical.copy(),
            motor_radians_per_second=shaft_rate.copy(),
            transmitted_encoder_counts=next_transmitted_counts.copy(),
            transmitted_motor_angle_rad=next_transmitted_angle.copy(),
            transmitted_motor_radians_per_second=transmitted_rate.copy(),
        )
