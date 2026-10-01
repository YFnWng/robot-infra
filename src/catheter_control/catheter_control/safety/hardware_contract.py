"""
Convert ROS commands into the learned model's motor coordinates.

The constants and transformations mirror the production control manager and
Teensy firmware. MPPI must evaluate this projection before model rollout so a
sampled motion matches the command the hardware can actually execute.

Encoder zero is an established calibration invariant. This module deliberately
exposes no operation that changes or offsets encoder zero.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


N_AXES = 6
ENCODER_DEGREES_PER_COUNT = 0.045
ENCODER_RADIANS_PER_COUNT = np.deg2rad(ENCODER_DEGREES_PER_COUNT)
MAX_MOTOR_RPM = 250

# Firmware jointVRate: physical motor-axis units/s produced by one motor RPM.
_LINEAR_RATE_MM_PER_REV = 67.319841 / 20.0
_ROTATION_RATE_REV_PER_REV = 0.375 / 5.0
_CATHETER_BEND_RATE_MM_PER_REV = -1.190625
_SHEATH_BEND_RATE_REV_PER_REV = 0.375 / 5.0
MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM = np.asarray([
    _LINEAR_RATE_MM_PER_REV / 60.0,
    _ROTATION_RATE_REV_PER_REV / 60.0 * 360.0,
    _CATHETER_BEND_RATE_MM_PER_REV / 60.0,
    _LINEAR_RATE_MM_PER_REV / 60.0,
    _ROTATION_RATE_REV_PER_REV / 60.0 * 360.0,
    _SHEATH_BEND_RATE_REV_PER_REV / 60.0 * 360.0,
], dtype=np.float64)
MOTOR_AXIS_UNITS_PER_ENCODER_COUNT = np.asarray([
    _LINEAR_RATE_MM_PER_REV / 360.0,
    _ROTATION_RATE_REV_PER_REV,
    _CATHETER_BEND_RATE_MM_PER_REV / 360.0,
    _LINEAR_RATE_MM_PER_REV / 360.0,
    _ROTATION_RATE_REV_PER_REV,
    _SHEATH_BEND_RATE_REV_PER_REV,
], dtype=np.float64) * ENCODER_DEGREES_PER_COUNT


def _vector(name: str, value, *, nonnegative: bool = False) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (N_AXES,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values")
    if nonnegative and np.any(result < 0.0):
        raise ValueError(f"{name} must be nonnegative")
    return result


def _vectors(name: str, value) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.ndim < 1 or result.shape[-1] != N_AXES or not np.all(
            np.isfinite(result)):
        raise ValueError(f"{name} must end in six finite values")
    return result


@dataclass(frozen=True)
class ProjectedVelocity:
    """One feasible logical command and the firmware motion it represents."""

    logical_velocity: np.ndarray
    requested_motor_axis_velocity: np.ndarray
    motor_rpm: np.ndarray
    motor_radians_per_second: np.ndarray
    realized_motor_axis_velocity: np.ndarray
    realized_logical_velocity: np.ndarray


@dataclass(frozen=True)
class HardwareContract:
    """Fixed hardware conversions plus one catheter's logical limits."""

    position_lower: np.ndarray
    position_upper: np.ndarray
    velocity_min: np.ndarray
    velocity_max: np.ndarray
    position_guard_horizon_s: float = 0.05
    control_position_margin: np.ndarray | None = None
    model_encoder_count_lower: np.ndarray | None = None
    model_encoder_count_upper: np.ndarray | None = None
    model_encoder_count_margin: np.ndarray | None = None
    feedback_limit_tolerance: np.ndarray | None = None

    def __post_init__(self) -> None:
        lower = _vector("position_lower", self.position_lower)
        upper = _vector("position_upper", self.position_upper)
        minimum = _vector("velocity_min", self.velocity_min, nonnegative=True)
        maximum = _vector("velocity_max", self.velocity_max, nonnegative=True)
        if np.any(lower >= upper):
            raise ValueError(
                "every position lower bound must be below its upper bound")
        # A controller-local contract may deliberately disable one axis by
        # setting both bounds to zero.  The production manager retains the
        # nonzero hardware limits loaded from its own profile.
        if np.any(maximum < 0.0) or np.any(minimum > maximum):
            raise ValueError("velocity bounds are inconsistent")
        if (not np.isfinite(self.position_guard_horizon_s)
                or self.position_guard_horizon_s <= 0.0):
            raise ValueError("position_guard_horizon_s must be positive")
        margin = (np.zeros(N_AXES, dtype=np.float64)
                  if self.control_position_margin is None
                  else _vector("control_position_margin",
                               self.control_position_margin,
                               nonnegative=True))
        feedback_tolerance = (
            np.zeros(N_AXES, dtype=np.float64)
            if self.feedback_limit_tolerance is None
            else _vector("feedback_limit_tolerance",
                         self.feedback_limit_tolerance,
                         nonnegative=True))
        if np.any(2.0*margin >= upper-lower):
            raise ValueError(
                "control position margins must leave a nonempty range")
        count_lower = self.model_encoder_count_lower
        count_upper = self.model_encoder_count_upper
        if (count_lower is None) != (count_upper is None):
            raise ValueError(
                "model encoder lower and upper bounds must be specified together")
        if count_lower is not None:
            count_lower = np.asarray(count_lower, dtype=np.float64)
            count_upper = np.asarray(count_upper, dtype=np.float64)
            if (count_lower.shape != (3,) or count_upper.shape != (3,)
                    or not np.all(np.isfinite(count_lower))
                    or not np.all(np.isfinite(count_upper))
                    or np.any(count_lower >= count_upper)):
                raise ValueError(
                    "model encoder bounds must contain three finite ordered values")
        count_margin = (
            np.zeros(3, dtype=np.float64)
            if self.model_encoder_count_margin is None else
            np.asarray(self.model_encoder_count_margin, dtype=np.float64))
        if (count_margin.shape != (3,)
                or not np.all(np.isfinite(count_margin))
                or np.any(count_margin < 0.0)):
            raise ValueError(
                "model encoder count margin must contain three nonnegative "
                "finite values")
        if (count_lower is None and np.any(count_margin > 0.0)):
            raise ValueError(
                "model encoder count margin requires model encoder bounds")
        if (count_lower is not None
                and np.any(count_lower+count_margin
                           >= count_upper-count_margin)):
            raise ValueError(
                "model encoder count margin must leave a nonempty range")
        object.__setattr__(self, "position_lower", lower.copy())
        object.__setattr__(self, "position_upper", upper.copy())
        object.__setattr__(self, "velocity_min", minimum.copy())
        object.__setattr__(self, "velocity_max", maximum.copy())
        object.__setattr__(self, "control_position_margin", margin.copy())
        object.__setattr__(
            self, "feedback_limit_tolerance", feedback_tolerance.copy())
        object.__setattr__(
            self, "model_encoder_count_lower",
            None if count_lower is None else count_lower.copy())
        object.__setattr__(
            self, "model_encoder_count_upper",
            None if count_upper is None else count_upper.copy())
        object.__setattr__(
            self, "model_encoder_count_margin", count_margin.copy())

    def position_is_valid(self, position) -> bool:
        """Return whether a command/state lies inside every exact hard limit."""
        try:
            value = _vector("position", position)
        except ValueError:
            return False
        return bool(np.all(value >= self.position_lower)
                    and np.all(value <= self.position_upper))

    def feedback_position_is_valid(self, position) -> bool:
        """Validate measured POS with the manager's quantization allowance.

        This tolerance applies only to feedback plausibility. Position
        commands, trajectory projection, and simulated state integration
        continue to use the exact hard limits through :meth:`position_is_valid`.
        """
        try:
            value = _vector("position feedback", position)
        except ValueError:
            return False
        tolerance = self.feedback_limit_tolerance
        return bool(np.all(value >= self.position_lower-tolerance)
                    and np.all(value <= self.position_upper+tolerance))

    def resolve_feedback_position(self, position) -> np.ndarray:
        """Resolve tolerance-qualified feedback onto exact command bounds.

        Encoder quantization can place measured feedback just beyond an exact
        hard limit.  Such feedback is valid only within the configured
        qualification tolerance; planners must nevertheless start projection
        from the exact command domain.  This operation changes neither the
        encoder reference nor the estimator state.
        """
        value = _vector("position feedback", position)
        tolerance = self.feedback_limit_tolerance
        outside = np.flatnonzero(
            (value < self.position_lower-tolerance)
            | (value > self.position_upper+tolerance))
        if outside.size:
            axis = int(outside[0])
            raise ValueError(
                "position feedback is outside feedback-qualified limits: "
                f"axis={axis} value={value[axis]:.9g} exact="
                f"[{self.position_lower[axis]:.9g}, "
                f"{self.position_upper[axis]:.9g}] "
                f"tolerance={tolerance[axis]:.9g}")
        return np.clip(value, self.position_lower, self.position_upper)

    @property
    def control_position_lower(self) -> np.ndarray:
        """Conservative bounds used by autonomous velocity projection."""
        return self.position_lower+self.control_position_margin

    @property
    def control_position_upper(self) -> np.ndarray:
        """Conservative bounds used by autonomous velocity projection."""
        return self.position_upper-self.control_position_margin

    def model_encoder_counts_are_valid(self, encoder_counts) -> bool:
        """Validate raw channels consumed by the learned v171 model."""
        if (self.model_encoder_count_lower is None
                or self.model_encoder_count_upper is None):
            return False
        try:
            counts = _vector("encoder_counts", encoder_counts)[:3]
        except ValueError:
            return False
        return bool(np.all(counts >= self.model_encoder_count_lower)
                    and np.all(counts <= self.model_encoder_count_upper))

    @staticmethod
    def encoder_counts_to_motor_radians(encoder_counts) -> np.ndarray:
        """Convert six uncoupled raw encoder counts to shaft radians."""
        counts = _vector("encoder_counts", encoder_counts)
        return counts * ENCODER_RADIANS_PER_COUNT

    @staticmethod
    def encoder_counts_to_motor_axis_position(encoder_counts) -> np.ndarray:
        """Mirror firmware ``currentPos`` before logical-axis coupling."""
        counts = _vector("encoder_counts", encoder_counts)
        return counts * MOTOR_AXIS_UNITS_PER_ENCODER_COUNT

    @classmethod
    def encoder_counts_to_logical_position(cls, encoder_counts) -> np.ndarray:
        """Mirror the six-axis POS frame reconstructed by the firmware."""
        motor = cls.encoder_counts_to_motor_axis_position(encoder_counts)
        logical = motor.copy()
        logical[0] += logical[2]
        logical[5] -= logical[4]
        return logical

    @staticmethod
    def logical_position_to_motor_axis_position(logical_position) -> np.ndarray:
        """Invert the firmware position coupling before count conversion."""
        motor = _vectors("logical_position", logical_position).copy()
        motor[..., 0] -= motor[..., 2]
        motor[..., 5] += motor[..., 4]
        return motor

    def _guard_model_encoder_velocity(
            self, motor_velocity, joint_position) -> np.ndarray:
        """Keep the first three motor axes inside the learned-model envelope.

        The model limits are raw encoder-count limits and therefore live in
        coupled motor coordinates, not independent logical joint coordinates.
        Guard one position horizon ahead before firmware RPM quantization.
        """
        motor, position = np.broadcast_arrays(
            _vectors("motor_axis_velocity", motor_velocity),
            _vectors("joint_position", joint_position))
        guarded = motor.copy()
        if (self.model_encoder_count_lower is None
                or self.model_encoder_count_upper is None):
            return guarded
        motor_position = self.logical_position_to_motor_axis_position(position)
        scale = MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[:3]
        counts = motor_position[..., :3]/scale
        count_rate = guarded[..., :3]/scale
        operational_lower = (
            self.model_encoder_count_lower+self.model_encoder_count_margin)
        operational_upper = (
            self.model_encoder_count_upper-self.model_encoder_count_margin)
        lower_rate = (
            operational_lower-counts
        )/self.position_guard_horizon_s
        upper_rate = (
            operational_upper-counts
        )/self.position_guard_horizon_s
        below = counts < operational_lower
        above = counts > operational_upper
        inside = ~(below | above)
        clipped = np.clip(count_rate, lower_rate, upper_rate)
        count_rate = np.where(inside, clipped, count_rate)
        count_rate = np.where(below, np.maximum(0.0, count_rate), count_rate)
        count_rate = np.where(above, np.minimum(0.0, count_rate), count_rate)
        guarded[..., :3] = count_rate*scale
        return guarded

    def _quantize_guarded_motor_velocity(
            self, motor_velocity, joint_position):
        """Quantize while preserving the model-count one-horizon bound."""
        guarded = self._guard_model_encoder_velocity(
            motor_velocity, joint_position)
        rpm_request = guarded/MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
        rpm = np.minimum(
            np.floor(np.abs(rpm_request)+0.5), MAX_MOTOR_RPM)
        signed_rpm = np.sign(rpm_request)*rpm
        if self.model_encoder_count_lower is not None:
            position = np.broadcast_to(
                _vectors("joint_position", joint_position), guarded.shape)
            motor_position = self.logical_position_to_motor_axis_position(
                position)
            scale = MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[:3]
            counts = motor_position[..., :3]/scale
            operational_lower = (
                self.model_encoder_count_lower
                + self.model_encoder_count_margin)
            operational_upper = (
                self.model_encoder_count_upper
                - self.model_encoder_count_margin)
            count_rate_lower = (
                operational_lower-counts
            )/self.position_guard_horizon_s
            count_rate_upper = (
                operational_upper-counts
            )/self.position_guard_horizon_s
            motor_rate_a = count_rate_lower*scale
            motor_rate_b = count_rate_upper*scale
            rpm_a = (
                motor_rate_a
                / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[:3])
            rpm_b = (
                motor_rate_b
                / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM[:3])
            rpm_lower = np.ceil(np.minimum(rpm_a, rpm_b))
            rpm_upper = np.floor(np.maximum(rpm_a, rpm_b))
            below = counts < operational_lower
            above = counts > operational_upper
            inside = ~(below | above)
            requested_rpm = signed_rpm[..., :3]
            clipped_rpm = np.clip(requested_rpm, rpm_lower, rpm_upper)
            requested_rpm = np.where(inside, clipped_rpm, requested_rpm)
            requested_rpm = np.where(
                below, np.maximum(0.0, requested_rpm), requested_rpm)
            requested_rpm = np.where(
                above, np.minimum(0.0, requested_rpm), requested_rpm)
            signed_rpm[..., :3] = requested_rpm
        radians_per_second = signed_rpm*(2.0*np.pi/60.0)
        realized = signed_rpm*MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
        return guarded, signed_rpm, radians_per_second, realized

    def project_logical_velocity(
            self, requested, joint_position) -> np.ndarray:
        """Mirror `ControlManager._clamp_command` for velocity commands."""
        velocity = _vector("requested velocity", requested).copy()
        position = _vector("joint_position", joint_position)
        lower = self.control_position_lower
        upper = self.control_position_upper
        for axis in range(N_AXES):
            velocity[axis] = np.clip(
                velocity[axis], -self.velocity_max[axis],
                self.velocity_max[axis])
            if 0.0 < abs(velocity[axis]) < self.velocity_min[axis]:
                velocity[axis] = np.copysign(
                    self.velocity_min[axis], velocity[axis])
            if position[axis] < lower[axis]:
                velocity[axis] = max(0.0, velocity[axis])
            elif position[axis] > upper[axis]:
                velocity[axis] = min(0.0, velocity[axis])
            else:
                minimum = ((lower[axis] - position[axis])
                           / self.position_guard_horizon_s)
                maximum = ((upper[axis] - position[axis])
                           / self.position_guard_horizon_s)
                velocity[axis] = np.clip(velocity[axis], minimum, maximum)
            if 0.0 < abs(velocity[axis]) < self.velocity_min[axis]:
                velocity[axis] = 0.0
        return velocity

    @staticmethod
    def logical_to_motor_axis_velocity(logical_velocity) -> np.ndarray:
        """Apply the coupling transforms used by the Teensy VEL parser."""
        motor = _vector("logical_velocity", logical_velocity).copy()
        motor[0] -= motor[2]
        motor[5] += motor[4]
        return motor

    @staticmethod
    def motor_axis_to_logical_velocity(motor_axis_velocity) -> np.ndarray:
        """Invert the firmware coupling after integer-RPM realization."""
        logical = _vector("motor_axis_velocity", motor_axis_velocity).copy()
        logical[0] += logical[2]
        logical[5] -= logical[4]
        return logical

    @staticmethod
    def motor_radians_per_second_to_motor_axis_velocity(rates) -> np.ndarray:
        """Invert the fixed firmware RPM-to-motor-axis unit conversion."""
        value = _vector("motor_radians_per_second", rates)
        rpm = value/(2.0*np.pi/60.0)
        return rpm*MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM

    @staticmethod
    def quantize_motor_velocity(motor_axis_velocity) -> tuple[
            np.ndarray, np.ndarray, np.ndarray]:
        """Mirror firmware conversion to integer RPM and shaft radians/s."""
        requested = _vector("motor_axis_velocity", motor_axis_velocity)
        signed_rpm_request = requested / MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
        # Firmware applies std::roundf to a nonnegative magnitude.
        rpm = np.floor(np.abs(signed_rpm_request) + 0.5)
        rpm = np.minimum(rpm, MAX_MOTOR_RPM).astype(np.int64)
        signed_rpm = np.sign(signed_rpm_request) * rpm
        radians_per_second = signed_rpm * (2.0 * np.pi / 60.0)
        realized = signed_rpm * MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
        return rpm, radians_per_second, realized

    def project_velocity(
            self, requested, joint_position) -> ProjectedVelocity:
        """Return manager-feasible command and predicted firmware motion."""
        logical = self.project_logical_velocity(requested, joint_position)
        motor_requested = self.logical_to_motor_axis_velocity(logical)
        motor_requested, signed_rpm, radians_per_second, realized = (
            self._quantize_guarded_motor_velocity(
                motor_requested, joint_position))
        rpm = np.abs(signed_rpm).astype(np.int64)
        logical = self.motor_axis_to_logical_velocity(motor_requested)
        realized_logical = self.motor_axis_to_logical_velocity(realized)
        return ProjectedVelocity(
            logical_velocity=logical,
            requested_motor_axis_velocity=motor_requested,
            motor_rpm=rpm,
            motor_radians_per_second=radians_per_second,
            realized_motor_axis_velocity=realized,
            realized_logical_velocity=realized_logical,
        )

    def project_publishable_velocity(self, requested,
                                     joint_position) -> np.ndarray:
        """Return a guarded logical command the manager will not enlarge.

        Model-count projection happens in coupled motor coordinates and can
        reduce a logical component below the manager's configured velocity
        floor. Publishing that value would let the manager round it back up
        and defeat the model-envelope guard. Convert such components to hold
        and re-project until the command is both guarded and publishable.
        """
        logical = _vector("requested velocity", requested).copy()
        position = _vector("joint_position", joint_position)
        floor_tolerance = np.maximum(
            1.0e-12, 1.0e-9*np.maximum(1.0, self.velocity_min))
        for _ in range(N_AXES+1):
            projected = self.project_velocity(logical, position)
            logical = projected.logical_velocity.copy()
            magnitude = np.abs(logical)
            # Coupled motor/logical conversions can return the exact manager
            # floor one floating-point ulp low (for example
            # 1.9999999999999998 for a 2.0 floor). That is not a genuinely
            # subminimum command: the manager would preserve it at the floor.
            # Snap only this numerical boundary case; materially subfloor
            # model-envelope projections still become a hold.
            near_floor = (
                (magnitude > 0.0)
                & (magnitude < self.velocity_min)
                & (magnitude >= self.velocity_min-floor_tolerance))
            logical[near_floor] = np.copysign(
                self.velocity_min[near_floor], logical[near_floor])
            magnitude = np.abs(logical)
            below_floor = (
                (magnitude > 0.0)
                & (magnitude < self.velocity_min-floor_tolerance))
            if not np.any(below_floor):
                return logical
            logical[below_floor] = 0.0
        return np.zeros(N_AXES, dtype=np.float64)

    def project_velocity_batch(self, requested,
                               joint_position) -> ProjectedVelocity:
        """Vectorized form of :meth:`project_velocity` for MPPI samples.

        Inputs must end in six axes; all leading dimensions are broadcast.
        This follows the same manager clamp, coupling, and integer-RPM
        realization as the scalar path.
        """
        velocity, position = np.broadcast_arrays(
            _vectors("requested velocity", requested),
            _vectors("joint_position", joint_position))
        logical = np.clip(
            velocity, -self.velocity_max, self.velocity_max).copy()
        below_floor = ((np.abs(logical) > 0.0)
                       & (np.abs(logical) < self.velocity_min))
        logical = np.where(
            below_floor, np.copysign(self.velocity_min, logical), logical)

        lower = self.control_position_lower
        upper = self.control_position_upper
        below = position < lower
        above = position > upper
        logical = np.where(below, np.maximum(0.0, logical), logical)
        logical = np.where(above, np.minimum(0.0, logical), logical)
        inside = ~(below | above)
        lower_rate = ((lower - position)
                      / self.position_guard_horizon_s)
        upper_rate = ((upper - position)
                      / self.position_guard_horizon_s)
        guarded = np.clip(logical, lower_rate, upper_rate)
        logical = np.where(inside, guarded, logical)
        logical = np.where(
            ((np.abs(logical) > 0.0)
             & (np.abs(logical) < self.velocity_min)), 0.0, logical)

        motor_requested = logical.copy()
        motor_requested[..., 0] -= motor_requested[..., 2]
        motor_requested[..., 5] += motor_requested[..., 4]
        motor_requested, signed_rpm, radians_per_second, realized = (
            self._quantize_guarded_motor_velocity(
                motor_requested, position))
        logical = motor_requested.copy()
        logical[..., 0] += logical[..., 2]
        logical[..., 5] -= logical[..., 4]
        rpm = np.abs(signed_rpm).astype(np.int64)
        realized_logical = realized.copy()
        realized_logical[..., 0] += realized_logical[..., 2]
        realized_logical[..., 5] -= realized_logical[..., 4]
        return ProjectedVelocity(
            logical_velocity=logical,
            requested_motor_axis_velocity=motor_requested,
            motor_rpm=rpm,
            motor_radians_per_second=radians_per_second,
            realized_motor_axis_velocity=realized,
            realized_logical_velocity=realized_logical,
        )

    @staticmethod
    def integrate_motor_angles(motor_angles, motor_radians_per_second,
                               dt: float) -> np.ndarray:
        """Advance candidate shaft angles over one strictly positive step."""
        angle = _vector("motor_angles", motor_angles)
        rate = _vector("motor_radians_per_second", motor_radians_per_second)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and strictly positive")
        return angle + rate * float(dt)

    @staticmethod
    def integrate_joint_positions(joint_positions, logical_velocity,
                                  dt: float) -> np.ndarray:
        """Advance logical position for the next horizon-step limit guard."""
        position = _vector("joint_positions", joint_positions)
        rate = _vector("logical_velocity", logical_velocity)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and strictly positive")
        return position + rate * float(dt)


def load_hardware_contract(limits_file: str | Path, catheter: str,
                           position_guard_horizon_s: float = 0.05
                           ) -> HardwareContract:
    """Load one catheter limit profile used by the production manager."""
    path = Path(limits_file).expanduser().resolve()
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    profiles = document.get("catheters", {})
    if catheter not in profiles:
        raise ValueError(
            f"catheter {catheter!r} is not defined in {path}; "
            f"available profiles: {sorted(profiles)}")
    profile = profiles[catheter]
    return HardwareContract(
        position_lower=profile["pos_lower"],
        position_upper=profile["pos_upper"],
        velocity_min=profile.get("vel_min", [0.0] * N_AXES),
        velocity_max=profile["vel_max"],
        position_guard_horizon_s=position_guard_horizon_s,
        control_position_margin=profile.get(
            "control_position_margin", [0.0] * N_AXES),
        feedback_limit_tolerance=profile.get(
            "feedback_limit_tolerance", [0.0] * N_AXES),
        model_encoder_count_lower=profile.get("model_encoder_count_lower"),
        model_encoder_count_upper=profile.get("model_encoder_count_upper"),
        model_encoder_count_margin=profile.get(
            "model_encoder_count_margin", [0.0, 0.0, 0.0]),
    )
