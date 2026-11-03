"""Guarded, episode-labelled excitation for causal proximal identification.

Coordinates are relative logical catheter joints ``[lin_mm, rot_deg,
bend_mm]``.  The important distinction from the older broad identification
trajectory is that every fitting block has a known raw-motor basis.  In
particular, ``lin == bend`` cancels raw shaft 0 in the firmware mapping and
therefore isolates raw bend shaft 2.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .identification import IdentificationEpisode, _Segment, _D1_MAX


def resolve_tolerance_qualified_start(
        measured_position: Iterable[float],
        lower_limits: Iterable[float],
        upper_limits: Iterable[float],
        feedback_tolerance: Iterable[float]) -> np.ndarray:
    """Map valid measured feedback onto the exact command domain.

    Device feedback may sit a small, calibrated distance beyond a hard command
    boundary because of encoder quantization.  Accept only that configured
    tolerance, then clip the planning origin to the exact hard limits.  This
    keeps generated references command-valid without hiding a real limit
    violation.
    """
    measured = CausalExperimentGenerator._vector(
        measured_position, "measured_position")
    lower = CausalExperimentGenerator._vector(lower_limits, "lower_limits")
    upper = CausalExperimentGenerator._vector(upper_limits, "upper_limits")
    tolerance = CausalExperimentGenerator._vector(
        feedback_tolerance, "feedback_tolerance")
    if np.any(upper <= lower):
        raise ValueError("upper limits must exceed lower limits")
    if np.any(tolerance < 0.0):
        raise ValueError("feedback_tolerance must be non-negative")
    outside = np.flatnonzero(
        (measured < lower - tolerance) | (measured > upper + tolerance))
    if outside.size:
        axis = int(outside[0])
        raise ValueError(
            "run-start position is outside feedback-qualified limits: "
            f"axis={axis} value={measured[axis]:.9g} exact="
            f"[{lower[axis]:.9g}, {upper[axis]:.9g}] "
            f"tolerance={tolerance[axis]:.9g}")
    return np.clip(measured, lower, upper)


@dataclass(frozen=True)
class CausalExperimentConfig:
    # ``full`` preserves the original broad identification experiment.
    # The phase-specific schedules are intentionally composable so hardware
    # isolation can stop after every causal boundary instead of silently
    # continuing into the next excitation family.
    schedule: str = "full"
    # Threshold-clearing half-excursions selected from the 2026-08-29
    # identification fits.  The minimums prevent workspace resolution from
    # silently reducing a run back inside the fitted take-up ranges.
    amplitudes: tuple[float, float, float] = (6.0, 75.0, 5.5)
    minimum_amplitudes: tuple[float, float, float] = (5.0, 65.0, 4.75)
    margins: tuple[float, float, float] = (15.0, 20.0, 1.0)
    slow_speeds: tuple[float, float, float] = (2.0, 7.0, 2.0)
    fast_speeds: tuple[float, float, float] = (5.0, 20.0, 4.0)
    repeats: int = 3
    static_s: float = 15.0
    endpoint_dwell_s: float = 2.0
    between_episode_s: float = 1.0
    rotation_relax_s: float = 4.0
    timing_leads_ms: tuple[float, ...] = (20.0, 40.0, 80.0)
    timing_direction: int = 1
    bend_bias_position: float = 7.5
    insertion_center_position: float = 20.0
    insertion_plateaus: tuple[float, ...] = (
        0.0, 40.0 / 3.0, 80.0 / 3.0, 40.0)
    insertion_plateau_visits: int = 2
    insertion_plateau_dwell_s: float = 3.0
    tendon_sweep_limits: tuple[float, float] = (0.0, 15.0)
    allow_insertion_centering: bool = False
    max_duration_s: float = 950.0
    # tendon_start: short raw-shaft pulses that characterize motor start-up
    # (speed, rest, history, knob position, speed change, coupled leads).
    # Positions and limits are absolute bend in mm; speeds are logical mm/s.
    tendon_start_positions: tuple[float, ...] = (2.0, 6.0, 9.5)
    tendon_start_bend_limits: tuple[float, float] = (0.5, 10.7)
    tendon_start_pulse_s: float = 0.3
    tendon_start_rest_s: float = 0.3
    tendon_start_speeds: tuple[float, ...] = (2.0, 4.5)
    tendon_start_middle_speed: float = 4.0
    tendon_start_dwells_s: tuple[float, float, float] = (0.1, 1.0, 3.0)
    tendon_start_insertion_speeds: tuple[float, ...] = (2.0, 10.0)
    tendon_start_leads_s: tuple[float, ...] = (0.0, 0.04, 0.08)
    tendon_start_combined_repeats: int = 2
    tendon_start_static_s: float = 5.0


@dataclass(frozen=True)
class CausalExperimentEpisode(IdentificationEpisode):
    excitation_basis: str = "static"
    speed_tier: str = "none"
    repetition: int = 0
    # Phase-3 timing episodes are expressed in the two physical proximal
    # shafts: raw shaft 0 (insertion) and raw shaft 2 (tendon).  Logical
    # coordinates satisfy q_lin = q_raw0 + q_raw2 and q_bend = q_raw2.
    timing_raw_start: tuple[float, float] | None = None
    timing_raw_end: tuple[float, float] | None = None
    timing_raw_speeds: tuple[float, float] | None = None
    timing_raw_delays_s: tuple[float, float] | None = None
    insertion_plateau_mm: float | None = None
    branch_order: str | None = None
    # Optional piecewise-constant raw-shaft profile ``(duration_s, v_raw0,
    # v_raw2)`` from local time zero. It replaces the single delayed pulse per
    # shaft, so one episode can hold a rest, a start, a speed change while
    # running or a lead between shafts. Only the tendon_start schedule sets it.
    timing_raw_profile: tuple[tuple[float, float, float], ...] | None = None

    @property
    def is_timing_episode(self) -> bool:
        return self.timing_raw_start is not None

    def _profile_axis_state(self, axis: int, local_t: float):
        position = float(self.timing_raw_start[axis])
        elapsed = 0.0
        for duration, *speeds in self.timing_raw_profile:
            speed = float(speeds[axis])
            if local_t < elapsed + duration:
                return position + speed*max(0.0, local_t - elapsed), speed, 0.0
            position += speed*duration
            elapsed += duration
        return position, 0.0, 0.0

    def _timing_axis_state(self, axis: int, local_t: float):
        if self.timing_raw_profile is not None:
            return self._profile_axis_state(axis, float(local_t))
        start = float(self.timing_raw_start[axis])
        end = float(self.timing_raw_end[axis])
        speed = float(self.timing_raw_speeds[axis])
        delay = float(self.timing_raw_delays_s[axis])
        delta = end - start
        if abs(delta) <= 1e-12:
            return start, 0.0, 0.0
        # Phase 3 is an onset-timing experiment, not a smooth trajectory.
        # Drive each physical shaft with one constant, already-qualified pulse
        # so the reliable-speed floor cannot change its onset, topology, or
        # integrated travel after conversion to coupled logical coordinates.
        duration = abs(delta) / speed
        shifted = float(local_t) - delay
        if shifted < 0.0:
            return start, 0.0, 0.0
        if shifted >= duration:
            return end, 0.0, 0.0
        u = shifted / duration
        return (
            start + delta * u,
            math.copysign(speed, delta),
            0.0,
        )

    def state(self, local_t: float):
        if not self.is_timing_episode:
            return super().state(local_t)
        raw0 = self._timing_axis_state(0, local_t)
        raw2 = self._timing_axis_state(1, local_t)
        position = np.array([raw0[0] + raw2[0], 0.0, raw2[0]])
        velocity = np.array([raw0[1] + raw2[1], 0.0, raw2[1]])
        acceleration = np.array([raw0[2] + raw2[2], 0.0, raw2[2]])
        return position, velocity, acceleration

    def _timing_axis_active(self, axis: int, local_t: float) -> bool:
        if self.timing_raw_profile is not None:
            return self._profile_axis_state(axis, float(local_t))[1] != 0.0
        delta = self.timing_raw_end[axis] - self.timing_raw_start[axis]
        if abs(delta) <= 1e-12:
            return False
        duration = abs(delta) / self.timing_raw_speeds[axis]
        shifted = float(local_t) - self.timing_raw_delays_s[axis]
        return 0.0 <= shifted < duration

    def timing_raw_velocity(self, local_t: float) -> np.ndarray:
        """Return the two commanded physical-shaft velocities."""
        if not self.is_timing_episode:
            raise ValueError("raw timing velocity requires a timing episode")
        return np.asarray([
            self._timing_axis_state(axis, local_t)[1]
            for axis in range(2)
        ], dtype=float)

    def command_speed_limits(self, local_t: float):
        if not self.is_timing_episode:
            return super().command_speed_limits(local_t)
        if self.timing_raw_profile is not None:
            raw0, raw2 = np.abs(self.timing_raw_velocity(local_t))
            return np.array([raw0 + raw2, 0.0, raw2])
        raw0 = (self.timing_raw_speeds[0]
                if self._timing_axis_active(0, local_t) else 0.0)
        raw2 = (self.timing_raw_speeds[1]
                if self._timing_axis_active(1, local_t) else 0.0)
        return np.array([raw0 + raw2, 0.0, raw2])

    @property
    def maximum_command_speed_limits(self):
        if not self.is_timing_episode:
            return super().maximum_command_speed_limits
        if self.timing_raw_profile is not None:
            # The ceiling sums magnitudes even when the two shafts move in
            # opposite directions; logical insertion is raw0 + raw2.
            raw0, raw2 = np.max(np.abs(np.asarray(
                [segment[1:] for segment in self.timing_raw_profile])), axis=0)
            return np.array([raw0 + raw2, 0.0, raw2])
        raw0, raw2 = self.timing_raw_speeds
        return np.array([raw0 + raw2, 0.0, raw2])

    @property
    def planned_raw_onset_delays_s(self):
        if not self.is_timing_episode:
            return None
        return tuple(float(value) for value in self.timing_raw_delays_s)


class CausalExperimentGenerator:
    """Build a deterministic five-block causal identification experiment."""

    version = "causal_proximal_identification_v8"
    SCHEDULES = frozenset({
        "full", "phase_0_2", "stationary", "insertion",
        "tendon_motor", "compensated_bend", "timing",
        "chassis_knob_backdrive", "insertion_rotation",
        "compensated_bend_insertion_sweep", "tendon_start", "tendon_reversal",
    })
    SCHEDULE_ALIASES = {
        "phase_1": "stationary",
        "phase_2a": "insertion",
        "phase_2b": "tendon_motor",
        "phase_2c": "compensated_bend",
        "phase_3": "timing",
        "phase_2d": "insertion_rotation",
        "phase_2e": "compensated_bend_insertion_sweep",
        "backdrive": "chassis_knob_backdrive",
    }

    def __init__(self, start_position: Iterable[float],
                 lower_limits: Iterable[float],
                 upper_limits: Iterable[float],
                 minimum_speeds: Iterable[float],
                 maximum_speeds: Iterable[float], dt: float,
                 config: CausalExperimentConfig | None = None) -> None:
        self.config = config or CausalExperimentConfig()
        self.start_position = self._vector(start_position, "start_position")
        self.lower_limits = self._vector(lower_limits, "lower_limits")
        self.upper_limits = self._vector(upper_limits, "upper_limits")
        self.minimum_speeds = self._vector(
            minimum_speeds, "minimum_speeds")
        self.maximum_speeds = self._vector(
            maximum_speeds, "maximum_speeds")
        self.dt = float(dt)
        self._validate_inputs()
        requested_schedule = str(self.config.schedule).strip().lower()
        self.schedule = self.SCHEDULE_ALIASES.get(
            requested_schedule, requested_schedule)
        if self.schedule not in self.SCHEDULES:
            raise ValueError(
                "causal schedule must be one of "
                f"{sorted(self.SCHEDULES)}; got {self.config.schedule!r}")

        margins = self._vector(self.config.margins, "margins")
        if self.schedule == "compensated_bend_insertion_sweep":
            # This explicit boundary-characterization schedule intentionally
            # spans the reviewed logical insertion range and the complete
            # one-sided bend range.  It remains subject to the exact hard
            # limits and feedback-tolerance gates; no endpoint is clipped.
            margins = margins.copy()
            margins[[0, 2]] = 0.0
        self.usable_lower = self.lower_limits + margins
        self.usable_upper = self.upper_limits - margins
        if np.any(self.usable_upper <= self.usable_lower):
            raise ValueError("causal margins leave no usable position range")
        if np.any(self.start_position < self.lower_limits) or np.any(
                self.start_position > self.upper_limits):
            raise ValueError("run-start position is outside hard limits")

        requested = self._vector(self.config.amplitudes, "amplitudes")
        center = self.start_position.copy()
        configured_center = float(self.config.insertion_center_position)
        if not math.isfinite(configured_center):
            raise ValueError("insertion_center_position must be finite")
        if (self.config.allow_insertion_centering
                and self.schedule != "stationary"):
            center[0] = configured_center
        self.experiment_center = center
        if (self.schedule != "stationary"
                and not self.usable_lower[0] <= center[0]
                <= self.usable_upper[0]):
            raise ValueError(
                "causal insertion center is outside the margin-qualified "
                f"range: center={center[0]} range="
                f"[{self.usable_lower[0]}, {self.usable_upper[0]}]")
        available = np.array([
            min(center[0] - self.usable_lower[0],
                self.usable_upper[0] - center[0]),
            min(center[1] - self.usable_lower[1],
                self.usable_upper[1] - center[1]),
            math.inf,
        ])
        bias = float(self.config.bend_bias_position)
        uses_bend_bias = self.schedule in {
            "full", "phase_0_2", "tendon_motor", "compensated_bend",
            "timing", "compensated_bend_insertion_sweep"}
        if (uses_bend_bias
                and not self.usable_lower[2] <= bias <= self.usable_upper[2]):
            raise ValueError(
                "bend_bias_position is outside the margin-qualified range")
        bend_available = min(
            bias - self.usable_lower[2], self.usable_upper[2] - bias,
            center[0] - self.usable_lower[0],
            self.usable_upper[0] - center[0])
        available[2] = bend_available
        self.amplitudes = np.maximum(0.0, np.minimum(requested, available))
        if self.schedule == "compensated_bend_insertion_sweep":
            plateaus = np.asarray(
                self.config.insertion_plateaus, dtype=float)
            bend_low, bend_high = self.config.tendon_sweep_limits
            self.amplitudes = np.array([
                float(np.max(np.abs(plateaus - self.start_position[0]))),
                0.0,
                max(float(self.config.bend_bias_position) - bend_low,
                    bend_high - float(self.config.bend_bias_position)),
            ])
        minimum = self._vector(
            self.config.minimum_amplitudes, "minimum_amplitudes")
        required_axes = {
            "full": (0, 1, 2),
            "phase_0_2": (0, 2),
            "stationary": (),
            "insertion": (0,),
            "tendon_motor": (2,),
            "compensated_bend": (2,),
            "timing": (0, 2),
            "chassis_knob_backdrive": (0, 2),
            "insertion_rotation": (0, 1),
            "compensated_bend_insertion_sweep": (),
            "tendon_start": (),
            "tendon_reversal": (),
        }[self.schedule]
        if any(self.amplitudes[axis] + 1e-12 < minimum[axis]
               for axis in required_axes):
            raise ValueError(
                "insufficient margin-qualified excursion: requested="
                f"{requested.tolist()} resolved={self.amplitudes.tolist()} "
                f"minimum={minimum.tolist()}")
        self.bend_bias_relative = bias - self.start_position[2]
        self.center_relative = center - self.start_position

        self.slow_speeds = self._resolve_speeds(
            self.config.slow_speeds, "slow_speeds")
        self.fast_speeds = self._resolve_speeds(
            self.config.fast_speeds, "fast_speeds")
        if np.any(self.fast_speeds < self.slow_speeds):
            raise ValueError("fast_speeds must be no smaller than slow_speeds")

        self._episodes = self._build_episodes()
        self.duration = sum(ep.duration_s for ep in self._episodes)
        if self.duration > self.config.max_duration_s:
            raise ValueError(
                f"causal experiment duration {self.duration:.3f}s exceeds "
                f"maximum {self.config.max_duration_s:.3f}s")
        self._validate_plan()

    @staticmethod
    def _vector(values, name):
        result = np.asarray(tuple(values), dtype=float)
        if result.shape != (3,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain three finite values")
        return result

    def _validate_inputs(self):
        if not math.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if np.any(self.upper_limits <= self.lower_limits):
            raise ValueError("upper limits must exceed lower limits")
        for name in ("amplitudes", "minimum_amplitudes", "margins"):
            if np.any(self._vector(getattr(self.config, name), name) < 0.0):
                raise ValueError(f"{name} must be non-negative")
        if self.config.repeats < 1:
            raise ValueError("repeats must be positive")
        for name in ("static_s", "endpoint_dwell_s", "between_episode_s",
                     "rotation_relax_s", "max_duration_s"):
            value = float(getattr(self.config, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        leads = np.asarray(self.config.timing_leads_ms, dtype=float)
        if (leads.ndim != 1 or leads.size == 0
                or not np.all(np.isfinite(leads)) or np.any(leads <= 0.0)):
            raise ValueError("timing_leads_ms must contain positive values")
        if np.any(np.diff(leads) <= 0.0):
            raise ValueError("timing_leads_ms must be strictly increasing")
        ticks = leads * 1e-3 / self.dt
        if not np.allclose(ticks, np.round(ticks), atol=1e-9):
            raise ValueError(
                "timing_leads_ms must align with the command period")
        if int(self.config.timing_direction) not in (-1, 1):
            raise ValueError("timing_direction must be -1 or 1")
        plateaus = np.asarray(self.config.insertion_plateaus, dtype=float)
        if (plateaus.ndim != 1 or plateaus.size != 4
                or not np.all(np.isfinite(plateaus))
                or np.any(np.diff(plateaus) <= 0.0)):
            raise ValueError(
                "insertion_plateaus must contain four strictly increasing "
                "finite values")
        if int(self.config.insertion_plateau_visits) != 2:
            raise ValueError("insertion_plateau_visits must equal two")
        plateau_dwell = float(self.config.insertion_plateau_dwell_s)
        if not math.isfinite(plateau_dwell) or plateau_dwell <= 0.0:
            raise ValueError("insertion_plateau_dwell_s must be positive")
        tendon_limits = np.asarray(
            self.config.tendon_sweep_limits, dtype=float)
        if (tendon_limits.shape != (2,)
                or not np.all(np.isfinite(tendon_limits))
                or tendon_limits[1] <= tendon_limits[0]):
            raise ValueError(
                "tendon_sweep_limits must contain increasing finite bounds")

    def _resolve_speeds(self, configured, name):
        speeds = self._vector(configured, name)
        if np.any(speeds <= 0.0):
            raise ValueError(f"{name} must be positive")
        if np.any(speeds < self.minimum_speeds) or np.any(
                speeds > self.maximum_speeds):
            raise ValueError(
                f"{name} must lie inside reliable speed limits; configured="
                f"{speeds.tolist()} reliable=[{self.minimum_speeds.tolist()}, "
                f"{self.maximum_speeds.tolist()}]")
        return speeds

    @staticmethod
    def _move_duration(start, end, speeds):
        moving = np.abs(end - start) > 1e-12
        if not np.any(moving):
            return 0.0
        return float(np.max(
            _D1_MAX * np.abs(end[moving] - start[moving]) / speeds[moving]))

    def _episode(self, name, basis, tier, repetition, waypoints, speeds,
                 endpoint_holds=True, final_hold=None,
                 insertion_plateau_mm=None, branch_order=None):
        points = [np.asarray(point, dtype=float) for point in waypoints]
        segments = []
        for index in range(len(points) - 1):
            moving = np.abs(points[index + 1] - points[index]) > 1e-12
            limits = np.where(moving, speeds, 0.0)
            duration = self._move_duration(
                points[index], points[index + 1], speeds)
            segments.append(_Segment(
                "move", duration, points[index], points[index + 1],
                speed_limits=limits))
            hold = self.config.endpoint_dwell_s if endpoint_holds else 0.0
            if index == len(points) - 2 and final_hold is not None:
                hold = float(final_hold)
            if hold > 0.0:
                segments.append(_Segment(
                    "hold", hold, points[index + 1], points[index + 1],
                    speed_limits=np.zeros(3)))
        return CausalExperimentEpisode(
            name=name, start_s=0.0,
            duration_s=sum(segment.duration for segment in segments),
            segments=tuple(segments), excitation_basis=basis,
            speed_tier=tier, repetition=repetition,
            insertion_plateau_mm=insertion_plateau_mm,
            branch_order=branch_order)

    def _hold(self, name, point, duration, basis="static",
              insertion_plateau_mm=None):
        segment = _Segment(
            "hold", float(duration), point, point,
            speed_limits=np.zeros(3))
        return CausalExperimentEpisode(
            name=name, start_s=0.0, duration_s=float(duration),
            segments=(segment,), excitation_basis=basis,
            speed_tier="none", repetition=0,
            insertion_plateau_mm=insertion_plateau_mm)

    def _timing_episode(self, name, basis, tier, repetition, start, end,
                        raw_speeds, raw_delays_s):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        raw_start = np.array([start[0] - start[2], start[2]])
        raw_end = np.array([end[0] - end[2], end[2]])
        raw_speeds = np.asarray(raw_speeds, dtype=float)
        raw_delays_s = np.asarray(raw_delays_s, dtype=float)
        durations = np.zeros(2)
        moving = np.abs(raw_end - raw_start) > 1e-12
        durations[moving] = (
            np.abs(raw_end[moving] - raw_start[moving])
            / raw_speeds[moving])
        duration = float(np.max(raw_delays_s + durations)
                         + self.config.endpoint_dwell_s)
        return CausalExperimentEpisode(
            name=name, start_s=0.0, duration_s=duration, segments=(),
            excitation_basis=basis, speed_tier=tier,
            repetition=repetition,
            timing_raw_start=tuple(raw_start.tolist()),
            timing_raw_end=tuple(raw_end.tolist()),
            timing_raw_speeds=tuple(raw_speeds.tolist()),
            timing_raw_delays_s=tuple(raw_delays_s.tolist()))

    def _build_timing_episodes(self, center, bias):
        """Build Phase 3 with exact, single-publisher shaft onset offsets."""
        episodes = [self._hold("static_start", center, self.config.static_s)]
        episodes.append(self._episode(
            "bend_bias_enter", "setup_compensated", "slow", 0,
            [center, bias], self.slow_speeds,
            final_hold=self.config.endpoint_dwell_s))

        direction = float(self.config.timing_direction)
        full_raw_delta = np.array([self.amplitudes[0], self.amplitudes[2]])
        full_logical_delta = np.array([
            full_raw_delta[0] + full_raw_delta[1], 0.0,
            full_raw_delta[1]])
        precondition_speeds = self.slow_speeds.copy()
        precondition_speeds[0] = min(
            self.maximum_speeds[0],
            self.slow_speeds[0] + self.slow_speeds[2])
        # End with motion opposite the measured trial direction. Every trial
        # then begins from the same deliberate reversal history at the bias.
        episodes.append(self._episode(
            "timing_direction_precondition", "timing_precondition",
            "slow", 0,
            [bias, bias + direction * full_logical_delta, bias],
            precondition_speeds, endpoint_holds=False,
            final_hold=self.config.between_episode_s))

        leads_s = [float(value) * 1e-3
                   for value in self.config.timing_leads_ms]
        conditions = [
            ("insertion_only", True, False, 0.0, 0.0),
            ("tendon_only", False, True, 0.0, 0.0),
            ("simultaneous", True, True, 0.0, 0.0),
        ]
        conditions.extend(
            (f"insertion_lead_{int(round(lead * 1e3)):03d}ms",
             True, True, 0.0, lead)
            for lead in leads_s)
        conditions.extend(
            (f"tendon_lead_{int(round(lead * 1e3)):03d}ms",
             True, True, lead, 0.0)
            for lead in leads_s)

        for tier_index, (tier, logical_speeds) in enumerate((
                ("slow", self.slow_speeds),
                ("fast", self.fast_speeds))):
            raw_speeds = np.array([
                logical_speeds[0],
                min(logical_speeds[0], logical_speeds[2]),
            ])
            for repetition in range(1, self.config.repeats + 1):
                # Cyclic/reversed ordering prevents one condition from always
                # inheriting the same accumulated mechanical history.
                offset = (repetition - 1) * 3
                order = list(range(len(conditions)))
                order = order[offset:] + order[:offset]
                if tier_index:
                    order.reverse()
                for condition_index in order:
                    condition, use0, use2, delay0, delay2 = (
                        conditions[condition_index])
                    raw_delta = np.array([
                        self.amplitudes[0] if use0 else 0.0,
                        self.amplitudes[2] if use2 else 0.0,
                    ])
                    logical_delta = np.array([
                        raw_delta[0] + raw_delta[1], 0.0, raw_delta[1]])
                    endpoint = bias + direction * logical_delta
                    suffix = "pos" if direction > 0.0 else "neg"
                    stem = f"timing_{condition}_{tier}_rep{repetition}_{suffix}"
                    episodes.append(self._timing_episode(
                        stem, "timing_" + condition, tier, repetition,
                        bias, endpoint, raw_speeds, (delay0, delay2)))
                    recovery_speeds = logical_speeds.copy()
                    recovery_speeds[0] = min(
                        self.maximum_speeds[0],
                        raw_speeds[0] + raw_speeds[1])
                    episodes.append(self._episode(
                        stem + "_recover", "timing_recovery", tier,
                        repetition, [endpoint, bias], recovery_speeds,
                        endpoint_holds=False,
                        final_hold=self.config.between_episode_s))

        episodes.append(self._episode(
            "bend_bias_exit", "setup_compensated", "slow", 0,
            [bias, center], self.slow_speeds,
            final_hold=self.config.between_episode_s))
        episodes.append(self._hold(
            "static_end", center, self.config.static_s))
        return episodes

    def _raw_profile_episode(self, name, basis, tier, repetition, start,
                             profile):
        """One raw-shaft profile episode starting at relative logical ``start``.

        Every segment must give each logical axis either zero speed or a speed
        inside the reliable band, so the manager projection cannot change the
        raw command (the collection node aborts if it would).
        """
        start = np.asarray(start, dtype=float)
        segments = tuple((float(d), float(a), float(b))
                         for d, a, b in profile if d > 0.0)
        if not segments:
            raise ValueError(f"{name}: empty raw profile")
        for duration, raw0, raw2 in segments:
            logical = np.array([raw0 + raw2, 0.0, raw2])
            moving = np.abs(logical) > 1e-12
            if (not math.isfinite(duration)
                    or np.any(np.abs(logical[moving])
                              < self.minimum_speeds[moving] - 1e-9)
                    or np.any(np.abs(logical)
                              > self.maximum_speeds + 1e-9)):
                raise ValueError(
                    f"{name}: segment {(duration, raw0, raw2)} gives logical "
                    f"speed {logical.tolist()} outside the reliable band")
        raw_start = np.array([start[0] - start[2], start[2]])
        raw_end = raw_start + np.sum(
            [[d*a, d*b] for d, a, b in segments], axis=0)
        speeds = np.max(np.abs([[a, b] for _, a, b in segments]), axis=0)
        onsets = []
        for axis in (1, 2):
            elapsed, onset = 0.0, 0.0
            for segment in segments:
                if segment[axis] != 0.0:
                    onset = elapsed
                    break
                elapsed += segment[0]
            onsets.append(onset)
        episode = CausalExperimentEpisode(
            name=name, start_s=0.0,
            duration_s=float(sum(segment[0] for segment in segments)),
            segments=(), excitation_basis=basis, speed_tier=tier,
            repetition=repetition, timing_raw_start=tuple(raw_start.tolist()),
            timing_raw_end=tuple(raw_end.tolist()),
            timing_raw_speeds=tuple(speeds.tolist()),
            timing_raw_delays_s=tuple(onsets), timing_raw_profile=segments)
        end = np.array([raw_end[0] + raw_end[1], start[1], raw_end[1]])
        return episode, end

    def _build_tendon_start_episodes(self):
        """Motor start-up characterization from short raw pulses.

        At every knob position: tendon-only pulses at each speed, both
        directions, same-direction and after-reversal history (++-- or --++,
        chosen so excursions head toward the middle of the range). At the
        middle position also: rest-time levels, an intermediate speed, speed
        changes without stopping, insertion-only references and coupled starts
        with the tendon leading. Raw shaft 0 is held for every tendon-only
        pulse and setup move.
        """
        cfg = self.config
        pulse, rest = float(cfg.tendon_start_pulse_s), float(
            cfg.tendon_start_rest_s)
        speeds = [float(v) for v in cfg.tendon_start_speeds]
        slow = speeds[0]
        positions = [float(v) for v in cfg.tendon_start_positions]
        middle_index = len(positions)//2
        midpoint = 0.5*sum(cfg.tendon_start_bend_limits)
        add, move_to_bend, finish = self._raw_profile_session("tendon_start")

        for index, position in enumerate(positions):
            move_to_bend(f"tendon_start_enter_p{index}", position)
            order = (-1, -1, 1, 1) if position > midpoint else (1, 1, -1, -1)
            for speed in speeds:
                for repetition in range(1, cfg.repeats + 1):
                    for k, sign in enumerate(order):
                        add(f"tendon_start_p{index}_{speed:g}mms_rep"
                            f"{repetition}_{k}", "tendon_start", f"{speed:g}",
                            repetition, [(rest, 0, 0), (pulse, 0, sign*speed)])
            if index != middle_index:
                continue
            short, medium, long = (float(v) for v in cfg.tendon_start_dwells_s)
            for dwell in (short, medium):
                for repetition in range(1, cfg.repeats + 1):
                    for k, sign in enumerate(order):
                        add(f"tendon_start_dwell_{dwell:g}s_rep{repetition}_{k}",
                            "tendon_start_dwell", f"{slow:g}", repetition,
                            [(dwell, 0, 0), (pulse, 0, sign*slow)])
            for repetition in range(1, cfg.repeats + 1):
                for sign in (order[0], order[2]):
                    add(f"tendon_start_dwell_{long:g}s_rep{repetition}_lead_{sign:+d}",
                        "tendon_start_dwell", f"{slow:g}", repetition,
                        [(rest, 0, 0), (pulse, 0, sign*slow)])
                    add(f"tendon_start_dwell_{long:g}s_rep{repetition}_{sign:+d}",
                        "tendon_start_dwell", f"{slow:g}", repetition,
                        [(long, 0, 0), (pulse, 0, sign*slow)])
            fast = float(cfg.tendon_start_middle_speed)
            for repetition in range(1, cfg.repeats + 1):
                for k, sign in enumerate(order):
                    add(f"tendon_start_mid_{fast:g}mms_rep{repetition}_{k}",
                        "tendon_start", f"{fast:g}", repetition,
                        [(rest, 0, 0), (pulse, 0, sign*fast)])
            for repetition in range(1, 2*cfg.repeats + 1):
                sign = order[0] if repetition % 2 else -order[0]
                add(f"tendon_start_speed_change_rep{repetition}",
                    "tendon_start_speed_change", f"{slow:g}-{fast:g}",
                    repetition, [(rest, 0, 0), (pulse, 0, sign*slow),
                                 (pulse, 0, sign*fast), (pulse, 0, sign*slow)])
            for speed in (float(v) for v in cfg.tendon_start_insertion_speeds):
                for repetition in range(1, cfg.repeats + 1):
                    for sign in (1, -1):
                        add(f"tendon_start_insertion_{speed:g}mms_rep"
                            f"{repetition}_{sign:+d}", "tendon_start_insertion",
                            f"{speed:g}", repetition,
                            [(rest, 0, 0), (pulse, sign*speed, 0)])
            pairs = ((slow, slow), (-2*slow, slow), (-slow, -slow),
                     (2*slow, -slow))
            for lead in (float(v) for v in cfg.tendon_start_leads_s):
                for repetition in range(1, cfg.tendon_start_combined_repeats + 1):
                    for raw0, raw2 in pairs:
                        profile = ([(rest, 0, 0), (pulse, raw0, raw2)]
                                   if lead == 0.0 else
                                   [(rest, 0, 0), (lead, 0, raw2),
                                    (pulse - lead, raw0, raw2), (lead, raw0, 0)])
                        add(f"tendon_start_combined_lead_{round(lead*1e3):03d}ms_"
                            f"rep{repetition}_{raw0:+g}_{raw2:+g}",
                            f"tendon_start_combined_lead_{round(lead*1e3):03d}ms",
                            f"{slow:g}", repetition, profile)
        return finish()

    def _raw_profile_session(self, prefix):
        """Shared frame for raw-profile schedules.

        Returns ``add`` (append one raw-profile episode from the current
        state), ``move_to_bend`` (setup move to an absolute bend holding raw
        shaft 0) and ``finish`` (exit to the run start, end hold, raw shaft 0
        return and bend-band checks). Static holds bracket the schedule.
        """
        cfg = self.config
        setup_speed = min(self.slow_speeds[0], self.slow_speeds[2])
        setup_speeds = np.array(
            [setup_speed, self.slow_speeds[1], setup_speed])
        episodes = [self._hold(
            f"{prefix}_static_start", np.zeros(3), cfg.tendon_start_static_s)]
        state = {"p": np.zeros(3)}

        def add(name, basis, tier, repetition, profile):
            episode, end = self._raw_profile_episode(
                name, basis, tier, repetition, state["p"], profile)
            episodes.append(episode)
            state["p"] = end

        def move_to_bend(name, absolute_bend):
            target = state["p"].copy()
            delta = absolute_bend - (self.start_position[2] + target[2])
            target[[0, 2]] += delta  # [d,0,d] holds raw shaft 0
            episodes.append(self._episode(
                name, "shaft_2", "slow", 0, [state["p"], target],
                setup_speeds, endpoint_holds=False, final_hold=0.5))
            state["p"] = target

        def finish():
            final = state["p"]
            if not np.isclose(final[0] - final[2], 0.0, atol=1e-9):
                raise ValueError(f"{prefix} raw shaft 0 does not return to start")
            episodes.append(self._episode(
                f"{prefix}_exit", "shaft_2", "slow", 0, [final, np.zeros(3)],
                setup_speeds, endpoint_holds=False, final_hold=0.5))
            episodes.append(self._hold(
                f"{prefix}_static_end", np.zeros(3), cfg.tendon_start_static_s))
            self._validate_tendon_start(episodes)
            return episodes

        return add, move_to_bend, finish

    def _build_tendon_reversal_episodes(self):
        """On-the-fly reversal characterization at the middle knob position.

        Each reversal changes the raw shaft sign without an intermediate zero
        command. Matched pairs separated by a rest are the in-session
        from-rest reference. Blocks: tendon reversals at each speed (with the
        rest-separated reference), tendon reversals combined with a speed
        change, insertion-shaft reversals, and tendon reversals while the
        insertion shaft keeps running. On legacy firmware every reversal is a
        coordinated restart; on the on-the-fly test image it is direction then
        speed without disabling. Raw shaft 0 returns to the run start.
        """
        cfg = self.config
        pulse, rest = float(cfg.tendon_start_pulse_s), float(
            cfg.tendon_start_rest_s)
        speeds = [float(v) for v in cfg.tendon_start_speeds]
        slow, fast = speeds[0], speeds[-1]
        positions = [float(v) for v in cfg.tendon_start_positions]
        add, move_to_bend, finish = self._raw_profile_session("tendon_reversal")
        move_to_bend("tendon_reversal_enter", positions[len(positions)//2])
        for speed in speeds:
            for repetition in range(1, cfg.repeats + 1):
                for sign in (1, -1):
                    add(f"tendon_reversal_{speed:g}mms_rep{repetition}_{sign:+d}",
                        "tendon_reversal", f"{speed:g}", repetition,
                        [(rest, 0, 0), (pulse, 0, sign*speed),
                         (pulse, 0, -sign*speed)])
                    add(f"tendon_reversal_rest_{speed:g}mms_rep{repetition}_{sign:+d}",
                        "tendon_reversal_rest", f"{speed:g}", repetition,
                        [(rest, 0, 0), (pulse, 0, sign*speed), (rest, 0, 0),
                         (pulse, 0, -sign*speed)])
        for repetition in range(1, cfg.repeats + 1):
            for sign in (1, -1):
                # The two orders cancel, so each pair returns to the position.
                for first, second in ((slow, fast), (fast, slow)):
                    add(f"tendon_reversal_speed_change_{first:g}-{second:g}mms_"
                        f"rep{repetition}_{sign:+d}",
                        "tendon_reversal_speed_change", f"{first:g}-{second:g}",
                        repetition, [(rest, 0, 0), (pulse, 0, sign*first),
                                     (pulse, 0, -sign*second)])
        for speed in (float(v) for v in cfg.tendon_start_insertion_speeds):
            for repetition in range(1, cfg.repeats + 1):
                for sign in (1, -1):
                    add(f"tendon_reversal_insertion_{speed:g}mms_rep{repetition}_"
                        f"{sign:+d}", "tendon_reversal_insertion", f"{speed:g}",
                        repetition, [(rest, 0, 0), (pulse, sign*speed, 0),
                                     (pulse, -sign*speed, 0)])
        for repetition in range(1, cfg.tendon_start_combined_repeats + 1):
            # Logical insertion raw0 + raw2 stays at 2*slow +- slow: inside the band.
            for raw0 in (2*slow, -2*slow):
                for sign in (1, -1):
                    add(f"tendon_reversal_with_insertion_rep{repetition}_"
                        f"{raw0:+g}_{sign:+d}", "tendon_reversal_with_insertion",
                        f"{slow:g}", repetition,
                        [(rest, 0, 0), (pulse, raw0, sign*slow),
                         (pulse, raw0, -sign*slow)])
        return finish()

    def _validate_tendon_start(self, episodes):
        """Every pulse stays inside the declared absolute bend band."""
        low, high = (float(v) for v in self.config.tendon_start_bend_limits)
        for episode in episodes:
            if episode.timing_raw_profile is None:
                continue
            times = np.r_[0.0, np.cumsum(
                [segment[0] for segment in episode.timing_raw_profile])]
            bend = np.array([episode.state(t)[0][2] for t in times])
            bend += self.start_position[2]
            if bend.min() < low - 1e-9 or bend.max() > high + 1e-9:
                raise ValueError(
                    f"{episode.name}: bend {bend.min():.3f}..{bend.max():.3f} "
                    f"outside [{low}, {high}]")

    def _build_compensated_bend_insertion_sweep(self):
        """Sweep compensated bending at four fixed insertion plateaus.

        The two passes reverse plateau order.  Speed parity is also reversed
        by traversing the second pass in the opposite order, so every plateau
        receives exactly one slow and one fast cycle.  Branch order flips
        between passes while physical history remains continuous.
        """
        plateaus = np.asarray(self.config.insertion_plateaus, dtype=float)
        bend_low, bend_high = map(
            float, self.config.tendon_sweep_limits)
        bend_bias = float(self.config.bend_bias_position)
        if np.any(plateaus < self.lower_limits[0] - 1e-9) or np.any(
                plateaus > self.upper_limits[0] + 1e-9):
            raise ValueError(
                "insertion plateau exceeds hard position limits")
        if not (self.lower_limits[2] - 1e-9 <= bend_low
                < bend_bias < bend_high
                <= self.upper_limits[2] + 1e-9):
            raise ValueError(
                "tendon sweep and bias must lie inside bend hard limits")

        def relative(insertion, bend):
            return (np.array([float(insertion), 0.0, float(bend)])
                    - self.start_position)

        home = np.zeros(3)
        home_bias = relative(self.start_position[0], bend_bias)
        episodes = [self._hold(
            "static_start", home, self.config.static_s)]
        episodes.append(self._episode(
            "insertion_sweep_bias_enter", "compensated_bend_setup",
            "slow", 0, [home, home_bias], self.slow_speeds,
            endpoint_holds=False))

        current = home_bias
        passes = (plateaus, plateaus[::-1])
        for pass_index, ordered_plateaus in enumerate(passes, start=1):
            high_first = pass_index == 1
            branch_order = "high_first" if high_first else "low_first"
            for rank, plateau in enumerate(ordered_plateaus):
                plateau = float(plateau)
                at_bias = relative(plateau, bend_bias)
                if not np.allclose(current, at_bias):
                    episodes.append(self._episode(
                        f"insertion_sweep_p{pass_index}_level_"
                        f"{plateau:06.3f}_transition",
                        "insertion_plateau_transition", "slow",
                        pass_index, [current, at_bias], self.slow_speeds,
                        endpoint_holds=False,
                        insertion_plateau_mm=plateau))
                episodes.append(self._hold(
                    f"insertion_sweep_p{pass_index}_level_"
                    f"{plateau:06.3f}_dwell",
                    at_bias, self.config.insertion_plateau_dwell_s,
                    basis="insertion_plateau_dwell",
                    insertion_plateau_mm=plateau))

                tier = "slow" if rank % 2 == 0 else "fast"
                speeds = (self.slow_speeds if tier == "slow"
                          else self.fast_speeds)
                first = bend_high if high_first else bend_low
                second = bend_low if high_first else bend_high
                waypoints = [
                    at_bias,
                    relative(plateau, first),
                    at_bias,
                    relative(plateau, second),
                    at_bias,
                ]
                episodes.append(self._episode(
                    f"compensated_bend_insertion_p{pass_index}_level_"
                    f"{plateau:06.3f}_{tier}_{branch_order}",
                    "compensated_bend_at_insertion", tier, pass_index,
                    waypoints, speeds,
                    insertion_plateau_mm=plateau,
                    branch_order=branch_order))
                current = at_bias

        if not np.allclose(current, home_bias):
            episodes.append(self._episode(
                "insertion_sweep_return_to_center",
                "insertion_plateau_transition", "slow", 0,
                [current, home_bias], self.slow_speeds,
                endpoint_holds=False,
                insertion_plateau_mm=float(self.start_position[0])))
        episodes.append(self._episode(
            "insertion_sweep_bias_exit", "compensated_bend_setup",
            "slow", 0, [home_bias, home], self.slow_speeds,
            endpoint_holds=False))
        episodes.append(self._hold(
            "static_end", home, self.config.static_s))
        return episodes

    def _build_episodes(self):
        z = np.zeros(3)
        center = self.center_relative.copy()
        s = np.array([self.amplitudes[0], 0.0, 0.0])
        r = np.array([0.0, self.amplitudes[1], 0.0])
        b = np.array([self.amplitudes[2], 0.0, self.amplitudes[2]])
        c_delta = np.array([0.0, 0.0, self.amplitudes[2]])
        bias = center + np.array([0.0, 0.0, self.bend_bias_relative])
        episodes = []
        do_insertion = self.schedule in {"full", "phase_0_2", "insertion"}
        do_rotation = self.schedule == "full"
        do_tendon = self.schedule in {
            "full", "phase_0_2", "tendon_motor"}
        do_compensated = self.schedule in {
            "full", "phase_0_2", "compensated_bend"}
        do_bend_bias = do_tendon or do_compensated
        do_mixed = self.schedule == "full"

        if self.schedule == "stationary":
            episodes.append(self._hold(
                "stationary_noise", z, self.config.static_s))
            return self._with_episode_starts(episodes)

        if self.schedule == "timing":
            return self._with_episode_starts(
                self._build_timing_episodes(center, bias))

        if self.schedule == "tendon_start":
            return self._with_episode_starts(
                self._build_tendon_start_episodes())

        if self.schedule == "tendon_reversal":
            return self._with_episode_starts(
                self._build_tendon_reversal_episodes())

        if self.schedule == "compensated_bend_insertion_sweep":
            return self._with_episode_starts(
                self._build_compensated_bend_insertion_sweep())

        if self.schedule == "chassis_knob_backdrive":
            # Logical coordinates obey raw0 = lin-bend and raw2 = bend.
            # Consequently [d,0,d] moves only the tightly driven knob while
            # holding the commanded chassis shaft fixed. This four-move probe
            # exposes any physical rack-and-pinion backdrive for visual and
            # encoder inspection without commanding coupled shaft motion.
            chassis_back = center-np.array([self.amplitudes[0], 0.0, 0.0])
            knob_delta = np.array([
                self.amplitudes[2], 0.0, self.amplitudes[2]])
            bent_back = chassis_back+knob_delta
            bent_center = center+knob_delta
            knob_speeds = self.slow_speeds.copy()
            common = min(knob_speeds[0], knob_speeds[2])
            knob_speeds[[0, 2]] = common
            episodes.extend([
                self._hold("backdrive_static_start", center,
                           self.config.static_s),
                self._episode(
                    "backdrive_chassis_backward", "shaft_0_backdrive_probe",
                    "slow", 1, [center, chassis_back], self.slow_speeds),
                self._episode(
                    "backdrive_knob_forward_bend",
                    "shaft_2_backdrive_probe", "slow", 1,
                    [chassis_back, bent_back], knob_speeds),
                self._episode(
                    "backdrive_chassis_forward", "shaft_0_backdrive_probe",
                    "slow", 1, [bent_back, bent_center], self.slow_speeds),
                self._episode(
                    "backdrive_knob_backward_relax",
                    "shaft_2_backdrive_probe", "slow", 1,
                    [bent_center, center], knob_speeds),
                self._hold("backdrive_static_end", center,
                           self.config.static_s),
            ])
            return self._with_episode_starts(episodes)

        if self.schedule == "insertion_rotation":
            # Test the observed insertion-induced release of torsional
            # windup without conflating it with an MPPI decision.  Each probe
            # first establishes a signed rotation preload, holds the rotation
            # motor command fixed while insertion traverses both directions,
            # and then returns rotation to zero.  Returning to ``center``
            # between probes makes the commanded history explicit; it does
            # not claim to reset hidden physical torsion.
            episodes.append(self._hold(
                "static_start", center, self.config.static_s))
            for tier_index, (tier, speeds) in enumerate((
                    ("slow", self.slow_speeds),
                    ("fast", self.fast_speeds))):
                for repetition in range(1, self.config.repeats + 1):
                    signs = [1.0, -1.0]
                    # Balance which torsional branch inherits the preceding
                    # history at every speed and repetition.
                    if (tier_index + repetition) % 2 == 0:
                        signs.reverse()
                    for sign in signs:
                        branch = "pos" if sign > 0.0 else "neg"
                        stem = (
                            f"insertion_rotation_{branch}_{tier}_"
                            f"rep{repetition}")
                        rotated = center + sign * r
                        episodes.append(self._episode(
                            stem + "_bias_enter",
                            "shaft_1_coupling_setup", tier, repetition,
                            [center, rotated], speeds,
                            endpoint_holds=False,
                            final_hold=self.config.endpoint_dwell_s))
                        episodes.append(self._episode(
                            stem + "_probe",
                            "shaft_0_at_rotation_bias", tier, repetition,
                            [rotated, rotated + s, rotated,
                             rotated - s, rotated], speeds))
                        episodes.append(self._episode(
                            stem + "_bias_exit",
                            "shaft_1_coupling_setup", tier, repetition,
                            [rotated, center], speeds,
                            endpoint_holds=False,
                            final_hold=self.config.rotation_relax_s))
            episodes.append(self._hold(
                "static_end", center, self.config.static_s))
            return self._with_episode_starts(episodes)

        if not np.allclose(center, z):
            episodes.append(self._episode(
                "insertion_center_enter", "setup_shaft_0", "slow", 0,
                [z, center], self.slow_speeds,
                final_hold=self.config.between_episode_s))
        episodes.append(self._hold(
            "static_start", center, self.config.static_s))

        if do_insertion:
            for tier, speeds in (("slow", self.slow_speeds),
                                 ("fast", self.fast_speeds)):
                for repetition in range(1, self.config.repeats + 1):
                    episodes.append(self._episode(
                        f"insertion_{tier}_rep{repetition}", "shaft_0",
                        tier, repetition,
                        [center, center + s, center, center - s, center],
                        speeds))

        if do_rotation:
            for tier, speeds in (("slow", self.slow_speeds),
                                 ("fast", self.fast_speeds)):
                for repetition in range(1, self.config.repeats + 1):
                    episodes.append(self._episode(
                        f"rotation_{tier}_rep{repetition}", "shaft_1",
                        tier, repetition,
                        [center, center + r, center, center - r, center],
                        speeds, final_hold=self.config.rotation_relax_s))

        if do_bend_bias:
            episodes.append(self._episode(
                "bend_bias_enter", "setup_compensated", "slow", 0,
                [center, bias], self.slow_speeds,
                final_hold=self.config.endpoint_dwell_s))
        if do_tendon:
            for tier, speeds0 in (("slow", self.slow_speeds),
                                  ("fast", self.fast_speeds)):
                # Equal logical lin/bend speed is essential: firmware
                # computes raw shaft-0 request as u_lin-u_bend.
                speeds = speeds0.copy()
                common = min(speeds[0], speeds[2])
                speeds[[0, 2]] = common
                for repetition in range(1, self.config.repeats + 1):
                    episodes.append(self._episode(
                        f"bend_motor_only_{tier}_rep{repetition}",
                        "shaft_2", tier, repetition,
                        [bias, bias + b, bias, bias - b, bias], speeds))

        if do_compensated:
            for tier, speeds in (("slow", self.slow_speeds),
                                 ("fast", self.fast_speeds)):
                for repetition in range(1, self.config.repeats + 1):
                    episodes.append(self._episode(
                        f"bend_compensated_{tier}_rep{repetition}",
                        "shaft_0_plus_2_validation", tier, repetition,
                        [bias, bias + c_delta, bias,
                         bias - c_delta, bias], speeds))

        mixed = np.array([
            0.5 * self.amplitudes[0],
            0.5 * self.amplitudes[1],
            0.5 * self.amplitudes[2],
        ])
        if do_mixed:
            episodes.append(self._episode(
                "mixed_validation", "mixed_validation", "fast", 1,
                [bias, bias + mixed, bias, bias - mixed, bias],
                self.fast_speeds))
        if do_bend_bias:
            episodes.append(self._episode(
                "bend_bias_exit", "setup_compensated", "slow", 0,
                [bias, center], self.slow_speeds,
                final_hold=self.config.between_episode_s))
        episodes.append(self._hold(
            "static_end", center, self.config.static_s))
        if not np.allclose(center, z):
            episodes.append(self._episode(
                "insertion_center_exit", "setup_shaft_0", "slow", 0,
                [center, z], self.slow_speeds, endpoint_holds=False))

        return self._with_episode_starts(episodes)

    @staticmethod
    def _with_episode_starts(episodes):
        result = []
        start_s = 0.0
        for episode in episodes:
            result.append(CausalExperimentEpisode(
                episode.name, start_s, episode.duration_s, episode.segments,
                episode.excitation_basis, episode.speed_tier,
                episode.repetition,
                episode.timing_raw_start, episode.timing_raw_end,
                episode.timing_raw_speeds, episode.timing_raw_delays_s,
                episode.insertion_plateau_mm, episode.branch_order,
                episode.timing_raw_profile))
            start_s += episode.duration_s
        return tuple(result)

    @property
    def episodes(self):
        return self._episodes

    @property
    def episode_names(self):
        return [episode.name for episode in self._episodes]

    def episode_index(self, t: float) -> int:
        value = min(max(float(t), 0.0), max(0.0, self.duration - 1e-12))
        starts = np.asarray([episode.start_s for episode in self._episodes])
        return int(np.searchsorted(starts, value, side="right") - 1)

    def active_episode(self, t: float):
        return self._episodes[self.episode_index(t)]

    def state(self, t: float):
        if t >= self.duration:
            return np.zeros(3), np.zeros(3), np.zeros(3)
        episode = self.active_episode(t)
        return episode.state(float(t) - episode.start_s)

    def relative_position(self, t: float):
        return self.state(t)[0]

    def relative_velocity(self, t: float):
        return self.state(t)[1]

    def command_speed_limits(self, t: float):
        if t >= self.duration:
            return np.zeros(3)
        episode = self.active_episode(t)
        return episode.command_speed_limits(float(t) - episode.start_s)

    def timing_raw_velocity(self, t: float) -> np.ndarray | None:
        """Return raw shaft-0/shaft-2 velocity for a Phase-3 sample.

        ``None`` distinguishes ordinary setup/recovery episodes from a timing
        pulse.  Consumers can therefore validate the physical command before
        publishing without reconstructing the intended signal from a floored
        logical command.
        """
        if t >= self.duration:
            return None
        episode = self.active_episode(t)
        if not episode.is_timing_episode:
            return None
        return episode.timing_raw_velocity(float(t) - episode.start_s)

    def step(self, t: float):
        return self.relative_position(t)

    def is_done(self, t: float):
        return float(t) >= self.duration

    def enforce_basis(self, t: float, velocity):
        """Preserve exact raw-motor isolation after feedback correction."""
        result = np.asarray(velocity, dtype=float).copy()
        episode = self.active_episode(t)
        if episode.excitation_basis in {
                "shaft_2", "shaft_2_backdrive_probe"}:
            # Bend tracking is authoritative; copy its signed command to lin.
            result[0] = result[2]
        elif episode.is_timing_episode:
            # Preserve the planned raw-shaft onset ordering even when the
            # position floor tracker is correcting the logical reference.
            raw0 = result[0] - result[2]
            raw2 = result[2]
            if not episode._timing_axis_active(
                    0, float(t) - episode.start_s):
                raw0 = 0.0
            if not episode._timing_axis_active(
                    1, float(t) - episode.start_s):
                raw2 = 0.0
            result[0] = raw0 + raw2
            result[2] = raw2
        elif episode.excitation_basis == "shaft_0_at_rotation_bias":
            # The measured effect is physical rotation released by insertion;
            # never let feedback shaping issue a rotation or tendon command
            # during the probe itself.
            result[1] = 0.0
            result[2] = 0.0
        elif episode.excitation_basis == "shaft_1_coupling_setup":
            # Establish/unwind torsion with the other commanded axes fixed.
            result[0] = 0.0
            result[2] = 0.0
        elif episode.excitation_basis == "compensated_bend_at_insertion":
            # The experiment measures production compensated bending at a
            # fixed logical insertion plateau.  Feedback shaping may correct
            # bend tracking, but cannot introduce insertion or rotation.
            result[0] = 0.0
            result[1] = 0.0
        elif episode.excitation_basis == "insertion_plateau_transition":
            # Move only logical insertion between measured plateaus while the
            # tendon remains at its common bias.
            result[1] = 0.0
            result[2] = 0.0
        elif episode.excitation_basis == "compensated_bend_setup":
            result[0] = 0.0
            result[1] = 0.0
        return result

    def margin_violation(self, absolute_position):
        position = self._vector(absolute_position, "absolute_position")
        below = np.flatnonzero(position < self.usable_lower - 1e-9)
        above = np.flatnonzero(position > self.usable_upper + 1e-9)
        # The run-start bend may intentionally be below the interior bias. It
        # is safe during the initial static and bias-entry/exit phases because
        # it is still inside the hard limit. All purposeful excitation points
        # were separately validated against usable limits.
        active = np.concatenate((below, above))
        return None if active.size == 0 else int(active[0])

    @property
    def metadata(self):
        return {
            "generator_version": self.version,
            "schedule": self.schedule,
            "duration_s": self.duration,
            "start_position": self.start_position.tolist(),
            "experiment_center": self.experiment_center.tolist(),
            "insertion_centering_enabled": bool(
                self.config.allow_insertion_centering),
            "requested_amplitudes": list(self.config.amplitudes),
            "resolved_amplitudes": self.amplitudes.tolist(),
            "peak_to_peak_excursions": (2.0 * self.amplitudes).tolist(),
            "usable_position_lower": self.usable_lower.tolist(),
            "usable_position_upper": self.usable_upper.tolist(),
            "bend_bias_position": self.config.bend_bias_position,
            "insertion_plateaus": list(self.config.insertion_plateaus),
            "insertion_plateau_visits": int(
                self.config.insertion_plateau_visits),
            "insertion_plateau_dwell_s": float(
                self.config.insertion_plateau_dwell_s),
            "tendon_sweep_limits": list(
                self.config.tendon_sweep_limits),
            "slow_speeds": self.slow_speeds.tolist(),
            "fast_speeds": self.fast_speeds.tolist(),
            "repeats": self.config.repeats,
            "timing_leads_ms": list(self.config.timing_leads_ms),
            "timing_direction": int(self.config.timing_direction),
            "command_bases": {
                "shaft_0": "[v,0,0]",
                "shaft_1": "[0,v,0]",
                "shaft_2": "[v,0,v]",
                "shaft_0_plus_2_validation": "[0,0,v]",
                "shaft_0_at_rotation_bias": "[v,0,0] at fixed q_rot",
                "shaft_1_coupling_setup": "[0,v,0]",
                "compensated_bend_at_insertion": "[0,0,v] at fixed q_lin",
                "insertion_plateau_transition": "[v,0,0] at tendon bias",
            },
            "episodes": [{
                "name": episode.name,
                "start_s": episode.start_s,
                "duration_s": episode.duration_s,
                "excitation_basis": episode.excitation_basis,
                "speed_tier": episode.speed_tier,
                "repetition": episode.repetition,
                "insertion_plateau_mm": episode.insertion_plateau_mm,
                "branch_order": episode.branch_order,
                "command_speed_limits": (
                    episode.maximum_command_speed_limits.tolist()),
                "planned_raw_onset_delays_s": (
                    None if episode.planned_raw_onset_delays_s is None else
                    list(episode.planned_raw_onset_delays_s)),
                "raw_profile": (
                    None if episode.timing_raw_profile is None else
                    [list(segment) for segment in episode.timing_raw_profile]),
            } for episode in self._episodes],
        }

    def _validate_plan(self):
        times = np.arange(0.0, self.duration + self.dt, self.dt)
        relative = np.asarray([self.relative_position(t) for t in times])
        velocity = np.asarray([self.relative_velocity(t) for t in times])
        absolute = relative + self.start_position
        if not np.all(np.isfinite(absolute)) or not np.all(
                np.isfinite(velocity)):
            raise ValueError("causal plan contains non-finite samples")
        if np.any(absolute < self.lower_limits - 1e-9) or np.any(
                absolute > self.upper_limits + 1e-9):
            raise ValueError("causal plan exceeds hard position limits")
        if np.any(np.abs(velocity) > self.maximum_speeds + 1e-7):
            raise ValueError("causal plan exceeds joint speed limits")
        limits = np.asarray([self.command_speed_limits(t) for t in times])
        if np.any(np.abs(velocity) > limits + 1e-7):
            raise ValueError("causal feed-forward exceeds episode ceiling")
        if not (np.allclose(relative[0], 0.0)
                and np.allclose(relative[-1], 0.0)
                and np.allclose(velocity[0], 0.0)
                and np.allclose(velocity[-1], 0.0)):
            raise ValueError("causal plan must start and finish at rest")
        for left, right in zip(self._episodes[:-1], self._episodes[1:]):
            left_state = left.state(left.duration_s)
            right_state = right.state(0.0)
            position_continuous = np.allclose(
                left_state[0], right_state[0], atol=1e-9)
            # A timing episode deliberately applies a velocity step at its
            # raw-shaft onset. All other episode boundaries remain at rest.
            velocity_continuous = (
                right.is_timing_episode
                or np.allclose(left_state[1], right_state[1], atol=1e-9))
            if not (position_continuous and velocity_continuous):
                raise ValueError(
                    f"episode boundary is discontinuous: {left.name} -> "
                    f"{right.name}")
