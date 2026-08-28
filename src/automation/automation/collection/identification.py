"""Deterministic, episode-labelled excitation for robot system identification.

The generator operates in relative physical joint coordinates
``[insertion_mm, rotation_deg, bend_mm]``.  It deliberately does not clip its
output: the complete plan is resolved against the measured run-start position
and validated before control is enabled.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


_D1_MAX = 1.875  # max derivative of 10u^3 - 15u^4 + 6u^5


def _smooth(u):
    return u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))


def _smooth_d1(u):
    return 30.0 * u * u * (u - 1.0) * (u - 1.0)


def _smooth_d2(u):
    return 60.0 * u * (2.0 * u * u - 3.0 * u + 1.0)


@dataclass(frozen=True)
class IdentificationConfig:
    amplitudes: tuple[float, float, float] = (20.0, 100.0, 6.0)
    margins: tuple[float, float, float] = (0.5, 5.0, 0.25)
    minimum_amplitudes: tuple[float, float, float] = (2.0, 20.0, 1.0)
    settle_s: float = 2.0
    dwell_s: float = 1.0
    hold_s: float = 2.0
    slow_fraction: float = 0.30
    medium_fraction: float = 0.70
    seed: int = 1
    max_duration_s: float = 900.0


@dataclass(frozen=True)
class _Segment:
    kind: str
    duration: float
    start: np.ndarray
    end: np.ndarray
    frequencies: np.ndarray | None = None
    phases: np.ndarray | None = None

    def state(self, t: float):
        if self.duration <= 0.0:
            return self.end.copy(), np.zeros(3), np.zeros(3)
        u = min(1.0, max(0.0, float(t) / self.duration))
        if self.kind == "hold":
            return self.start.copy(), np.zeros(3), np.zeros(3)
        if self.kind == "move":
            delta = self.end - self.start
            return (
                self.start + delta * _smooth(u),
                delta * _smooth_d1(u) / self.duration,
                delta * _smooth_d2(u) / self.duration ** 2,
            )
        if self.kind == "multisine":
            # sin(pi*u)^4 makes position, velocity and acceleration all zero
            # at the episode boundaries.
            x = math.pi * u
            window = math.sin(x) ** 4
            window_d1 = 4.0 * math.pi * math.sin(x) ** 3 * math.cos(x)
            window_d2 = 4.0 * math.pi ** 2 * (
                3.0 * math.sin(x) ** 2 * math.cos(x) ** 2
                - math.sin(x) ** 4)
            omega = 2.0 * math.pi * self.frequencies
            angle = omega * u + self.phases
            signal = np.sin(angle)
            signal_d1 = omega * np.cos(angle)
            signal_d2 = -(omega ** 2) * signal
            # Insertion and bending are one-sided from the run-start pose;
            # rotation is the only signed excursion.
            one_sided = np.array([True, False, True])
            signal = np.where(one_sided, 0.5 * (1.0 + signal), signal)
            signal_d1 = np.where(one_sided, 0.5 * signal_d1, signal_d1)
            signal_d2 = np.where(one_sided, 0.5 * signal_d2, signal_d2)
            scale = self.end
            return (
                scale * window * signal,
                scale * (window_d1 * signal + window * signal_d1)
                / self.duration,
                scale * (window_d2 * signal + 2.0 * window_d1 * signal_d1
                         + window * signal_d2) / self.duration ** 2,
            )
        raise ValueError(f"unknown identification segment kind {self.kind!r}")


@dataclass(frozen=True)
class IdentificationEpisode:
    name: str
    start_s: float
    duration_s: float
    segments: tuple[_Segment, ...]

    def state(self, local_t: float):
        remaining = min(max(float(local_t), 0.0), self.duration_s)
        for segment in self.segments:
            if remaining <= segment.duration:
                return segment.state(remaining)
            remaining -= segment.duration
        return self.segments[-1].state(self.segments[-1].duration)


class IdentificationGenerator:
    """Build and sample one complete hardware-safe identification plan."""

    version = "hardware_identification_v1"

    def __init__(
        self,
        start_position: Iterable[float],
        lower_limits: Iterable[float],
        upper_limits: Iterable[float],
        minimum_speeds: Iterable[float],
        maximum_speeds: Iterable[float],
        dt: float,
        config: IdentificationConfig | None = None,
    ) -> None:
        self.config = config or IdentificationConfig()
        self.start_position = self._vector(start_position, "start_position")
        self.lower_limits = self._vector(lower_limits, "lower_limits")
        self.upper_limits = self._vector(upper_limits, "upper_limits")
        self.minimum_speeds = self._vector(minimum_speeds, "minimum_speeds")
        self.maximum_speeds = self._vector(maximum_speeds, "maximum_speeds")
        self.dt = float(dt)
        if self.dt <= 0.0 or not math.isfinite(self.dt):
            raise ValueError("dt must be finite and positive")
        if np.any(self.upper_limits <= self.lower_limits):
            raise ValueError("upper limits must exceed lower limits")
        if np.any(self.maximum_speeds <= 0.0):
            raise ValueError("maximum speeds must be positive")
        self._validate_config()

        margins = np.asarray(self.config.margins, dtype=float)
        usable_lower = self.lower_limits + margins
        usable_upper = self.upper_limits - margins
        if np.any(self.start_position < self.lower_limits) or np.any(
                self.start_position > self.upper_limits):
            raise ValueError(
                "run-start position is outside identification limits: "
                f"start={self.start_position.tolist()} lower="
                f"{self.lower_limits.tolist()} upper={self.upper_limits.tolist()}")

        requested = np.asarray(self.config.amplitudes, dtype=float)
        # Insertion and bending use positive excursions. Rotation requires both
        # signed branches so the resolved amplitude is symmetric about run start.
        available = np.array([
            usable_upper[0] - self.start_position[0],
            min(usable_upper[1] - self.start_position[1],
                self.start_position[1] - usable_lower[1]),
            usable_upper[2] - self.start_position[2],
        ])
        self.amplitudes = np.minimum(requested, available)
        minimum = np.asarray(self.config.minimum_amplitudes, dtype=float)
        if np.any(self.amplitudes + 1e-12 < minimum):
            raise ValueError(
                "insufficient safe excursion for identification: requested="
                f"{requested.tolist()} resolved={self.amplitudes.tolist()} "
                f"minimum={minimum.tolist()}")

        self.slow_speeds = np.maximum(
            self.minimum_speeds,
            self.config.slow_fraction * self.maximum_speeds)
        self.medium_speeds = self.config.medium_fraction * self.maximum_speeds
        self.medium_enabled = self.medium_speeds >= 1.5 * self.slow_speeds
        self._episodes = self._build_episodes()
        self.duration = sum(ep.duration_s for ep in self._episodes)
        if self.duration > self.config.max_duration_s:
            raise ValueError(
                f"identification duration {self.duration:.3f}s exceeds "
                f"maximum {self.config.max_duration_s:.3f}s")
        self._validate_plan()

    @staticmethod
    def _vector(values, name):
        result = np.asarray(tuple(values), dtype=float)
        if result.shape != (3,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain three finite values")
        return result

    def _validate_config(self):
        for name in ("amplitudes", "margins", "minimum_amplitudes"):
            value = self._vector(getattr(self.config, name), name)
            if np.any(value < 0.0):
                raise ValueError(f"{name} must be non-negative")
        for name in ("settle_s", "dwell_s", "hold_s", "max_duration_s"):
            if not math.isfinite(getattr(self.config, name)) or getattr(
                    self.config, name) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 < self.config.slow_fraction <= 1.0:
            raise ValueError("slow_fraction must be in (0, 1]")
        if not 0.0 < self.config.medium_fraction <= 1.0:
            raise ValueError("medium_fraction must be in (0, 1]")

    @staticmethod
    def _move_duration(start, end, speeds):
        moving = np.abs(end - start) > 1e-12
        if not np.any(moving):
            return 0.0
        return float(np.max(
            _D1_MAX * np.abs(end[moving] - start[moving]) / speeds[moving]))

    def _waypoint_episode(self, name, waypoints, speeds, holds=None):
        points = [np.asarray(p, dtype=float) for p in waypoints]
        segments = []
        holds = holds or {}
        for index in range(len(points) - 1):
            duration = self._move_duration(points[index], points[index + 1], speeds)
            segments.append(_Segment(
                "move", duration, points[index], points[index + 1]))
            hold = float(holds.get(index + 1, 0.0))
            if hold > 0.0:
                segments.append(_Segment(
                    "hold", hold, points[index + 1], points[index + 1]))
        return name, tuple(segments)

    def _build_episodes(self):
        z = np.zeros(3)
        lin = np.array([self.amplitudes[0], 0.0, 0.0])
        rot = np.array([0.0, self.amplitudes[1], 0.0])
        bend = np.array([0.0, 0.0, self.amplitudes[2]])
        slow, medium = self.slow_speeds, self.medium_speeds
        specs = []

        def hold(name, duration):
            specs.append((name, (_Segment("hold", duration, z, z),)))

        def path(name, points, speeds, holds=None):
            specs.append(self._waypoint_episode(name, points, speeds, holds))

        hold("settle_start", self.config.settle_s)
        path("insertion_slow", [z, lin, z], slow)
        hold("dwell_after_insertion_slow", self.config.dwell_s)
        if self.medium_enabled[0]:
            path("insertion_medium", [z, lin, z], medium)
        else:
            hold("insertion_medium_skipped", self.config.dwell_s)
        hold("dwell_after_insertion_medium", self.config.dwell_s)
        path("rotation_slow", [z, rot, z, -rot, z], slow)
        hold("dwell_after_rotation_slow", self.config.dwell_s)
        if self.medium_enabled[1]:
            path("rotation_medium", [z, rot, z, -rot, z], medium)
        else:
            hold("rotation_medium_skipped", self.config.dwell_s)
        hold("dwell_after_rotation_medium", self.config.dwell_s)
        path("bend_out_and_back", [z, bend, z], slow)
        path("bend_hold_unload_relax", [z, bend, z], slow, {
            1: self.config.hold_s, 2: self.config.hold_s})
        path("repeated_bend_loops", [z, bend, z, bend, z], slow)

        mid_lin = 0.5 * lin
        path("bend_at_mid_insertion",
             [z, mid_lin, mid_lin + bend, mid_lin, z], slow)
        path("insertion_bend_interaction",
             [z, 0.6 * lin + 0.3 * bend, 0.3 * lin + 0.7 * bend,
              0.7 * lin + 0.6 * bend, z], slow)
        path("rotation_bend_interaction",
             [z, 0.5 * rot + 0.3 * bend, -0.4 * rot + 0.7 * bend,
              0.6 * rot + 0.5 * bend, z], slow)
        path("insertion_rotation_interaction",
             [z, 0.5 * lin + 0.4 * rot, 0.8 * lin - 0.4 * rot,
              0.3 * lin + 0.6 * rot, z], slow)

        target = 0.45 * lin + 0.35 * rot + 0.45 * bend
        path("opposite_history_revisit",
             [z, 0.7 * lin, target, z, -0.6 * rot, target, z], slow,
             {2: self.config.dwell_s, 5: self.config.dwell_s})

        rng = np.random.default_rng(self.config.seed)
        phases = rng.uniform(-math.pi, math.pi, size=3)
        pe_amplitude = 0.35 * self.amplitudes
        pe_frequencies = np.array([3.0, 5.0, 7.0])
        # The dimensionless frequencies specify cycles over the full segment.
        # Increase duration until dense analytic sampling satisfies every speed.
        pe_duration = 10.0
        while True:
            segment = _Segment(
                "multisine", pe_duration, z, pe_amplitude,
                pe_frequencies, phases)
            times = np.linspace(0.0, pe_duration, 2001)
            peak = np.max(np.abs([segment.state(t)[1] for t in times]), axis=0)
            if np.all(peak <= slow * (1.0 + 1e-10)):
                break
            pe_duration *= float(np.max(peak / slow)) * 1.001
        specs.append(("coupled_persistent_excitation", (segment,)))
        hold("settle_end", self.config.settle_s)

        episodes = []
        start = 0.0
        for name, segments in specs:
            duration = sum(segment.duration for segment in segments)
            episodes.append(IdentificationEpisode(name, start, duration, segments))
            start += duration
        return tuple(episodes)

    @property
    def episodes(self):
        return self._episodes

    @property
    def episode_names(self):
        return [episode.name for episode in self._episodes]

    @property
    def episode_starts(self):
        return [episode.start_s for episode in self._episodes]

    def episode_index(self, t: float) -> int:
        if not self._episodes:
            return -1
        value = min(max(float(t), 0.0), max(0.0, self.duration - 1e-12))
        starts = np.asarray(self.episode_starts)
        return int(np.searchsorted(starts, value, side="right") - 1)

    def state(self, t: float):
        if t >= self.duration:
            return np.zeros(3), np.zeros(3), np.zeros(3)
        index = self.episode_index(t)
        episode = self._episodes[index]
        return episode.state(float(t) - episode.start_s)

    def relative_position(self, t: float):
        return self.state(t)[0]

    def relative_velocity(self, t: float):
        return self.state(t)[1]

    def step(self, t: float):
        return self.relative_position(t)

    def is_done(self, t: float):
        return float(t) >= self.duration

    @property
    def metadata(self):
        return {
            "generator_version": self.version,
            "duration_s": self.duration,
            "start_position": self.start_position.tolist(),
            "requested_amplitudes": list(self.config.amplitudes),
            "resolved_amplitudes": self.amplitudes.tolist(),
            "margins": list(self.config.margins),
            "slow_speeds": self.slow_speeds.tolist(),
            "medium_speeds": self.medium_speeds.tolist(),
            "medium_enabled": self.medium_enabled.tolist(),
            "seed": self.config.seed,
            "episodes": [{
                "name": episode.name,
                "start_s": episode.start_s,
                "duration_s": episode.duration_s,
            } for episode in self._episodes],
        }

    def _validate_plan(self):
        sample_times = np.arange(0.0, self.duration + self.dt, self.dt)
        positions = np.asarray([self.relative_position(t) for t in sample_times])
        velocities = np.asarray([self.relative_velocity(t) for t in sample_times])
        absolute = positions + self.start_position
        if not np.all(np.isfinite(absolute)) or not np.all(np.isfinite(velocities)):
            raise ValueError("identification plan contains non-finite samples")
        if np.any(absolute < self.lower_limits - 1e-9) or np.any(
                absolute > self.upper_limits + 1e-9):
            raise ValueError("identification plan exceeds joint position limits")
        if np.any(np.abs(velocities) > self.maximum_speeds + 1e-7):
            raise ValueError("identification plan exceeds joint speed limits")
        q0, v0, _ = self.state(0.0)
        q1, v1, _ = self.state(self.duration)
        if not (np.allclose(q0, 0.0) and np.allclose(v0, 0.0)
                and np.allclose(q1, 0.0) and np.allclose(v1, 0.0)):
            raise ValueError("identification plan must start and finish at rest")
        for episode in self._episodes:
            before = episode.state(0.0)
            after = episode.state(episode.duration_s)
            if not (np.allclose(before[0], 0.0, atol=1e-9)
                    and np.allclose(before[1], 0.0, atol=1e-9)
                    and np.allclose(after[0], 0.0, atol=1e-9)
                    and np.allclose(after[1], 0.0, atol=1e-9)):
                raise ValueError(f"episode {episode.name} is not boundary-continuous")
