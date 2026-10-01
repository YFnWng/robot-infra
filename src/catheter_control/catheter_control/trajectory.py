"""Deterministic waypoint timing for catheter tip trajectories."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectoryUpdate:
    waypoint_index: int
    waypoint_count: int
    elapsed_s: float
    remaining_s: float
    tracking_error_mm: float
    within_tolerance: bool
    target_changed: bool
    complete: bool
    reached_waypoints: int
    timed_out_waypoints: int


class TrajectorySequencer:
    """Advance waypoints on settled tracking or an independent time budget."""

    def __init__(self, waypoints_m, timeouts_s, *, tolerance_mm: float,
                 settle_time_s: float):
        waypoints = np.asarray(waypoints_m, dtype=np.float64)
        timeouts = np.asarray(timeouts_s, dtype=np.float64)
        if (waypoints.ndim != 2 or waypoints.shape[1:] != (3,)
                or len(waypoints) < 1 or not np.all(np.isfinite(waypoints))):
            raise ValueError("waypoints must be a nonempty finite Nx3 array")
        if (timeouts.shape != (len(waypoints),)
                or not np.all(np.isfinite(timeouts))
                or np.any(timeouts <= 0.0)):
            raise ValueError(
                "timeouts must contain one positive value per waypoint")
        if not np.isfinite(tolerance_mm) or tolerance_mm <= 0.0:
            raise ValueError("tolerance_mm must be finite and positive")
        if not np.isfinite(settle_time_s) or settle_time_s < 0.0:
            raise ValueError("settle_time_s must be finite and nonnegative")
        self.waypoints = waypoints.copy()
        self.timeouts = timeouts.copy()
        self.tolerance_mm = float(tolerance_mm)
        self.settle_time_s = float(settle_time_s)
        self.index = 0
        self.started_at = None
        self.within_since = None
        self.reached_waypoints = 0
        self.timed_out_waypoints = 0
        self.complete = False

    @property
    def target(self) -> np.ndarray:
        return self.waypoints[self.index].copy()

    def start(self, now: float) -> None:
        if not np.isfinite(now):
            raise ValueError("start time must be finite")
        self.started_at = float(now)

    def update(self, tip_m, now: float) -> TrajectoryUpdate:
        if self.started_at is None:
            raise RuntimeError("trajectory has not started")
        if self.complete:
            raise RuntimeError("trajectory is already complete")
        tip = np.asarray(tip_m, dtype=np.float64)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError("tip must contain three finite coordinates")
        if not np.isfinite(now) or now < self.started_at:
            raise ValueError("update time must be finite and monotonic")
        now = float(now)
        elapsed = now-self.started_at
        error = 1000.0*float(np.linalg.norm(self.waypoints[self.index]-tip))
        within = error <= self.tolerance_mm
        if within:
            self.within_since = self.within_since or now
        else:
            self.within_since = None
        reached = bool(
            within and now-self.within_since >= self.settle_time_s)
        timed_out = elapsed >= self.timeouts[self.index]
        changed = False
        if reached or timed_out:
            if reached:
                self.reached_waypoints += 1
            else:
                self.timed_out_waypoints += 1
            if self.index+1 == len(self.waypoints):
                self.complete = True
            else:
                self.index += 1
                self.started_at = now
                self.within_since = None
                changed = True
                # Feedback describes the newly active target, not the target
                # that caused this transition.
                elapsed = 0.0
                error = 1000.0*float(np.linalg.norm(
                    self.waypoints[self.index]-tip))
                within = error <= self.tolerance_mm
        remaining = (0.0 if self.complete else max(
            0.0, self.timeouts[self.index]-(now-self.started_at)))
        return TrajectoryUpdate(
            waypoint_index=self.index,
            waypoint_count=len(self.waypoints),
            elapsed_s=elapsed,
            remaining_s=remaining,
            tracking_error_mm=error,
            within_tolerance=within,
            target_changed=changed,
            complete=self.complete,
            reached_waypoints=self.reached_waypoints,
            timed_out_waypoints=self.timed_out_waypoints)
