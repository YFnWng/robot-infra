"""Causal scheduling primitives for the single-owner estimator."""
from __future__ import annotations

import math


class CausalMarkerSchedule:
    """Rate-limit marker work and defer observations ahead of model state."""

    def __init__(self, rate_hz: float, started_s: float):
        rate_hz = float(rate_hz)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        self.period_s = 1.0/rate_hz
        self.next_allowed_s = float(started_s)

    def decision(self, now_s: float, marker_timestamp_ns: int,
                 state_timestamp_ns: int | None) -> str:
        # ROS timer phases and decimal rates are floating-point values. Treat
        # sub-nanosecond representation error as on-time, not a missed release.
        if float(now_s)+1e-9 < self.next_allowed_s:
            return "rate_limited"
        if state_timestamp_ns is None:
            return "awaiting_estimator"
        if int(marker_timestamp_ns) > int(state_timestamp_ns):
            return "awaiting_encoder"
        return "ready"

    def commit(self, now_s: float):
        now_s = float(now_s)
        # Advance the absolute release phase instead of restarting the period
        # at callback completion. With a 50 Hz owner and a 20 Hz marker rate,
        # relative scheduling quantizes every release to 60 ms (16.7 Hz).
        # Absolute releases alternate across owner ticks and retain 20 Hz on
        # average without ever accumulating a marker backlog.
        elapsed_periods = max(
            1, int(math.floor(
                (now_s-self.next_allowed_s)/self.period_s))+1)
        self.next_allowed_s += elapsed_periods*self.period_s
