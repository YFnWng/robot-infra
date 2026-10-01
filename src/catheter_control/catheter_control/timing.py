"""Small bounded timing probes for controller audit diagnostics."""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import math
import threading

import numpy as np


@dataclass
class _Window:
    count: int = 0
    total_ms: float = 0.0
    maximum_ms: float = 0.0
    samples_ms: deque | None = None


class TimingWindows:
    """Accumulate bounded per-diagnostic-window timing summaries.

    Recording performs no allocation after a metric's first observation. The
    dedicated lock is never held while application locks are acquired.
    """

    def __init__(self, maximum_samples: int = 2048):
        if maximum_samples < 1:
            raise ValueError("maximum_samples must be positive")
        self._lock = threading.Lock()
        self._windows: dict[str, _Window] = {}
        self._latest_ms: dict[str, float] = {}
        self._maximum_samples = int(maximum_samples)

    def record_seconds(self, name: str, seconds: float):
        milliseconds = 1e3*float(seconds)
        if not math.isfinite(milliseconds):
            return
        with self._lock:
            window = self._windows.get(name)
            if window is None:
                window = _Window(samples_ms=deque(
                    maxlen=self._maximum_samples))
                self._windows[name] = window
            window.count += 1
            window.total_ms += milliseconds
            if window.count == 1:
                window.maximum_ms = milliseconds
            else:
                window.maximum_ms = max(window.maximum_ms, milliseconds)
            window.samples_ms.append(milliseconds)
            self._latest_ms[name] = milliseconds

    def latest(self) -> dict[str, float]:
        """Return the most recent value of every metric without resetting."""
        with self._lock:
            return dict(self._latest_ms)

    def snapshot_and_reset(self) -> dict[str, float | int]:
        with self._lock:
            active = self._windows
            self._windows = {}
        result: dict[str, float | int] = {}
        for name, window in active.items():
            prefix = f"timing_{name}_window"
            result[f"{prefix}_count"] = window.count
            result[f"{prefix}_mean_ms"] = window.total_ms/window.count
            samples = np.asarray(window.samples_ms, dtype=np.float64)
            result[f"{prefix}_p50_ms"] = float(np.percentile(samples, 50))
            result[f"{prefix}_p95_ms"] = float(np.percentile(samples, 95))
            result[f"{prefix}_p99_ms"] = float(np.percentile(samples, 99))
            result[f"{prefix}_max_ms"] = window.maximum_ms
        return result


class PeriodicTimerProbe:
    """Estimate start lateness relative to a periodic release phase."""

    def __init__(self, period_s: float, started_s: float):
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be positive and finite")
        self.period_s = float(period_s)
        self.next_release_s = float(started_s)+self.period_s

    def observe(self, started_s: float) -> float:
        """Return nonnegative lateness and advance to the next release."""
        started_s = float(started_s)
        lateness = max(0.0, started_s-self.next_release_s)
        elapsed_periods = max(
            1, int(math.floor(
                (started_s-self.next_release_s)/self.period_s))+1)
        self.next_release_s += elapsed_periods*self.period_s
        return lateness
