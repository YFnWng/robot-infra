"""Opt-in process stall diagnostics for the camera node (ROS-independent).

Both camera rigs usually skip frames at the same instant, which points to a
process-wide stall keeping both capture threads away from grab(). These
monitors measure the two leading candidates without changing behaviour:
garbage-collection pauses and interpreter/CPU scheduling stalls (GIL holds).
Each ``window()`` returns maxima since the previous call plus cumulative
counters.
"""
from __future__ import annotations

import gc
import threading
import time


class GcPauseMonitor:
    """Time every garbage collection through ``gc.callbacks``."""

    def __init__(self):
        self._lock = threading.Lock()
        self._started_ns = None
        self._count = 0
        self._total_ms = 0.0
        self._window_max_ms = 0.0
        self._window_max_generation = -1
        self._installed = False

    def _callback(self, phase, info):
        now = time.perf_counter_ns()
        if phase == "start":
            self._started_ns = now
            return
        if self._started_ns is None:
            return
        pause_ms = (now - self._started_ns)*1e-6
        self._started_ns = None
        with self._lock:
            self._count += 1
            self._total_ms += pause_ms
            if pause_ms > self._window_max_ms:
                self._window_max_ms = pause_ms
                self._window_max_generation = int(info.get("generation", -1))

    def start(self):
        if not self._installed:
            gc.callbacks.append(self._callback)
            self._installed = True

    def stop(self):
        if self._installed:
            gc.callbacks.remove(self._callback)
            self._installed = False

    def window(self):
        with self._lock:
            values = {
                "gc_pause_max_ms": self._window_max_ms,
                "gc_pause_max_generation": self._window_max_generation,
                "gc_pauses": self._count,
                "gc_pause_total_ms": self._total_ms,
            }
            self._window_max_ms = 0.0
            self._window_max_generation = -1
        return values


class SchedulingProbe:
    """Thread that sleeps ``period_s`` and records how late it wakes.

    A late wake-up means the thread could not run: the GIL was held, or the
    CPU was busy. Lateness over ``stall_threshold_ms`` is counted as a stall.
    """

    def __init__(self, period_s: float = 0.002, stall_threshold_ms: float = 10.0):
        if not period_s > 0.0 or not stall_threshold_ms > 0.0:
            raise ValueError("probe period and stall threshold must be positive")
        self.period_s = float(period_s)
        self.stall_threshold_ms = float(stall_threshold_ms)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._window_max_ms = 0.0
        self._stalls = 0
        self._wakes = 0

    def _run(self):
        period_ns = int(self.period_s*1e9)
        while not self._stop.is_set():
            target = time.perf_counter_ns() + period_ns
            time.sleep(self.period_s)
            late_ms = max(0.0, (time.perf_counter_ns() - target)*1e-6)
            with self._lock:
                self._wakes += 1
                self._window_max_ms = max(self._window_max_ms, late_ms)
                self._stalls += int(late_ms > self.stall_threshold_ms)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._run, name="stall-probe", daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def window(self):
        with self._lock:
            values = {
                "probe_late_max_ms": self._window_max_ms,
                "probe_stalls": self._stalls,
                "probe_wakes": self._wakes,
            }
            self._window_max_ms = 0.0
        return values
