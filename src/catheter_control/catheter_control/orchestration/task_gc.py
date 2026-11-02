"""Process-local, bounded GC deferral shared by both controller families.

Only cyclic GC is deferred; reference counting remains enabled. Maintenance
never waits for recurrent work, and no collection runs while that work is live.
"""
from contextlib import contextmanager
from functools import wraps
import gc
import math
import os
import threading
import time


def resident_bytes():
    with open('/proc/self/statm', encoding='ascii') as stream:
        return int(stream.read().split()[1]) * os.sysconf('SC_PAGE_SIZE')


class TaskGcPolicy:
    def __init__(self, *, enabled=False, maximum_s=300., growth_mb=256.,
                 clock=time.monotonic, rss=resident_bytes, collector=gc):
        if not all(math.isfinite(x) and x > 0 for x in (maximum_s, growth_mb)):
            raise ValueError('task GC duration and RSS growth bounds must be positive finite')
        self.enabled = enabled
        self.maximum_s = maximum_s
        self.growth_bytes = growth_mb * 1024**2
        self.clock, self.rss, self.gc = clock, rss, collector
        self.lock = threading.Lock()
        self.workers = 0
        self.active = self.pending = self.collecting = False
        self.original = None
        self.collections = 0
        self.last_collection_ms = 0.
        self.peak_growth_bytes = 0
        self.last_guard_reason = ''

    @contextmanager
    def work(self):
        with self.lock:
            admitted = not self.collecting
            if admitted:
                self.workers += 1
        try:
            yield admitted
        finally:
            if admitted:
                with self.lock:
                    self.workers -= 1

    def begin(self):
        with self.lock:
            if self.pending or self.collecting:
                return False
            if not self.enabled or self.active:
                return True
            # Sample before changing process-global policy; failure leaves it intact.
            baseline = self.rss()
            self.original = (self.gc.isenabled(), self.gc.get_threshold())
            self.started, self.baseline = self.clock(), baseline
            self.peak_growth_bytes = 0
            self.last_guard_reason = ''
            self.gc.disable()
            self.active = True
            return True

    def stop(self):
        with self.lock:
            if self.active:
                self.active = False
                self.pending = True

    def guard_reason(self):
        with self.lock:
            if not self.active:
                return ''
            try:
                growth = max(0, self.rss() - self.baseline)
                self.peak_growth_bytes = max(self.peak_growth_bytes, growth)
                reason = ('task_gc_duration_limit' if self.clock()-self.started >= self.maximum_s
                          else 'task_gc_rss_growth_limit' if growth >= self.growth_bytes else '')
            except (OSError, ValueError, IndexError):
                reason = 'task_gc_rss_unavailable'
            self.last_guard_reason = reason
            return reason

    def reserve_collection(self):
        """Caller must also exclude arming/output under its controller lock."""
        with self.lock:
            if not self.pending or self.active:
                return False
            # Close admission first so a continuously busy estimator cannot
            # starve maintenance. Existing callbacks drain without waiting;
            # the next maintenance tick collects only after the count is zero.
            self.collecting = True
            return self.workers == 0

    def collect_reserved(self):
        with self.lock:
            if not self.collecting or self.workers:
                raise RuntimeError('GC maintenance was not reserved')
        started = self.clock()
        try:
            self.gc.collect(2)
        finally:
            with self.lock:
                enabled, thresholds = self.original
                self.gc.set_threshold(*thresholds)
                self.gc.enable() if enabled else self.gc.disable()
                self.original = None
                self.pending = self.collecting = False
                self.collections += 1
                self.last_collection_ms = 1000*(self.clock()-started)

    def summary(self):
        with self.lock:
            return dict(enabled=self.enabled, active=self.active,
                        maximum_s=self.maximum_s,
                        growth_limit_mb=self.growth_bytes/1024**2,
                        pending=self.pending, collecting=self.collecting,
                        collections=self.collections,
                        last_collection_ms=self.last_collection_ms,
                        peak_growth_mb=self.peak_growth_bytes/1024**2,
                        last_guard_reason=self.last_guard_reason)


def gc_work(callback):
    """Skip a recurrent tick during idle collection, never block behind it."""
    @wraps(callback)
    def wrapped(self, *args, **kwargs):
        policy = getattr(self, '_task_gc', None)
        if policy is None:
            return callback(self, *args, **kwargs)
        with policy.work() as admitted:
            if admitted:
                return callback(self, *args, **kwargs)
    return wrapped
