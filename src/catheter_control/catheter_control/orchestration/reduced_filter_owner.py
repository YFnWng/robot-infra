"""Reduced-filter estimator ownership for non-commanding planning; ROS-independent.

The cr_meta_lnn stream and rollout backend own all estimator and forecast mathematics. This module only
converts measured encoder counts, keeps the ordered input handoff, maps stream status to controller
health, and builds plan-time roots under one lock so a planner never reads a stream mid-update.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading

import numpy as np

from control_interface_py.encoder_coordinates import counts_to_logical

from ..safety.hardware_contract import ENCODER_RADIANS_PER_COUNT


def encoder_sample(counts):
    """Raw six-axis counts to measured shaft radians and logical tendon mm."""
    counts = np.asarray(counts, float)
    if counts.shape != (6,) or not np.isfinite(counts).all():
        raise ValueError('six finite encoder counts required')
    return counts[:3]*ENCODER_RADIANS_PER_COUNT, float(counts_to_logical(counts[None])[0, 2])


class OrderedInbox:
    """Ordered handoff from subscriptions to the owner; overflow latches and is never silent."""

    def __init__(self, maximum_events=2048):
        self.events = deque()
        self.maximum_events = maximum_events
        self.overflowed = False
        self.lock = threading.Lock()

    def put(self, event):
        with self.lock:
            if len(self.events) >= self.maximum_events:
                self.overflowed = True
                return False
            self.events.append(event)
            return True

    def drain(self):
        with self.lock:
            events = list(self.events)
            self.events.clear()
            return events, self.overflowed


HEALTH = {'tracking': 'TRACKING', 'holding': 'HOLDING', 'fault': 'FAULT'}


@dataclass(frozen=True)
class OwnerHealth:
    estimator_health: str   # INITIALIZING, TRACKING, HOLDING or FAULT (latched)
    ready: bool             # tracking, caught up, kernels prepared
    reason: str | None
    consecutive_rejections: int


@dataclass(frozen=True)
class RootResult:
    root: object | None
    joint_position: np.ndarray | None
    reason: str | None
    seconds: float


class ReducedFilterOwner:
    """Single owner of one ReducedFilterStream and its rollout backend."""

    def __init__(self, stream, backend, *, candidate_counts, step_s, clock):
        counts = sorted({int(count) for count in candidate_counts})
        if not counts or counts[0] < 1 or not np.isfinite(step_s) or step_s <= 0:
            raise ValueError('positive candidate counts and step required')
        self.stream, self.backend = stream, backend
        self.candidate_counts, self.step_s, self.clock = counts, float(step_s), clock
        self.lock = threading.Lock()
        self.prepared_law_counts = set()
        self.preparations = []   # (law count, seconds) per kernel preparation
        self.latest_counts = None
        self.last_result = None
        self.faulted = False

    def add_encoder(self, stamp_ns, counts):
        shaft, tendon = encoder_sample(counts)
        with self.lock:
            self.stream.aligner.add_encoder(stamp_ns, shaft, tendon)
            self.latest_counts = np.asarray(counts, float).copy()

    def add_image(self, stamp_ns, points_mm):
        with self.lock:
            return self.stream.aligner.add_image(stamp_ns, points_mm)

    def mark_history_lost(self):
        with self.lock:
            self.stream.aligner.history_lost = True

    def process(self):
        """Advance at most one image, as the research observer does."""
        with self.lock:
            result = self.stream.process()
            if result is None:
                return None
            self.last_result = result
            self.faulted = self.faulted or result.status == 'fault'
            # Kernel shapes depend on the active law count. At the first preparation (inside the start-up
            # queue allowance) every count up to the estimator's maximum is prepared, the others from a
            # shape-only example root, so no preparation runs inside a later step. A count not prepared
            # yet (unknown maximum) is still prepared here, after a step and never during a solve.
            if result.status in ('initialized', 'tracking'):
                laws = self._law_count()
                if laws not in self.prepared_law_counts:
                    committed_ns = int(self.stream.aligner.committed[0])
                    maximum = int(self.stream.estimator.options.get('maximum_laws', laws))
                    for count in [laws] + [k for k in range(1, maximum+1) if k != laws]:
                        if count in self.prepared_law_counts:
                            continue
                        started = self.clock()
                        root = (self.backend.root(self.stream, committed_ns) if count == laws
                                else self.backend.example_root(self.stream, committed_ns, count))
                        self.backend.prepare(root, self.candidate_counts, self.step_s)
                        self.prepared_law_counts.add(count)
                        self.preparations.append((count, self.clock()-started))
            return result

    def _law_count(self):
        return int(len(self.stream.estimator.laws.means))

    @property
    def prepared(self):
        return (self.stream.estimator is not None
                and self._law_count() in self.prepared_law_counts)

    def health(self):
        with self.lock:
            result = self.last_result
            if self.faulted:
                return OwnerHealth('FAULT', False, None if result is None else result.reason,
                                   0 if result is None else result.consecutive_rejections)
            if result is None:
                return OwnerHealth('INITIALIZING', False, None, 0)
            health = HEALTH.get(result.status, 'INITIALIZING')
            ready = (health == 'TRACKING' and self.prepared and not self.stream.aligner.pending)
            return OwnerHealth(health, ready, result.reason, result.consecutive_rejections)

    def plan_root(self):
        """Root at the newest measured encoder sample; refused (never faulted) when unavailable."""
        started = self.clock()
        with self.lock:
            history = self.stream.aligner.history
            if not self.prepared or not history or self.latest_counts is None:
                return RootResult(None, None, 'not_ready', self.clock()-started)
            plan_ns = int(history[-1][0])
            counts = self.latest_counts.copy()
            try:
                root = self.backend.root(self.stream, plan_ns)
            except ValueError as error:
                return RootResult(None, None, f'root_refused: {error}', self.clock()-started)
            if not self.backend.prepared_for(root, self.candidate_counts):
                return RootResult(None, None, 'kernel_not_prepared', self.clock()-started)
        return RootResult(root, counts_to_logical(counts[None])[0], None, self.clock()-started)
