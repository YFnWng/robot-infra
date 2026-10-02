from types import SimpleNamespace

import numpy as np
import pytest

from runtime_supervision.compute_profile import (
    Measurements, ReplayEvent, replay)


def test_console_routes_to_canonical_offline_owner(monkeypatch):
    from catheter_control import bootstrap as controller_bootstrap
    from runtime_supervision import bootstrap

    modules = []
    monkeypatch.setattr(controller_bootstrap, "run_in_venv", modules.append)
    bootstrap.compute_profile()
    assert modules == ["runtime_supervision.compute_profile"]


class Runtime:
    def __init__(self):
        self.state = None
        self.counts = []
        self.observations = []
        self._rewind = []
        self.maximum_step_s = .04
        self.last_marker_timing_ms = {"rewind": 1., "correction": 2., "replay": 3.}

    def initialize(self, stamp, counts):
        return self.advance_encoder(stamp, counts)

    def advance_encoder(self, stamp, counts):
        self.state = SimpleNamespace(timestamp_ns=stamp)
        self._rewind.append(self.state)
        self.counts.append(stamp)
        return self.state

    def diagnostics(self):
        return {}

    def clone_state(self):
        return SimpleNamespace(timestamp_ns=self.state.timestamp_ns)

    def observe_markers(self, stamp, points, quality):
        assert stamp <= self.state.timestamp_ns
        self.observations.append(stamp)
        return SimpleNamespace(accepted=True, reason="accepted")


def test_prefix_history_causal_deferral_and_measurement_window():
    runtime = Runtime()
    events = [ReplayEvent(100, 100, "encoder", np.zeros(6)),
              ReplayEvent(110, 125, "marker", np.zeros((4, 3)), {}),
              ReplayEvent(130, 130, "encoder", np.zeros(6)),
              ReplayEvent(140, 140, "encoder", np.zeros(6)),
              ReplayEvent(200, 200, "encoder", np.zeros(6))]
    meter = Measurements()
    starts = []
    plans, corrections = replay(runtime, None, None, events, [],
                                start_ns=120, end_ns=150, measured=meter,
                                encoder_period_ns=20, marker_period_ns=50,
                                on_window_start=lambda: starts.append(len(runtime.counts)))
    assert runtime.counts == [100, 130]
    assert runtime.observations == [125]
    assert not plans
    assert corrections[0]["replay_entries"] == 1
    assert corrections[0]["replay_substeps"] == 1
    assert all(row["source_ns"] >= 120 for row in meter.rows)
    assert meter.summaries()["encoder_propagation"]["count"] == 1
    assert starts == [1]


def test_latest_marker_replacement():
    runtime = Runtime()
    events = [ReplayEvent(100, 100, "encoder", np.zeros(6)),
              ReplayEvent(110, 125, "marker", np.zeros((4, 3)), {}),
              ReplayEvent(120, 128, "marker", np.zeros((4, 3)), {}),
              ReplayEvent(130, 130, "encoder", np.zeros(6))]
    replay(runtime, None, None, events, [], start_ns=0, end_ns=200,
           measured=Measurements(), encoder_period_ns=20)
    assert runtime.observations == [128]


def test_measurement_exception_is_not_swallowed():
    def fail():
        raise RuntimeError("test")
    meter = Measurements()
    with pytest.raises(RuntimeError, match="test"):
        meter.call("failure", 1, fail)
    assert not meter.rows
