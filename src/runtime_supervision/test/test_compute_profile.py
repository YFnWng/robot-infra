from types import SimpleNamespace

import numpy as np
import pytest

from runtime_supervision.compute_profile import (
    Measurements, ReplayEvent, replay, ReplayGcExperiment)


def test_stationary_gc_experiment_restores_policy_and_bounds_memory(monkeypatch):
    import gc
    original = gc.isenabled(), gc.get_threshold(), list(gc.callbacks)
    monkeypatch.setattr(gc, 'collect', lambda: 0)
    rss = [100_000_000]
    monkeypatch.setattr(ReplayGcExperiment, 'rss_bytes', staticmethod(lambda: rss[0]))
    experiment = ReplayGcExperiment('live-stationary',120.,1.,42,False)
    try:
        experiment.start()
        assert not gc.isenabled()
        rss[0] += 2*1024**2
        experiment.next_sample_s = 0.
        assert experiment.check() == 'gc_memory_growth_limit'
    finally:
        report = experiment.stop()
    assert report['policy'] == 'live-stationary'
    assert (gc.isenabled(),gc.get_threshold(),list(gc.callbacks)) == original


@pytest.mark.parametrize('domain,simulated,duration', [(214,False,60),(42,True,60),(42,False,241)])
def test_stationary_gc_experiment_rejects_wrong_scope(domain,simulated,duration):
    with pytest.raises(ValueError,match='stationary observer'):
        ReplayGcExperiment('live-stationary',duration,256.,domain,simulated)


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


def test_unmeasured_thread_cpu_is_null_not_zero_or_a_report_failure():
    meter=Measurements()
    meter.rows.append(dict(stage='age',source_ns=1,wall_ms=2.,thread_cpu_ms=None))
    result=meter.summaries()['age']
    assert result['count']==1 and result['thread_cpu'] is None
    assert result['wall']['p50_ms']==2.


def test_optional_event_observer_preserves_causal_order():
    runtime = Runtime()
    rows = []
    events = [ReplayEvent(100, 100, "encoder", np.zeros(6)),
              ReplayEvent(110, 100, "marker", np.zeros((4, 3)), {})]
    replay(runtime, None, None, events, [], start_ns=0, end_ns=200,
           measured=Measurements(), on_estimator_event=lambda stage, event, result:
           rows.append((stage, event.source_ns, result is not None)))
    assert rows == [("encoder", 100, False), ("before_marker", 100, False), ("marker", 100, True)]
    assert runtime.counts == [100] and runtime.observations == [100]


def test_receipt_hook_runs_before_owner_mutation_including_prefix():
    runtime = Runtime()
    rows = []
    events = [ReplayEvent(100, 100, "encoder", np.zeros(6)),
              ReplayEvent(130, 130, "encoder", np.zeros(6))]
    replay(runtime, None, None, events, [], start_ns=120, end_ns=140,
           measured=Measurements(), encoder_period_ns=20,
           before_event=lambda event: rows.append((event.receipt_ns, len(runtime.counts))))
    assert rows == [(100, 0), (130, 1)]


def test_optional_plan_owner_receives_causal_snapshot_and_window_meter(monkeypatch):
    from runtime_supervision import compute_profile as owner
    monkeypatch.setattr(owner,'CatheterMppi',lambda *a:None)
    runtime=Runtime();meter=Measurements();seen=[]
    def solve(root,position,target,event,measurements):
        seen.append((root.timestamp_ns,event.receipt_ns,measurements is meter))
        return SimpleNamespace(valid=True,reason='ok',elapsed_s=.001,
            sample_projection_ms=0.,rollout_ms=0.,cost_weighting_ms=0.,update_projection_ms=0.)
    events=[ReplayEvent(100,100,'encoder',np.zeros(6)),
            ReplayEvent(101,100,'position',np.zeros(6)),
            ReplayEvent(110,100,'plan',None),ReplayEvent(130,100,'plan',None)]
    plans,_=replay(runtime,None,None,events,[(100,np.zeros(3))],
        start_ns=120,end_ns=140,measured=meter,plan_function=solve)
    assert seen==[(100,110,False),(100,130,True)] and len(plans)==1
