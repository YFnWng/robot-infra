"""Opt-in camera-node stall diagnostics: GC pauses, scheduling stalls, fused-stamp lag."""
import gc
import random
import time
from types import SimpleNamespace

import pytest

from perception.marker_tracking import fused_timestamp_values
from perception.stall_diagnostics import GcPauseMonitor, SchedulingProbe


def test_gc_monitor_times_collections_and_resets_window():
    monitor = GcPauseMonitor()
    monitor.start()
    try:
        garbage = [[i] for i in range(200_000)]
        del garbage
        gc.collect()
        first = monitor.window()
        assert first['gc_pauses'] >= 1 and first['gc_pause_max_ms'] > 0.0
        assert first['gc_pause_max_generation'] == 2
        assert monitor.window()['gc_pause_max_ms'] == 0.0  # window reset
    finally:
        monitor.stop()
    count = monitor.window()['gc_pauses']
    gc.collect()
    assert monitor.window()['gc_pauses'] == count  # uninstalled


def test_scheduling_probe_detects_a_gil_hold():
    probe = SchedulingProbe(period_s=0.002, stall_threshold_ms=10.0)
    probe.start()
    try:
        time.sleep(0.05)
        quiet = probe.window()
        values = [random.random() for _ in range(2_000_000)]
        values.sort()  # one C call: holds the GIL for its whole duration
        time.sleep(0.05)
        held = probe.window()
    finally:
        probe.stop()
    assert quiet['probe_wakes'] > 5
    assert held['probe_late_max_ms'] > 50.0
    assert held['probe_stalls'] > quiet['probe_stalls']


def test_probe_rejects_invalid_settings():
    with pytest.raises(ValueError):
        SchedulingProbe(period_s=0.0)


def test_fused_stamp_values_report_lag_under_either_policy():
    fused = SimpleNamespace(weighted_timestamp_ns=1_000_000_000)
    assert fused_timestamp_values(fused, 1_008_250_000, 'newest') == {
        'fused_timestamp_policy': 'newest', 'fused_weighted_stamp_lag_ms': '8.250'}
    legacy = SimpleNamespace()
    assert fused_timestamp_values(legacy, 5, 'weighted_mean')[
        'fused_weighted_stamp_lag_ms'] == 'unknown'


def test_enabled_options_require_a_supporting_wheel_defaults_do_not():
    from perception.marker_tracking import _require_wheel_support

    def old_fuse(estimates, maximum_disagreement_mm=5.0):
        return None

    class OldWorker:
        def __init__(self, rig_id, camera, preview_stride=3, on_frame_ready=None):
            pass

    def new_fuse(estimates, maximum_disagreement_mm=5.0, *, timestamp_policy='newest'):
        return None

    class NewWorker:
        def __init__(self, rig_id, camera, preview_stride=3, on_frame_ready=None,
                     collect_loop_statistics=False):
            pass

    old = {'fuse_marker_estimates': old_fuse, 'CameraCaptureWorker': OldWorker}
    new = {'fuse_marker_estimates': new_fuse, 'CameraCaptureWorker': NewWorker}
    _require_wheel_support(old, 'newest', False)  # default path: unchanged
    for policy, diagnostics, missing in (('weighted_mean', False, 'fused_timestamp_policy'),
                                         ('newest', True, 'stall_diagnostics_enabled')):
        with pytest.raises(ValueError, match=missing):
            _require_wheel_support(old, policy, diagnostics)
        _require_wheel_support(new, policy, diagnostics)
