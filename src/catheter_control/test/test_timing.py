import pytest

from catheter_control.orchestration.timing import PeriodicTimerProbe, TimingWindows


def test_timing_windows_report_and_reset():
    timing = TimingWindows()
    timing.record_seconds("runtime_lock_wait", 0.001)
    timing.record_seconds("runtime_lock_wait", 0.003)

    result = timing.snapshot_and_reset()

    assert result["timing_runtime_lock_wait_window_count"] == 2
    assert result[
        "timing_runtime_lock_wait_window_mean_ms"] == pytest.approx(2)
    assert result["timing_runtime_lock_wait_window_p50_ms"] == pytest.approx(2)
    assert result["timing_runtime_lock_wait_window_p95_ms"] == pytest.approx(2.9)
    assert result["timing_runtime_lock_wait_window_p99_ms"] == pytest.approx(2.98)
    assert result["timing_runtime_lock_wait_window_max_ms"] == pytest.approx(3)
    assert timing.latest()["runtime_lock_wait"] == pytest.approx(3)
    assert timing.snapshot_and_reset() == {}


def test_periodic_timer_probe_preserves_phase_and_skips_missed_releases():
    probe = PeriodicTimerProbe(period_s=0.02, started_s=10.0)

    assert probe.observe(10.021) == pytest.approx(0.001)
    assert probe.observe(10.040) == pytest.approx(0.0)
    assert probe.observe(10.105) == pytest.approx(0.045)
    assert probe.observe(10.120) == pytest.approx(0.0)


@pytest.mark.parametrize("period", [0.0, -0.1, float("inf")])
def test_periodic_timer_probe_rejects_invalid_period(period):
    with pytest.raises(ValueError):
        PeriodicTimerProbe(period_s=period, started_s=0.0)
