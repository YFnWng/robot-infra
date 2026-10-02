import pytest

from catheter_control.orchestration.causal_schedule import CausalMarkerSchedule


def test_marker_is_deferred_until_estimator_reaches_its_timestamp():
    schedule = CausalMarkerSchedule(rate_hz=20.0, started_s=1.0)

    assert schedule.decision(1.0, 120, None) == "awaiting_estimator"
    assert schedule.decision(1.0, 120, 119) == "awaiting_encoder"
    assert schedule.decision(1.0, 120, 120) == "ready"


def test_committed_marker_is_rate_limited_without_losing_causality():
    schedule = CausalMarkerSchedule(rate_hz=20.0, started_s=2.0)

    assert schedule.decision(2.0, 100, 100) == "ready"
    schedule.commit(2.0)
    assert schedule.decision(2.049, 101, 200) == "rate_limited"
    assert schedule.decision(2.050, 201, 200) == "awaiting_encoder"
    assert schedule.decision(2.050, 200, 200) == "ready"


def test_absolute_phase_avoids_20_hz_to_16_hz_quantization():
    schedule = CausalMarkerSchedule(rate_hz=20.0, started_s=0.0)
    releases = []
    for tick in range(101):
        now = tick*0.02
        if schedule.decision(now, tick, tick) == "ready":
            releases.append(now)
            schedule.commit(now)

    assert len(releases) == 41
    assert releases[-1] == pytest.approx(2.0)
    assert max(b-a for a, b in zip(releases, releases[1:])) <= 0.061


def test_late_commit_skips_missed_releases_without_bursting():
    schedule = CausalMarkerSchedule(rate_hz=20.0, started_s=1.0)

    schedule.commit(1.18)

    assert schedule.next_allowed_s == pytest.approx(1.20)
    assert schedule.decision(1.199, 1, 1) == "rate_limited"
    assert schedule.decision(1.20, 1, 1) == "ready"


@pytest.mark.parametrize("rate", [0.0, -1.0, float("inf")])
def test_schedule_rejects_invalid_rate(rate):
    with pytest.raises(ValueError):
        CausalMarkerSchedule(rate_hz=rate, started_s=0.0)
