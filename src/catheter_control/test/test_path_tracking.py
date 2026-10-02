import numpy as np
import pytest

from catheter_control.planning.path_tracking import (
    ArcLengthPath, GovernorConfig, PathProgressGovernor, ReferenceHorizon)


def config(**changes):
    values = dict(
        nominal_speed_m_s=.01, total_timeout_s=10.0,
        final_tolerance_mm=1.0, final_settle_time_s=.1,
        soft_error_mm=2.0, pause_error_mm=4.0,
        resume_error_mm=1.5, hard_error_mm=8.0)
    values.update(changes)
    return GovernorConfig(**values)


def test_arc_path_interpolates_knots_and_finds_closest_point():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0], [.02, .01, 0]])
    points, tangents = path.evaluate(path.arc_knots)
    assert points == pytest.approx(path.knots)
    assert np.linalg.norm(tangents, axis=1) == pytest.approx(np.ones(3))
    closest = path.closest([.005, .002, 0])
    assert closest.arc_m == pytest.approx(.005, abs=5e-4)
    assert closest.distance_mm == pytest.approx(2.0, abs=1e-4)


def test_progress_governor_uses_bounded_recovery_without_rewinding():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    running = governor.update([0, 0, 0], .1)
    assert running.state == "RUNNING"
    prior = running.progress_m
    recovering = governor.update([0, .005, 0], .2)
    assert recovering.state == "RECOVERY_ADVANCE"
    assert 0.0 < recovering.speed_scale <= config().recovery_speed_scale
    assert recovering.progress_m > prior
    resumed = governor.update([recovering.progress_m, 0, 0], .3)
    assert resumed.state == "RUNNING"
    assert resumed.progress_m > recovering.progress_m


def test_progress_governor_does_not_asymptotically_freeze_before_hard_error():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    governor.progress_m = .004

    update = governor.update([-.000000001, 0, 0], .1)

    assert update.state == "RECOVERY_ADVANCE"
    assert update.speed_scale > 0.0


def test_progress_governor_freezes_for_external_transmission_hold():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    running = governor.update([0, 0, 0], .1)

    held = governor.update(
        [running.progress_m, 0, 0], .6, external_hold=True)
    resumed = governor.update([running.progress_m, 0, 0], .7)

    assert held.state == "TRANSMISSION_HOLD"
    assert held.speed_scale == 0.0
    assert held.progress_m == pytest.approx(running.progress_m)
    assert resumed.progress_m > held.progress_m


def test_progress_governor_catches_up_to_overshoot_inside_path_tube():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    governor.progress_m = .005

    update = governor.update([.007, .0002, 0], .1)

    assert update.state == "RUNNING"
    assert update.progress_m >= .007
    assert update.reference_point_m[0] >= .007
    assert not update.hard_error


def test_progress_governor_does_not_jump_to_coincident_closed_path_end():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0], [0, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)

    update = governor.update([0, 0, 0], .1)

    assert update.progress_m == pytest.approx(.001)
    assert update.progress_m < .5*path.length_m


def test_progress_governor_does_not_jump_back_at_closed_path_end():
    # The path first reaches [0,0,0] at arc 20 mm and closes there again at
    # arc 40 mm. Global geometric projection deliberately chooses the first
    # coincident branch, while phase control must stay near the endpoint.
    path = ArcLengthPath([
        [-.02, 0, 0], [0, 0, 0], [.01, 0, 0], [0, 0, 0]])
    assert path.closest([0, 0, 0]).arc_m < .75*path.length_m
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    governor.progress_m = path.length_m-.0002

    update = governor.update([0, 0, 0], .1)

    assert update.state == "FINAL_HOLD"
    assert update.progress_m == pytest.approx(path.length_m)
    assert update.reference_error_mm < 1e-9
    # Geometric reporting remains global and is independent of phase branch.
    assert update.closest.arc_m < .75*path.length_m


def test_progress_governor_only_completes_after_final_settle():
    path = ArcLengthPath([[0, 0, 0], [.001, 0, 0]])
    governor = PathProgressGovernor(
        path, config(nominal_speed_m_s=.01, final_settle_time_s=.2))
    governor.start(0.0)
    assert not governor.update([.001, 0, 0], .1).complete
    assert not governor.update([.001, 0, 0], .2).complete
    assert governor.update([.001, 0, 0], .41).complete


def test_reference_horizon_resamples_at_estimator_time():
    reference = ReferenceHorizon(
        path_id="test", sequence=2, source_timestamp_ns=10_000_000_000,
        received_at_s=5.0, sample_period_s=.1,
        positions_m=np.column_stack((np.arange(6)*.001, np.zeros((6, 2)))),
        tangents=np.tile([1.0, 0.0, 0.0], (6, 1)),
        expiry_s=.2, progress_m=0.0, total_length_m=.01,
        final_hold=False, progress_paused=False)
    targets = reference.targets(
        10_000_000_000, horizon_steps=2, rollout_step_s=.15, now_s=5.1)
    assert targets[:, 0] == pytest.approx([.0015, .003])
    sampled_targets, sampled_tangents = reference.sample(
        10_000_000_000, horizon_steps=2,
        rollout_step_s=.15, now_s=5.1)
    np.testing.assert_allclose(sampled_targets, targets)
    np.testing.assert_allclose(sampled_tangents, [[1, 0, 0], [1, 0, 0]])
    with pytest.raises(ValueError, match="stale"):
        reference.targets(
            10_000_000_000, horizon_steps=2,
            rollout_step_s=.15, now_s=5.3)


@pytest.mark.parametrize("knots", [
    [], [[0, 0, 0]], [[0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [np.nan, 0, 0]],
])
def test_arc_path_rejects_invalid_knots(knots):
    with pytest.raises(ValueError):
        ArcLengthPath(knots)
