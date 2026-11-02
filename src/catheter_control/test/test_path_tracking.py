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


def test_missed_xz_station_changes_intent_without_manufacturing_coverage():
    path = ArcLengthPath([[0, 0, 0], [.006, 0, 0], [0, 0, 0]])
    governor = PathProgressGovernor(path, config(
        nominal_speed_m_s=.001, final_tolerance_mm=.8), xz_only=True)
    governor.start(0)
    governor.update([0, 0, .0007], .01)
    for i, x in enumerate(np.linspace(0, .0019, 20)):
        update = governor.update([x, 0, .003], .02+i*.02)
    update = governor.update([.0019, 0, .0005], .43)
    station = governor.coverage.points[governor.coverage.next_station]
    assert update.coverage_fraction == 0
    assert update.state == "RECOVERY_HOLD"
    assert update.reference_point_m == pytest.approx(station)
    assert station[0] < .0019
    points, _ = governor.preview(1., [0, .2, .8])
    assert points == pytest.approx(np.broadcast_to(station, (3, 3)))
    for i in range(1, 11):
        tip = (1-i/10)*np.array([.0019, 0, .0005])+i/10*station
        update = governor.update(tip, .43+i*.02)
    assert update.coverage_fraction > 0
    assert update.reference_point_m[0] > station[0]


def test_external_hold_does_not_supply_missed_station_motion_intent():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    update = governor.update([.004, .003, 0], .1, external_hold=True)
    assert update.state == "TRANSMISSION_HOLD"
    assert update.coverage_fraction == 0
    assert update.reference_point_m == pytest.approx(path.knots[0])


def test_arc_path_interpolates_knots_and_finds_closest_point():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0], [.02, .01, 0]])
    points, tangents = path.evaluate(path.arc_knots)
    assert points == pytest.approx(path.knots)
    assert np.linalg.norm(tangents, axis=1) == pytest.approx(np.ones(3))
    closest = path.closest([.005, .002, 0])
    assert closest.arc_m == pytest.approx(.005, abs=5e-4)
    assert closest.distance_mm == pytest.approx(2.0, abs=1e-4)


def test_progress_governor_holds_recovery_without_clock_advance():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    running = governor.update([0, 0, 0], .1)
    assert running.state == "RUNNING"
    prior = running.progress_m
    recovering = governor.update([0, .005, 0], .2)
    assert recovering.state == "RECOVERY_HOLD"
    assert 0.0 < recovering.speed_scale <= config().recovery_speed_scale
    assert recovering.progress_m == prior
    resumed = governor.update([recovering.progress_m, 0, 0], .3)
    assert resumed.state == "RUNNING"
    assert resumed.progress_m == recovering.progress_m


def test_progress_governor_does_not_force_progress_to_escape_lag():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    governor.progress_m = .004

    update = governor.update([-.000000001, 0, 0], .1)

    assert update.state == "RECOVERY_HOLD"
    assert update.speed_scale > 0.0
    assert update.progress_m == .004


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
    assert resumed.progress_m == held.progress_m


def test_progress_governor_catches_up_to_overshoot_inside_path_tube():
    path = ArcLengthPath([[0, 0, 0], [.1, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)
    for i, x in enumerate(np.linspace(0, .005, 20)):
        governor.update([x, 0, 0], i*.003)

    update = governor.update([.007, .0002, 0], .1)

    assert update.state == "RUNNING"
    assert update.progress_m >= .007-1e-12
    assert update.reference_point_m[0] >= .007
    assert not update.hard_error


def test_progress_governor_does_not_jump_to_coincident_closed_path_end():
    path = ArcLengthPath([[0, 0, 0], [.01, 0, 0], [0, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0.0)

    update = governor.update([0, 0, 0], .1)

    assert update.progress_m == pytest.approx(0)
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
    # Isolate branch selection from the independent ordered-coverage tests.
    governor.progress_m = path.length_m-.0002
    governor.target_arc_m = path.length_m
    governor.coverage.next_station = len(governor.coverage.arcs)
    governor.coverage.covered_m = path.length_m
    update = governor.update([0, 0, 0], .1)

    assert update.state == "FINAL_HOLD"
    assert update.progress_m == pytest.approx(path.length_m)
    assert update.reference_error_mm < 1e-9
    # Geometric reporting remains global and is independent of phase branch.
    assert update.closest.arc_m < .75*path.length_m


def test_stationary_preview_is_spatial_and_clock_invariant():
    path = circle()
    governor = PathProgressGovernor(path, config(nominal_speed_m_s=.001))
    governor.start(0)
    update = governor.update(path.knots[0], 0)
    offsets = np.linspace(-.2, .6, 41)
    points, tangents = governor.preview(1, offsets)
    assert np.linalg.norm(points[0]-path.knots[0]) > 0
    first = ReferenceHorizon(
        path_id="position", sequence=1, source_timestamp_ns=1_000_000_000,
        received_at_s=0, sample_period_s=.02, positions_m=points,
        tangents=tangents, expiry_s=.2, progress_m=0,
        total_length_m=path.length_m, final_hold=False, progress_paused=False)
    update = governor.update(path.knots[0], .033)
    refreshed, refreshed_tangents = governor.preview(1, offsets)
    second = ReferenceHorizon(
        path_id="position", sequence=2, source_timestamp_ns=1_033_000_000,
        received_at_s=.033, sample_period_s=.02, positions_m=refreshed,
        tangents=refreshed_tangents, expiry_s=.2, progress_m=0,
        total_length_m=path.length_m, final_hold=False, progress_paused=False)
    assert update.progress_m == 0
    assert refreshed == pytest.approx(points)
    assert first.compatible_with(second, 1_100_000_000, 4, .04, .05)


def test_position_target_advances_only_after_measured_capture():
    path = ArcLengthPath([[0, 0, 0], [.02, 0, 0]])
    governor = PathProgressGovernor(path, config(nominal_speed_m_s=.001))
    governor.start(0)
    initial = governor.update([0, 0, 0], 0)
    stationary = governor.update([0, 0, 0], .4)
    assert stationary.reference_point_m == pytest.approx(initial.reference_point_m)
    lagging = governor.update([.0002, 0, 0], .45)
    assert lagging.reference_point_m == pytest.approx(initial.reference_point_m)
    captured = governor.update([.0005, 0, 0], .49)
    assert captured.reference_point_m[0] > initial.reference_point_m[0]
    assert captured.progress_m == pytest.approx(.0005)
    held = governor.update([.0006, 0, 0], .5, external_hold=True)
    assert held.progress_m == captured.progress_m
    assert governor.preview(0, [-.2, .6])[0][:, 0] == pytest.approx(
        [held.progress_m, held.progress_m])


def test_gap_or_duplicate_cannot_advance_phase_or_latched_target():
    path = ArcLengthPath([[0, 0, 0], [.02, 0, 0]])
    governor = PathProgressGovernor(path, config(nominal_speed_m_s=.001))
    governor.start(0)
    initial = governor.update([0, 0, 0], 0)
    for now, stamp in [(1, 1), (1.1, 1)]:
        update = governor.update([.002, 0, 0], now, observation_time=stamp)
        assert update.progress_m == 0
        assert update.reference_point_m == pytest.approx(initial.reference_point_m)


def test_progress_governor_only_completes_after_final_settle():
    path = ArcLengthPath([[0, 0, 0], [.001, 0, 0]])
    governor = PathProgressGovernor(
        path, config(nominal_speed_m_s=.01, final_settle_time_s=.2))
    governor.start(0.0)
    governor.update([0, 0, 0], 0.0)
    assert not governor.update([.001, 0, 0], .1).complete
    assert not governor.update([.001, 0, 0], .2).complete
    assert governor.update([.001, 0, 0], .41).complete


def circle():
    angles = np.linspace(0, 2*np.pi, 73)
    return ArcLengthPath(np.column_stack((
        .01*np.cos(angles), .01*np.sin(angles), np.zeros_like(angles))))


def test_stationary_closed_path_endpoint_never_certifies_a_lap():
    path = circle()
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    for now in np.arange(0, 12, .1):
        update = governor.update(path.knots[0], now)
    assert update.coverage_fraction == 0
    assert not update.complete
    assert update.timed_out


def test_insertion_out_and_back_requires_return_leg():
    path = ArcLengthPath([[0, 0, 0], [0, 0, .005], [0, 0, 0]])
    governor = PathProgressGovernor(path, config(total_timeout_s=60))
    governor.start(0)
    # Dense fresh observations of the outbound leg cannot count as a return.
    for i, z in enumerate(np.linspace(0, .005, 101)):
        update = governor.update([0, 0, z], i*.05)
    assert not update.complete
    assert update.coverage_fraction < .75
    for i, z in enumerate(np.linspace(.005, 0, 101)[1:], 101):
        update = governor.update([0, 0, z], i*.05)
    for now in [10.1, 10.5, 11.0]:
        update = governor.update([0, 0, 0], now)
    assert update.coverage_fraction == pytest.approx(1)
    assert update.complete


@pytest.mark.parametrize("anchor", [[.025306638330221176, .02214009314775467,
                                     .06893202662467957], [0, 0, 0], [.2, -.1, .4]])
@pytest.mark.parametrize("noise_mm", [0, .1])
def test_recorded_insertion_settings_enter_endpoint_capture(anchor, noise_mm):
    anchor = np.array(anchor)
    path = ArcLengthPath([anchor, anchor+[0, 0, .005], anchor])
    governor = PathProgressGovernor(path, config(
        nominal_speed_m_s=.001, total_timeout_s=60,
        final_tolerance_mm=1.8, final_settle_time_s=.3,
        pause_error_mm=5, hard_error_mm=15), lookahead_s=.6)
    governor.start(0)
    for i, z in enumerate(np.r_[np.linspace(0, .005, 101),
                                np.linspace(.005, 0, 101)[1:]]):
        update = governor.update(anchor+[noise_mm*.001*np.sin(i), 0, z], i*.05)
        if i == 100:
            assert not update.traversal_complete
            assert not update.complete
        if i == 150:
            assert update.progress_fraction > .5
    for now in [10.1, 10.5, 11]:
        update = governor.update(anchor, now)
    assert update.complete
    assert update.state == "FINAL_HOLD"
    assert update.progress_fraction == 1
    assert update.reference_point_m == pytest.approx(anchor)
    assert update.endpoint_error_mm == 0
    overshoot = governor.update(anchor+[0, 0, -.008], 11.1)
    assert not overshoot.complete
    assert overshoot.state == "FINAL_HOLD"
    assert overshoot.endpoint_error_mm == pytest.approx(8)


def test_stationary_retraced_line_does_not_enter_endpoint_capture():
    anchor = np.array([.025306638330221176, .02214009314775467,
                       .06893202662467957])
    path = ArcLengthPath([anchor, anchor+[0, 0, .005], anchor])
    governor = PathProgressGovernor(path, config(
        nominal_speed_m_s=.001, total_timeout_s=60,
        final_tolerance_mm=1.8), lookahead_s=.6)
    governor.start(0)
    for now in np.arange(0, 10, .05):
        update = governor.update(anchor, now)
    assert not update.complete
    assert not update.traversal_complete
    assert update.coverage_fraction == 0
    assert update.state != "FINAL_HOLD"


def test_scalar_x_traversal_and_settle_ignore_arbitrary_yz_motion():
    anchor = np.array([.025, .022, .069])
    path = ArcLengthPath([anchor, anchor+[.003, 0, 0], anchor])
    governor = PathProgressGovernor(path, config(
        nominal_speed_m_s=.001, total_timeout_s=60,
        final_tolerance_mm=.8, final_settle_time_s=.3), x_only=True)
    governor.start(0)
    for i, x in enumerate(np.r_[np.linspace(0, .003, 61),
                                np.linspace(.003, 0, 61)[1:]]):
        update = governor.update(anchor+[x, .02*np.sin(i), -.03], i*.05)
        if i == 60:
            assert not update.complete
    for now in [6.1, 6.4, 6.7]:
        update = governor.update(anchor+[0, .02, -.03], now)
    assert update.complete
    assert update.endpoint_error_mm == 0
    assert update.closest.distance_mm == 0
    assert not update.hard_error


def test_xz_traversal_ignores_y_but_requires_z_endpoint_settling():
    anchor = np.array([.025, .022, .069])
    path = ArcLengthPath([anchor, anchor+[.006, 0, 0], anchor])
    governor = PathProgressGovernor(path, config(
        nominal_speed_m_s=.001, total_timeout_s=90,
        final_tolerance_mm=.8, final_settle_time_s=.3), xz_only=True)
    governor.start(0)
    for i, x in enumerate(np.r_[np.linspace(0, .006, 121),
                                np.linspace(.006, 0, 121)[1:]]):
        update = governor.update(anchor+[x, .02, 0], i*.05)
    update = governor.update(anchor+[0, .02, .002], 12.1)
    assert not update.complete
    assert update.endpoint_error_mm == pytest.approx(2.)
    for now in [12.2, 12.6, 13.]:
        update = governor.update(anchor+[0, .02, 0], now)
    assert update.complete
    assert update.endpoint_error_mm == 0


def test_yz_motion_alone_cannot_earn_scalar_x_coverage():
    path = ArcLengthPath([[0, 0, 0], [.003, 0, 0], [0, 0, 0]])
    governor = PathProgressGovernor(path, config(total_timeout_s=60), x_only=True)
    governor.start(0)
    for i in range(100):
        update = governor.update([0, i*.001, -.03], i*.05)
    assert update.coverage_fraction == 0
    assert not update.complete


def test_changing_scalar_x_policy_rejects_reference_commit():
    from dataclasses import replace
    reference = ReferenceHorizon(
        path_id="xline", sequence=1, source_timestamp_ns=1_000_000_000,
        received_at_s=1., sample_period_s=.1,
        positions_m=np.tile([.02, .03, .06], (12, 1)),
        tangents=np.tile([1., 0, 0], (12, 1)), expiry_s=1.,
        progress_m=0., total_length_m=.006, final_hold=False,
        progress_paused=False, x_only=True)
    current = replace(reference, sequence=2, x_only=False)
    assert not reference.compatible_with(current, 1_000_000_000, 4, .1, 1.1)
    current = replace(reference, sequence=2, x_only=False, xz_only=True)
    assert not reference.compatible_with(current, 1_000_000_000, 4, .1, 1.1)


def test_lagging_tip_reports_coverage_separately_from_reference():
    path = ArcLengthPath([[0, 0, 0], [.02, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    for now in np.arange(0, 1, .1):
        update = governor.update([.001*now, 0, 0], now)
    assert update.covered_arc_m < update.progress_m
    assert not update.complete


def test_circle_shortcut_and_endpoint_settling_cannot_complete():
    path = circle()
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    # Follow the first quarter, then cut across the circle to its endpoint.
    points, _ = path.evaluate(np.linspace(0, path.length_m/4, 50))
    for i, tip in enumerate(points):
        governor.update(tip, i*.03)
    for i, tip in enumerate(np.linspace(points[-1], path.knots[-1], 50)):
        governor.update(tip, 1.5+i*.03)
    governor.progress_m = path.length_m
    for now in [3.1, 3.3, 3.5]:
        update = governor.update(path.knots[-1], now)
    assert update.state == "FINAL_HOLD"
    assert update.reference_error_mm < 1e-6
    assert update.coverage_fraction < .4
    assert not update.complete


def test_full_circle_requires_observed_traversal_and_endpoint_settle():
    path = circle()
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    points, _ = path.evaluate(np.linspace(0, path.length_m, 300))
    for i, tip in enumerate(points):
        update = governor.update(tip, i*.03)
    assert update.traversal_complete
    assert update.coverage_fraction == 1
    governor.progress_m = path.length_m
    assert not governor.update(path.knots[-1], 9.0).complete
    assert governor.update(path.knots[-1], 9.2).complete


def test_marker_gap_and_duplicate_observations_do_not_credit_motion():
    path = ArcLengthPath([[0, 0, 0], [.02, 0, 0]])
    governor = PathProgressGovernor(path, config())
    governor.start(0)
    governor.update([0, 0, 0], 0, observation_time=0)
    update = governor.update([.02, 0, 0], 1, observation_time=1)
    assert update.covered_arc_m == 0
    assert not update.traversal_complete
    update = governor.update([.02, 0, 0], 1.1, observation_time=1)
    assert update.covered_arc_m == 0


def test_duplicate_endpoint_sample_and_gap_do_not_satisfy_settle():
    path = ArcLengthPath([[0, 0, 0], [.001, 0, 0]])
    governor = PathProgressGovernor(path, config(final_settle_time_s=.2))
    governor.start(0)
    governor.update([0, 0, 0], 0)
    governor.update([.001, 0, 0], .1)
    assert not governor.update([.001, 0, 0], .4, observation_time=.1).complete
    assert not governor.update([.001, 0, 0], 1).complete
    assert governor.update([.001, 0, 0], 1.21).complete


def test_endpoint_departure_resets_settling_and_timeout_cannot_succeed():
    path = ArcLengthPath([[0, 0, 0], [.003, 0, 0]])
    governor = PathProgressGovernor(
        path, config(final_settle_time_s=.2, total_timeout_s=.7))
    governor.start(0)
    governor.update([0, 0, 0], 0)
    governor.update([.003, 0, 0], .3)
    assert not governor.update([.003, .002, 0], .4).complete
    assert not governor.update([.003, 0, 0], .5).complete
    update = governor.update([.003, 0, 0], .71)
    assert update.traversal_complete
    assert update.timed_out
    assert not update.complete


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
