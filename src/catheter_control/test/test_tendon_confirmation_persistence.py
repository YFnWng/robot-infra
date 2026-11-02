"""Raw-geometry confirmation under held actuation; no ROS or hardware I/O."""
import numpy as np
import pytest

from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator


def session(**options):
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3,
        **options))
    motor = np.zeros(3)
    markers = np.array([[0., 0., 0.], [0., 0., .01],
                        [0., 0., .02], [0., 0., .03]])
    stamp = [0]
    def observe(deformation_mm, *, new_motor=None, duplicate=False):
        if new_motor is not None:
            motor[2] = new_motor
        owner.advance_motor(motor)
        points = markers.copy()
        # Three of the six pair distances change: scale to the RMS descriptor.
        points[-1, 2] += np.sqrt(2.)*deformation_mm/1000.
        if not duplicate:
            stamp[0] += 50_000_000
        return owner.observe_response(motor, np.eye(4), np.eye(6, 3), np.ones(6),
            timestamp_ns=stamp[0], observed_marker_points_m=points)
    for _ in range(5):
        observe(0.)
    return owner, observe


def onset(observe):
    # Establish the source-time reference for the new motor direction before
    # introducing deformation; pre-reversal movement is not onset evidence.
    observe(0., new_motor=.1)
    observe(.35, new_motor=.4)
    state = observe(.35)
    assert state.phase[2] == "PROVISIONAL"
    assert state.confirmation_count[2] == 1
    return state


def test_credible_onset_persists_through_small_threshold_fluctuation():
    owner, observe = session()
    first = onset(observe)
    assert observe(.28).confirmation_count[2] == 2
    final = observe(.28)
    assert final.phase[2] == "ENGAGED"
    assert final.confirmation_count[2] == 3
    np.testing.assert_array_equal(owner.previous_raw_motor, [0., 0., .4])
    assert final.engagement_anchor_motor_rad[2] == first.engagement_anchor_motor_rad[2]


@pytest.mark.parametrize("ratio, expected", [(1., "PROVISIONAL"), (.8, "PROVISIONAL"),
                                           (.75, "ENGAGED")])
def test_recorded_first_hold_dip_requires_sufficient_hysteresis(ratio, expected):
    # Diagnostic sample at ROS 1791415814.071131, reanchored repeat02:
    # raw descriptor RMS = 0.775332 * unchanged 0.300 mm onset scale.
    # Replay a matching policy input, not a counterfactual closed-loop run.
    _, observe = session(distal_confirmation_release_ratio=ratio)
    onset(observe)
    dip_mm = .775332*.3
    assert observe(dip_mm).confirmation_count[2] == 2
    assert observe(dip_mm).phase[2] == expected


def test_hysteresis_cannot_initiate_engagement_below_onset_floor():
    _, observe = session()
    for _ in range(10):
        state = observe(.28, new_motor=.4)
        assert state.phase[2] == "TAKEUP"
        assert state.confirmation_count[2] == 0


def test_lost_deformation_resets_support_and_requires_full_onset_again():
    _, observe = session()
    onset(observe)
    for _ in range(3):
        state = observe(.1)
    assert state.phase[2] == "PROVISIONAL"
    assert state.confirmation_count[2] == 0
    for _ in range(5):
        state = observe(.28)
        assert state.phase[2] == "PROVISIONAL"
        assert state.confirmation_count[2] == 0
    for _ in range(4):
        state = observe(.35)
    assert state.phase[2] == "ENGAGED"


def test_duplicate_visual_evidence_cannot_complete_confirmation():
    _, observe = session()
    onset(observe)
    for _ in range(5):
        state = observe(.28, duplicate=True)
        assert state.phase[2] == "PROVISIONAL"
        assert state.confirmation_count[2] == 1


def test_checkpoint_preserves_hysteresis_without_changing_travel_accounting():
    owner, observe = session()
    onset(observe)
    checkpoint = owner.clone_state()
    observe(.28)
    assert observe(.28).phase[2] == "ENGAGED"
    owner.restore_state(checkpoint)
    restored = owner.snapshot()
    assert restored.phase[2] == "PROVISIONAL"
    assert restored.confirmation_count[2] == 1
    travel = owner.accumulated_since_reversal.copy()
    observe(.28)
    assert observe(.28).phase[2] == "ENGAGED"
    np.testing.assert_array_equal(owner.accumulated_since_reversal, travel)


def test_new_pipeline_confirmation_view_matches_shared_owner_in_hysteresis_band():
    from types import SimpleNamespace
    from catheter_control.planning.engagement import (
        confirmation_from_checkpoint, tendon_confirmation)
    owner, observe = session()
    onset(observe)
    runtime = SimpleNamespace(tendon_mm_per_shaft_rad=-.18949385411878789)
    for _ in range(2):
        snapshot = observe(.28)
        view = tendon_confirmation(runtime, snapshot)
        checkpoint_view = confirmation_from_checkpoint(runtime, owner.clone_state())
        assert view == checkpoint_view
        assert view.phase == snapshot.phase[2]
        assert view.count == snapshot.confirmation_count[2]
        assert view.direction == -snapshot.motion_direction[2]


def test_hysteresis_disabled_retains_strict_floor():
    _, observe = session(distal_confirmation_release_ratio=1.)
    onset(observe)
    observe(.28)
    state = observe(.28)
    assert state.phase[2] == "PROVISIONAL"
    assert state.confirmation_count[2] == 0


@pytest.mark.parametrize("ratio", [0., -.1, 1.1, float("nan"), float("inf")])
def test_release_ratio_is_validated(ratio):
    with pytest.raises(ValueError, match="release_ratio"):
        BacklashConfig(width_rad=(1., 1., 1.), distal_confirmation_release_ratio=ratio)
