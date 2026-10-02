from pathlib import Path

import numpy as np
import pytest
import yaml

from catheter_control.transmission.backlash import (
    BacklashConfig, BacklashFeedforwardCompensator, BacklashSnapshot,
    BacklashStateEstimator, TakeupTransactionArbiter,
    rollout_backlash_state, rollout_raw_interface_play)
from catheter_control.safety.hardware_contract import load_hardware_contract


ROOT = Path(__file__).resolve().parents[2]
LIMITS = ROOT / "control_interface" / "config" / "catheter_limits.yaml"


def _pose(x=0.0, y=0.0, z=0.0):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


def _jacobian():
    result = np.zeros((6, 3))
    result[3:, :] = np.eye(3)
    return result


def test_reversal_consumes_takeup_and_interface_response_confirms_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 2.0, 3.0), minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        width_learning_rate=.1))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)
    forward = estimator.observe([1.1, 0, 0], _pose(.1), _jacobian(), scale)
    assert forward.phase[0] == "ENGAGED"
    assert forward.engaged_direction[0] == 1

    reversal = estimator.observe([.5, 0, 0], _pose(.1), _jacobian(), scale)
    assert reversal.phase[0] == "TAKEUP"
    assert reversal.remaining_rad[0] == pytest.approx(.4)

    engaged = estimator.observe([-.1, 0, 0], _pose(-.2), _jacobian(), scale)
    assert engaged.phase[0] == "ENGAGED"
    assert engaged.engaged_direction[0] == -1
    assert engaged.remaining_rad[0] == 0.0
    assert .5 <= engaged.width_rad[0] <= 1.5


def test_static_motion_does_not_confirm_or_learn_width():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0), directional_purity=.9))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    before = estimator.snapshot()
    static = estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    np.testing.assert_array_equal(static.width_rad, before.width_rad)
    assert static.engaged_direction.tolist() == [0, 0, 0]


def test_joint_observer_confirms_multiple_transmitting_axes():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    mixed = estimator.observe(
        [.4, -.3, 0], _pose(.4, -.3), _jacobian(), np.ones(6))

    assert mixed.phase[:2] == ("ENGAGED", "ENGAGED")
    assert mixed.engaged_direction.tolist() == [1, -1, 0]
    np.testing.assert_allclose(
        mixed.inferred_transmitted_increment_rad, [.4, -.3, 0], atol=.01)
    assert np.all(mixed.response_evidence[:2] > .9)


def test_response_observer_accumulates_subthreshold_camera_increments():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    first = estimator.observe([.02, 0, 0], _pose(.02), _jacobian(),
                              np.ones(6))
    second = estimator.observe([.04, 0, 0], _pose(.04), _jacobian(),
                               np.ones(6))
    confirmed = estimator.observe([.06, 0, 0], _pose(.06), _jacobian(),
                                  np.ones(6))

    assert first.phase[0] == "TAKEUP"
    assert second.phase[0] == "TAKEUP"
    assert confirmed.phase[0] == "ENGAGED"
    assert confirmed.inferred_transmitted_increment_rad[0] == pytest.approx(
        .06, abs=.01)


def test_stopped_subthreshold_shaft_does_not_block_ready_shaft():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.10))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    # Axis 1 moved enough to enter the response window, but stopped just
    # below its own inference floor. Axis 2 then develops a fully observable
    # response. The inactive residue must not hold the entire joint window.
    estimator.observe([0, .09, -.04], _pose(z=-.04), _jacobian(), np.ones(6))
    ready = estimator.observe(
        [0, .09, -.14], _pose(z=-.14), _jacobian(), np.ones(6))

    assert ready.phase[1] == "TAKEUP"
    assert ready.phase[2] == "ENGAGED"
    assert ready.inferred_transmitted_increment_rad[1] == 0.0
    assert ready.inferred_transmitted_increment_rad[2] == pytest.approx(
        -.14, abs=.01)


def test_distal_bending_is_primary_tendon_engagement_evidence():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.10,
        minimum_distal_bending_increment=.05))
    mode = np.array([1.0, 0.0, 0.0])
    strain = np.zeros(3)
    estimator.advance_motor([0, 0, 0])
    estimator.observe_response(
        [0, 0, 0], _pose(), _jacobian(), np.ones(6), strain, mode)

    # No interface-body motion is present. The marker-corrected distal state
    # nevertheless changes strongly along the learned bending mode.
    # The distal observation is allowed to supersede the generic 0.10-rad
    # interface-fit readiness floor.
    estimator.advance_motor([0, 0, -.04])
    engaged = estimator.observe_response(
        [0, 0, -.04], _pose(), _jacobian(), np.ones(6),
        np.array([.08, 0.0, 0.0]), mode)

    assert engaged.phase[2] == "ENGAGED"
    assert engaged.engaged_direction[2] == -1
    assert engaged.tendon_distal_response_confirmed
    assert engaged.distal_bending_increment == pytest.approx(.08)
    assert engaged.tendon_distal_response_evidence == pytest.approx(1.0)
    assert engaged.inferred_transmitted_increment_rad[2] == 0.0
    # Width learning remains tied to metrically attributed interface motion;
    # distal-only confirmation changes engagement, not the width prior.
    assert engaged.width_rad[2] == pytest.approx(1.0)


def test_distal_bending_noise_below_floor_does_not_confirm_tendon():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.10,
        minimum_distal_bending_increment=.05))
    mode = np.array([1.0, 0.0, 0.0])
    estimator.advance_motor([0, 0, 0])
    estimator.observe_response(
        [0, 0, 0], _pose(), _jacobian(), np.ones(6), np.zeros(3), mode)
    estimator.advance_motor([0, 0, -.2])
    noise = estimator.observe_response(
        [0, 0, -.2], _pose(), _jacobian(), np.ones(6),
        np.array([.02, 0.0, 0.0]), mode)

    assert noise.phase[2] == "TAKEUP"
    assert not noise.tendon_distal_response_confirmed
    assert noise.tendon_distal_response_evidence == pytest.approx(.4)


def test_interface_play_does_not_use_distal_tendon_response_as_confirmation():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.10,
        minimum_distal_bending_increment=.05,
        distal_confirmation_enabled=False))
    mode = np.array([1.0, 0.0, 0.0])
    estimator.advance_motor([0, 0, 0])
    estimator.observe_response(
        [0, 0, 0], _pose(), _jacobian(), np.ones(6), np.zeros(3), mode)
    estimator.advance_motor([0, 0, -.2])
    state = estimator.observe_response(
        [0, 0, -.2], _pose(), _jacobian(), np.ones(6),
        np.array([.2, 0.0, 0.0]), mode)

    assert state.phase[2] == "TAKEUP"
    assert not state.tendon_distal_response_confirmed


def test_raw_interface_play_filters_only_selected_interface_coordinate():
    state = BacklashSnapshot(
        width_rad=np.array([1., 1., 1.]),
        width_positive_rad=np.array([1., 1., 1.]),
        width_negative_rad=np.array([1., 1., 1.]),
        remaining_rad=np.array([0., 0., 1.]),
        motion_direction=np.array([1, 1, 1], dtype=np.int8),
        engaged_direction=np.array([1, 1, 0], dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "TAKEUP"))
    raw = np.zeros((1, 2, 3))
    raw[0, :, :] = [2.0, -3.0, 4.0]

    result = rollout_raw_interface_play(
        raw, state, .1, filtered_axes=(False, False, True))

    np.testing.assert_allclose(
        result.interface_motor_radians_per_second[0, :, :2],
        raw[0, :, :2])
    np.testing.assert_allclose(
        result.interface_motor_radians_per_second[0, :, 2], 0.0)
    assert result.remaining_rad[0, 2] == pytest.approx(.2)


def test_response_accumulation_resets_at_shaft_reversal():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    estimator.observe([.04, 0, 0], _pose(.04), _jacobian(), np.ones(6))

    reversal = estimator.observe(
        [.02, 0, 0], _pose(.02), _jacobian(), np.ones(6))
    after = estimator.observe(
        [-.01, 0, 0], _pose(-.01), _jacobian(), np.ones(6))

    assert reversal.phase[0] == "TAKEUP"
    assert after.phase[0] == "TAKEUP"
    assert after.inferred_transmitted_increment_rad[0] == 0.0


def test_joint_observer_only_confirms_axis_with_observed_response():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    mixed = estimator.observe(
        [.4, .4, 0], _pose(.4, 0), _jacobian(), np.ones(6))

    assert mixed.phase[:2] == ("ENGAGED", "TAKEUP")
    assert mixed.engaged_direction.tolist() == [1, 0, 0]
    assert mixed.inferred_transmitted_increment_rad[0] == pytest.approx(
        .4, abs=.01)
    assert abs(mixed.inferred_transmitted_increment_rad[1]) < .05


def test_joint_observer_does_not_confirm_ambiguous_collinear_axes():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        minimum_response_evidence=.5))
    jacobian = _jacobian()
    jacobian[:, 1] = jacobian[:, 0]
    estimator.observe([0, 0, 0], _pose(), jacobian, np.ones(6))

    ambiguous = estimator.observe(
        [.4, .4, 0], _pose(.4), jacobian, np.ones(6))

    assert ambiguous.phase[:2] == ("TAKEUP", "TAKEUP")
    assert np.all(ambiguous.response_evidence[:2] < .5)


def test_unconfirmed_takeup_is_bounded_by_prior_width():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    taking_up = estimator.observe(
        [.4, 0, 0], _pose(), _jacobian(), np.ones(6))
    exhausted = estimator.observe(
        [1.1, 0, 0], _pose(), _jacobian(), np.ones(6))
    assert taking_up.remaining_rad[0] == pytest.approx(.6)
    assert exhausted.remaining_rad[0] == 0.0
    assert exhausted.engaged_direction[0] == 0


def test_takeup_requires_repeated_measured_response_confirmation():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    first = estimator.observe([.4, 0, 0], _pose(.1), _jacobian(), scale)
    second = estimator.observe([.6, 0, 0], _pose(.2), _jacobian(), scale)
    confirmed = estimator.observe([.8, 0, 0], _pose(.3), _jacobian(), scale)

    assert first.phase[0] == "PROVISIONAL"
    # The unused bounded gap is retained for safe fallback, but PROVISIONAL
    # gives normal control authority and prevents more full-rate take-up.
    assert first.remaining_rad[0] == pytest.approx(.6)
    assert first.engaged_direction[0] == 1
    assert second.phase[0] == "PROVISIONAL"
    assert confirmed.phase[0] == "ENGAGED"
    assert confirmed.confirmation_count[0] == 3
    assert confirmed.engaged_direction[0] == 1


def test_subthreshold_interface_noise_does_not_trigger_provisional_handoff():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    noisy = estimator.observe(
        [.4, 0, 0], _pose(.01), _jacobian(), scale)

    assert noisy.phase[0] == "TAKEUP"
    assert noisy.engaged_direction[0] == 0
    assert noisy.confirmation_count[0] == 0


def test_provisional_engagement_survives_inconclusive_motion():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3,
        provisional_rejection_observations=2))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    provisional = estimator.observe(
        [.4, 0, 0], _pose(.1), _jacobian(), scale)
    first_miss = estimator.observe(
        [.6, 0, 0], _pose(.1), _jacobian(), scale)
    still_provisional = estimator.observe(
        [.8, 0, 0], _pose(.1), _jacobian(), scale)

    assert provisional.phase[0] == "PROVISIONAL"
    assert first_miss.phase[0] == "PROVISIONAL"
    assert still_provisional.phase[0] == "PROVISIONAL"
    assert still_provisional.engaged_direction[0] == 1
    assert still_provisional.confirmation_count[0] == 1
    assert still_provisional.response_classification[0] == "INCONCLUSIVE"
    assert still_provisional.provisional_rejection_count[0] == 0


def test_provisional_handoff_is_not_revoked_by_inconclusive_samples():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        maximum_width_gain=1.5,
        engagement_confirmation_observations=3,
        provisional_rejection_observations=2))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    # The attributed response is deliberately observed after travel has
    # exceeded the calibrated maximum. Missing evidence after that response
    # is inconclusive: it must neither restart full-rate take-up nor apply the
    # pre-response travel bound to an already observed engagement boundary.
    provisional = estimator.observe(
        [1.6, 0, 0], _pose(.2), _jacobian(), scale)
    first_miss = estimator.observe(
        [1.8, 0, 0], _pose(.2), _jacobian(), scale)
    second_miss = estimator.observe(
        [2.0, 0, 0], _pose(.2), _jacobian(), scale)

    assert provisional.phase[0] == "PROVISIONAL"
    assert first_miss.phase[0] == "PROVISIONAL"
    assert not first_miss.failed[0]
    assert second_miss.phase[0] == "PROVISIONAL"
    assert not second_miss.failed[0]
    assert second_miss.response_classification[0] == "INCONCLUSIVE"


def test_provisional_engagement_fails_closed_after_repeated_contradiction():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3,
        provisional_rejection_observations=2))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    provisional = estimator.observe(
        [.4, 0, 0], _pose(.1), _jacobian(), scale)
    first_contradiction = estimator.observe(
        [.6, 0, 0], _pose(-.1), _jacobian(), scale)
    failed = estimator.observe(
        [.8, 0, 0], _pose(-.3), _jacobian(), scale)

    assert provisional.phase[0] == "PROVISIONAL"
    assert first_contradiction.phase[0] == "PROVISIONAL"
    assert first_contradiction.response_classification[0] == "CONTRADICTORY"
    assert first_contradiction.provisional_rejection_count[0] == 1
    assert failed.phase[0] == "FAILED"
    assert failed.response_classification[0] == "CONTRADICTORY"
    assert failed.provisional_rejection_count[0] == 2


def test_hardware_style_response_gaps_do_not_revoke_provisional_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        maximum_width_gain=1.5,
        engagement_confirmation_observations=3,
        provisional_rejection_observations=2))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    first = estimator.observe([.4, 0, 0], _pose(.1), _jacobian(), scale)
    second = estimator.observe([.8, 0, 0], _pose(.2), _jacobian(), scale)
    small_same_direction = estimator.observe(
        [1.2, 0, 0], _pose(.22), _jacobian(), scale)
    accepted_noop = estimator.observe(
        [1.6, 0, 0], _pose(.22), _jacobian(), scale)
    confirmed = estimator.observe(
        [1.8, 0, 0], _pose(.32), _jacobian(), scale)

    assert first.phase[0] == "PROVISIONAL"
    assert second.phase[0] == "PROVISIONAL"
    assert second.confirmation_count[0] == 2
    assert small_same_direction.phase[0] == "PROVISIONAL"
    assert small_same_direction.response_classification[0] == "INCONCLUSIVE"
    assert accepted_noop.phase[0] == "PROVISIONAL"
    assert accepted_noop.response_classification[0] == "INCONCLUSIVE"
    assert not accepted_noop.failed[0]
    assert confirmed.phase[0] == "ENGAGED"
    assert confirmed.confirmation_count[0] == 3


def test_response_can_confirm_before_nominal_width_is_exhausted():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(2.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    engaged = estimator.observe(
        [.4, 0, 0], _pose(.2), _jacobian(), np.ones(6))

    assert engaged.phase[0] == "ENGAGED"
    assert engaged.remaining_rad[0] == 0.0
    assert engaged.engaged_direction[0] == 1


def test_virtual_motor_blocks_gap_and_passes_only_excess_travel():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01))

    initial = estimator.advance_motor([0, 0, 0])
    blocked = estimator.advance_motor([.6, 0, 0])
    crossed = estimator.advance_motor([1.2, 0, 0])
    continued = estimator.advance_motor([1.5, 0, 0])

    np.testing.assert_allclose(initial, [0, 0, 0])
    np.testing.assert_allclose(blocked, [0, 0, 0])
    np.testing.assert_allclose(crossed, [.2, 0, 0])
    np.testing.assert_allclose(continued, [.5, 0, 0])
    assert estimator.snapshot().remaining_rad[0] == 0.0


def test_virtual_motor_loads_new_gap_on_reversal():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01))
    estimator.advance_motor([0, 0, 0])
    estimator.advance_motor([1.5, 0, 0])

    blocked = estimator.advance_motor([1.0, 0, 0])
    crossed = estimator.advance_motor([0.0, 0, 0])

    np.testing.assert_allclose(blocked, [.5, 0, 0])
    np.testing.assert_allclose(crossed, [0.0, 0, 0])
    assert estimator.snapshot().motion_direction[0] == -1


def test_unconfirmed_takeup_beyond_calibrated_bound_fails_closed():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        maximum_width_gain=1.5,
        engagement_confirmation_observations=3))
    scale = np.ones(6)
    estimator.observe([0, 0, 0], _pose(), _jacobian(), scale)

    estimator.observe([.8, 0, 0], _pose(), _jacobian(), scale)
    failed = estimator.observe([1.6, 0, 0], _pose(), _jacobian(), scale)

    assert failed.phase[0] == "FAILED"
    assert failed.failed.tolist() == [True, False, False]


def test_candidate_rollout_ends_boost_when_geometric_gap_is_exhausted():
    state = BacklashSnapshot(
        width_rad=np.ones(3),
        width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),
        remaining_rad=np.array([.6, 0.0, 0.0]),
        motion_direction=np.array([1, 0, 0], dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("TAKEUP", "UNKNOWN", "UNKNOWN"))
    desired = np.zeros((1, 4, 3))
    desired[0, :, 0] = 2.0

    result = rollout_backlash_state(
        desired, state, [10.0, 10.0, 10.0], .04)

    # 0.4 rad is consumed per step. Step two passes 0.2 rad, then the desired
    # rate resumes while the epistemic TAKEUP phase awaits confirmation.
    np.testing.assert_allclose(
        result.transmitted_motor_radians_per_second[0, :, 0],
        [0.0, 5.0, 2.0, 2.0])
    np.testing.assert_allclose(
        result.compensated_motor_radians_per_second[0, :, 0],
        [10.0, 10.0, 2.0, 2.0])
    np.testing.assert_array_equal(
        result.taking_up[0, :, 0], [True, True, False, False])
    assert result.takeup_delay_s[0] == pytest.approx(.06)


def test_candidate_reversal_from_engaged_state_loads_directional_gap():
    state = BacklashSnapshot(
        width_rad=np.array([0.0, 7.0, 0.0]),
        width_positive_rad=np.array([0.0, 7.0, 0.0]),
        width_negative_rad=np.array([0.0, 3.0, 0.0]),
        remaining_rad=np.zeros(3),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"))
    desired = np.zeros((1, 2, 3))
    desired[0, :, 1] = -5.0

    result = rollout_backlash_state(
        desired, state, [8.0, 40.0, 4.5], .04)

    np.testing.assert_allclose(
        result.transmitted_motor_radians_per_second[0, :, 1], [0.0, -5.0])
    np.testing.assert_allclose(
        result.compensated_motor_radians_per_second[0, :, 1], -40.0)
    assert result.motion_direction[0, 1] == -1
    assert result.remaining_rad[0, 1] == pytest.approx(0.0)
    assert result.taking_up[0, :, 1].all()
    assert result.takeup_delay_s[0] == pytest.approx(3.0/40.0)


def test_candidate_parallel_takeup_delay_is_slowest_axis_not_sum():
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))
    desired = np.ones((1, 1, 3))

    result = rollout_backlash_state(
        desired, state, [2.0, 4.0, 8.0], .04)

    assert result.takeup_delay_s[0] == pytest.approx(.5)


def test_takeup_direction_guard_persists_prior_motor_direction():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        takeup_velocity=(8.0, 40.0, 4.5)))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.array([0., 1., 0.]),
        motion_direction=np.array([0, -1, 0], dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "TAKEUP", "UNKNOWN"))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)

    held = compensator.hold_takeup_direction(
        np.zeros(6), [0, -12, 0, 0, 0, 0], position, contract, state)

    motor = contract.project_velocity(
        held, position).requested_motor_axis_velocity
    assert motor[1] < 0.0


def test_coordinated_takeup_holds_already_engaged_shafts():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        takeup_velocity=(8.0, 40.0, 4.5)))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.array([0., 1., 0.]),
        motion_direction=np.array([1, -1, -1], dtype=np.int8),
        engaged_direction=np.array([1, 0, -1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("ENGAGED", "TAKEUP", "ENGAGED"))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([3.0, -12.0, 2.0, 0, 0, 0])

    coordinated = compensator.coordinate_takeup(
        desired, position, contract, state)
    motor = contract.project_velocity(
        coordinated, position).motor_radians_per_second

    assert motor[1] < 0.0
    np.testing.assert_allclose(motor[[0, 2]], 0.0, atol=1e-12)


def test_coordinated_takeup_releases_complete_coupled_command():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, 1, -1], dtype=np.int8),
        engaged_direction=np.array([1, 1, -1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([3.0, 12.0, 2.0, 0, 0, 0])

    coordinated = compensator.coordinate_takeup(
        desired, position, contract, state)

    np.testing.assert_allclose(coordinated, desired)


def test_coordinated_takeup_predicts_new_axis_before_encoder_motion():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, -1, 0], dtype=np.int8),
        engaged_direction=np.array([1, -1, 0], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "UNKNOWN"))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([3.0, -12.0, 2.0, 0, 0, 0])

    coordinated = compensator.coordinate_takeup(
        desired, position, contract, state)
    motor = contract.project_velocity(
        coordinated, position).motor_radians_per_second

    assert motor[2] < 0.0
    np.testing.assert_allclose(motor[[0, 1]], 0.0, atol=1e-12)


def test_coordinated_takeup_predicts_reversal_before_encoder_motion():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, 1, -1], dtype=np.int8),
        engaged_direction=np.array([1, 1, -1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([3.0, -12.0, 2.0, 0, 0, 0])

    coordinated = compensator.coordinate_takeup(
        desired, position, contract, state)
    motor = contract.project_velocity(
        coordinated, position).motor_radians_per_second

    assert motor[1] < 0.0
    np.testing.assert_allclose(motor[[0, 2]], 0.0, atol=1e-12)


def test_direction_specific_width_is_selected_on_reversal():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(0.0, 0.0, 0.0),
        width_positive_rad=(2.0, 2.0, 2.0),
        width_negative_rad=(0.5, 0.5, 0.5),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    positive = estimator.observe(
        [.4, 0, 0], _pose(), _jacobian(), np.ones(6))
    assert positive.remaining_rad[0] == pytest.approx(1.6)
    negative = estimator.observe(
        [.2, 0, 0], _pose(), _jacobian(), np.ones(6))
    assert negative.remaining_rad[0] == pytest.approx(.3)
    assert negative.width_positive_rad[0] == 2.0
    assert negative.width_negative_rad[0] == .5


def test_directional_and_symmetric_calibrations_cannot_be_mixed():
    with pytest.raises(ValueError, match="cannot be combined"):
        BacklashConfig(
            width_rad=(1.0, 0.0, 0.0),
            width_positive_rad=(2.0, 2.0, 2.0),
            width_negative_rad=(0.5, 0.5, 0.5))


def test_feedforward_boosts_unknown_direction_and_passes_engaged_command():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    config = BacklashConfig(
        width_rad=(1.0, 2.0, 3.0),
        takeup_velocity=(8.0, 40.0, 4.5))
    estimator = BacklashStateEstimator(config)
    compensator = BacklashFeedforwardCompensator(config)
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([2.0, 7.0, 2.0, 0, 0, 0])

    boosted = compensator.command(
        desired, position, contract, estimator.snapshot())
    # Logical insertion equals bend, so firmware coupling leaves shaft 0
    # stationary. Boosting shaft 2 requires inverse-coupling insertion too.
    np.testing.assert_allclose(
        boosted[:3], [4.5, 40.0, 4.5], rtol=0.0, atol=0.01)

    motor_sign = np.sign(contract.project_velocity(
        desired, position).motor_radians_per_second[:3]).astype(np.int8)
    estimator.engaged_direction[:] = motor_sign
    estimator.motion_direction[:] = motor_sign
    estimator.remaining[:] = 0.0
    estimator.phase[:] = ["ENGAGED"]*3
    passed = compensator.command(
        desired, position, contract, estimator.snapshot())
    np.testing.assert_allclose(passed, desired)


def test_feedforward_drops_boost_after_gap_but_before_confirmation():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 2.0, 3.0),
        takeup_velocity=(8.0, 40.0, 4.5)))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    desired = np.array([2.0, 7.0, 2.0, 0, 0, 0])
    directions = np.sign(contract.project_velocity(
        desired, position).motor_radians_per_second[:3]).astype(np.int8)
    state = BacklashSnapshot(
        width_rad=np.array([1.0, 2.0, 3.0]),
        width_positive_rad=np.array([1.0, 2.0, 3.0]),
        width_negative_rad=np.array([1.0, 2.0, 3.0]),
        remaining_rad=np.zeros(3), motion_direction=directions,
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("TAKEUP", "TAKEUP", "TAKEUP"))

    passed = compensator.command(desired, position, contract, state)

    np.testing.assert_allclose(passed, desired)


def test_transaction_holds_ready_shafts_and_requires_fresh_replan():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        takeup_velocity=(8.0, 40.0, 4.5)))
    arbiter = TakeupTransactionArbiter(compensator)
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    requested_rates = np.array([5.0, -5.0, 3.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(
            requested_rates))

    unknown = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))
    started = arbiter.begin(desired, position, contract, unknown)
    assert started.state == "TAKEUP_ACTIVE"
    assert not started.execute_plan
    assert started.pending_mask.tolist() == [True, True, True]

    partially_ready = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.array([0., .5, 0.]),
        motion_direction=np.array([1, -1, 1], dtype=np.int8),
        engaged_direction=np.array([1, 0, 1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "TAKEUP", "ENGAGED"))
    partial = arbiter.advance(position, contract, partially_ready)
    partial_rates = contract.project_velocity(
        partial.command_logical_velocity,
        position).motor_radians_per_second[:3]
    assert partial.pending_mask.tolist() == [False, True, False]
    assert partial_rates[1] < 0.0
    np.testing.assert_allclose(partial_rates[[0, 2]], 0.0, atol=1e-12)

    ready = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([1, -1, 1], dtype=np.int8),
        engaged_direction=np.array([1, -1, 1], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "ENGAGED", "ENGAGED"))
    complete = arbiter.advance(position, contract, ready)
    assert complete.state == "REPLAN_REQUIRED"
    assert complete.replan_required
    np.testing.assert_array_equal(complete.command_logical_velocity, 0.0)
    assert arbiter.release_for_replan()

    replanned = arbiter.begin(desired, position, contract, ready)
    assert replanned.execute_plan
    np.testing.assert_allclose(
        replanned.command_logical_velocity,
        contract.project_velocity(desired, position).logical_velocity)


def test_transaction_holds_zero_until_provisional_response_is_confirmed():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    arbiter = TakeupTransactionArbiter(compensator)
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    rates = np.array([0.0, 4.0, 0.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(rates))
    taking_up = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))
    assert arbiter.begin(
        desired, position, contract, taking_up).state == "TAKEUP_ACTIVE"

    provisional = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.array([0, 1, 0], dtype=np.int32),
        phase=("UNKNOWN", "PROVISIONAL", "UNKNOWN"))
    decision = arbiter.advance(position, contract, provisional)

    assert decision.state == "CONFIRMATION_HOLD"
    assert not decision.replan_required
    assert not decision.execute_plan
    np.testing.assert_array_equal(decision.command_logical_velocity, 0.0)


def test_transaction_confirmation_hold_times_out_fail_closed():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    now = [0.0]
    arbiter = TakeupTransactionArbiter(
        BacklashFeedforwardCompensator(BacklashConfig(
            width_rad=(1.0, 1.0, 1.0))),
        confirmation_hold_timeout_s=0.5, clock=lambda: now[0])
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    rates = np.array([0.0, 4.0, 0.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(rates))
    taking_up = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))
    assert arbiter.begin(
        desired, position, contract, taking_up).state == "TAKEUP_ACTIVE"
    provisional = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.zeros(3),
        motion_direction=np.array([0, 1, 0], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.zeros(3),
        confirmation_count=np.array([0, 1, 0], dtype=np.int32),
        phase=("UNKNOWN", "PROVISIONAL", "UNKNOWN"))

    assert arbiter.advance(
        position, contract, provisional).state == "CONFIRMATION_HOLD"
    now[0] = 0.51
    timed_out = arbiter.advance(position, contract, provisional)

    assert timed_out.state == "FAILED"
    assert timed_out.reason == "takeup_confirmation_timeout"
    np.testing.assert_array_equal(timed_out.command_logical_velocity, 0.0)


def test_transaction_coupled_direction_feasibility_is_atomic():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    arbiter = TakeupTransactionArbiter(
        BacklashFeedforwardCompensator(BacklashConfig(
            width_rad=(1.0, 1.0, 1.0),
            takeup_velocity=(1.0, 1.0, 0.5))))
    home = np.array([20, 0, 0, 0, 0, 0], dtype=float)

    assert arbiter.physical_direction_is_feasible(0, 1, home, contract)
    assert arbiter.physical_direction_is_feasible(2, -1, home, contract)
    assert not arbiter.physical_direction_vector_is_feasible(
        [1, 0, -1], home, contract)


def test_transaction_ignores_uncommanded_shaft_takeup():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    arbiter = TakeupTransactionArbiter(compensator)
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    rates = np.array([0.0, 4.0, 0.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(rates))
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.array([1., 0., 1.]),
        motion_direction=np.array([1, 1, -1], dtype=np.int8),
        engaged_direction=np.array([0, 1, 0], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("TAKEUP", "ENGAGED", "TAKEUP"))

    decision = arbiter.begin(desired, position, contract, state)

    assert decision.execute_plan
    assert decision.active_mask.tolist() == [False, True, False]


def test_transaction_never_bypasses_unconfirmed_axis():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0)))
    arbiter = TakeupTransactionArbiter(
        compensator, response_free_mask=(True, True, False))
    position = np.array([20, 0, 7, 40, 0, 0], dtype=float)
    rates = np.array([0.0, 0.0, 3.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(rates))
    unknown = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "TAKEUP"))

    decision = arbiter.begin(desired, position, contract, unknown)

    assert not decision.execute_plan
    assert decision.state == TakeupTransactionArbiter.TAKEUP_ACTIVE
    assert decision.active_mask.tolist() == [False, False, True]
    assert abs(decision.requested_motor_radians_per_second[2]) > 0.0
    assert abs(decision.realized_motor_radians_per_second[2]) > 0.0


def test_transaction_saturates_atomically_at_coupled_joint_boundary():
    """A blocked bend shaft must not leak insertion during take-up."""
    contract = load_hardware_contract(LIMITS, "imricor_test")
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        takeup_velocity=(8.0, 40.0, 4.5)))
    arbiter = TakeupTransactionArbiter(compensator)
    middle = np.array([24, 0, 7, 40, 0, 0], dtype=float)
    # Positive physical shaft 2 requires negative logical bend.  It is
    # feasible in the middle of the range, then becomes infeasible when bend
    # reaches its lower boundary during the transaction.
    physical_rates = np.array([5.0, 0.0, 3.0, 0, 0, 0])
    desired = contract.motor_axis_to_logical_velocity(
        contract.motor_radians_per_second_to_motor_axis_velocity(
            physical_rates))
    unknown = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))

    started = arbiter.begin(desired, middle, contract, unknown)
    assert started.state == TakeupTransactionArbiter.TAKEUP_ACTIVE

    shaft_zero_ready = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.array([0., 1., 1.]),
        motion_direction=np.array([1, 0, 1], dtype=np.int8),
        engaged_direction=np.array([1, 0, 0], dtype=np.int8),
        confidence=np.ones(3),
        confirmation_count=np.ones(3, dtype=np.int32),
        phase=("ENGAGED", "UNKNOWN", "UNKNOWN"))
    boundary = middle.copy()
    boundary[2] = contract.control_position_lower[2]
    saturated = arbiter.advance(boundary, contract, shaft_zero_ready)

    assert saturated.state == TakeupTransactionArbiter.SATURATED_REPLAN
    assert saturated.replan_required
    np.testing.assert_array_equal(saturated.command_logical_velocity, 0.0)
    assert saturated.saturated_mask.tolist() == [False, False, True]
    assert saturated.leakage_mask.tolist() == [True, False, False]
    assert saturated.requested_motor_radians_per_second[2] > 0.0
    assert saturated.realized_motor_radians_per_second[2] == 0.0
    assert saturated.realized_motor_radians_per_second[0] < 0.0
    assert arbiter.release_for_replan()


def test_saturated_physical_direction_restores_only_after_limit_clearance():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    arbiter = TakeupTransactionArbiter(
        BacklashFeedforwardCompensator(BacklashConfig(
            width_rad=(1.0, 1.0, 1.0))))
    boundary = np.array([24, 0, contract.control_position_lower[2],
                         40, 0, 0], dtype=float)

    assert not arbiter.physical_direction_is_feasible(
        2, 1, boundary, contract)
    assert arbiter.physical_direction_is_feasible(
        2, -1, boundary, contract)

    cleared = boundary.copy()
    cleared[2] += 1.0
    assert arbiter.physical_direction_is_feasible(
        2, 1, cleared, contract)


def test_hardware_profile_keeps_takeup_separate_from_adaptation_holdoff():
    profile = yaml.safe_load((
        ROOT / "catheter_control" / "config"
        / "causal_v2_fixed_hardware.yaml").read_text())
    parameters = profile["catheter_mppi"]["ros__parameters"]
    assert parameters["backlash_width_rad"] == [0.0, 0.0, 0.0]
    assert parameters["backlash_width_positive_rad"] == [
        8.369, 7.020, 34.858]
    assert parameters["backlash_width_negative_rad"] == [
        6.856, 7.090, 3.587]
    assert parameters["backlash_takeup_velocity"] == [2.0, 5.0, 1.0]
    assert parameters[
        "adaptation_reversal_holdoff_normalized_action_shaft_2"] == 4.0
    assert (parameters["backlash_width_positive_rad"][2]
            != parameters[
                "adaptation_reversal_holdoff_normalized_action_shaft_2"])


def test_v175_hardware_profile_loads_atomic_transmission_artifact():
    profile = yaml.safe_load((
        ROOT / "catheter_control" / "config"
        / "v175_grouped_hardware.yaml").read_text())
    parameters = profile["catheter_mppi"]["ros__parameters"]
    assert "interface_transmission_checkpoint" not in parameters
    assert "jacobian_initialization_json" not in parameters
    launch = (ROOT / "bringup" / "launch"
              / "control.launch.py").read_text()
    assert "20260929_175554_grouped_no_rotation_v2.json" in launch
    assert parameters["backlash_compensation_enabled"] is True
    assert parameters["takeup_transaction_enabled"] is True
    assert parameters["backlash_width_rad"] == [0.0, 0.0, 0.0]
    assert parameters["backlash_width_positive_rad"] == [0.0, 0.0, 0.0]
    assert parameters["backlash_width_negative_rad"] == [0.0, 0.0, 0.0]
    assert parameters["backlash_takeup_velocity"] == [1.0, 2.5, 0.5]
    assert parameters["engaged_gain_enabled"] is True
    assert parameters["mppi_engaged_gain_scenarios"] is True
    assert parameters["mppi_engaged_gain_maximum_first_step_shift"] == 8.0
    assert parameters["horizon_steps"] == 4
    assert parameters["rollout_step_s"] == pytest.approx(0.04)
    assert parameters["mppi_point_rollout_step_s"] == pytest.approx(0.20)
    assert parameters["mppi_point_rollout_coarse_steps"] is True
    assert parameters["mppi_point_prediction_tail_steps"] == 0
    assert parameters["samples"] == 512
    assert "command_output_enabled" not in parameters


def test_backlash_belief_tracks_directional_interval_and_checkpoint():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 0.0, 0.0),
        minimum_motor_increment_rad=0.01,
        minimum_width_gain=0.5,
        maximum_width_gain=1.5))
    estimator.advance_motor([0.0, 0.0, 0.0])
    estimator.advance_motor([0.2, 0.0, 0.0])
    state = estimator.snapshot()
    assert state.remaining_rad[0] == pytest.approx(0.8)
    assert state.remaining_lower_rad[0] == pytest.approx(0.3)
    assert state.remaining_upper_rad[0] == pytest.approx(1.3)
    assert state.reversal_start_motor_rad[0] == pytest.approx(0.0)

    checkpoint = estimator.clone_state()
    estimator.advance_motor([0.6, 0.0, 0.0])
    assert estimator.snapshot().remaining_upper_rad[0] == pytest.approx(0.9)
    estimator.restore_state(checkpoint)
    assert estimator.snapshot().remaining_upper_rad[0] == pytest.approx(1.3)


def test_post_response_coast_then_stationary_updates_confirm_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    first = estimator.observe(
        [.4, 0, 0], _pose(.1), _jacobian(), np.ones(6))
    assert first.phase[0] == "PROVISIONAL"
    assert first.confirmation_count[0] == 1

    coast = estimator.observe_response(
        [.42, 0, 0], _pose(.1), _jacobian(), np.ones(6), timestamp_ns=20)
    second = estimator.observe_response(
        [.42, 0, 0], _pose(.1), _jacobian(), np.ones(6), timestamp_ns=30)
    final = estimator.observe_response(
        [.42, 0, 0], _pose(.1), _jacobian(), np.ones(6), timestamp_ns=40)

    assert coast.phase[0] == "PROVISIONAL"
    assert coast.confirmation_count[0] == 1
    assert second.phase[0] == "PROVISIONAL"
    assert second.confirmation_count[0] == 2
    assert final.phase[0] == "ENGAGED"
    assert final.confirmation_count[0] == 3
    assert final.engagement_anchor_motor_rad[0] == pytest.approx(.4)


def test_zero_motion_marker_updates_confirm_provisional_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    first = estimator.observe(
        [.4, 0, 0], _pose(.1), _jacobian(), np.ones(6))
    assert first.phase[0] == "PROVISIONAL"
    second = estimator.observe_response(
        [.4, 0, 0], _pose(.1), _jacobian(), np.ones(6),
        timestamp_ns=20)
    final = estimator.observe_response(
        [.4, 0, 0], _pose(.1), _jacobian(), np.ones(6),
        timestamp_ns=30)
    assert second.phase[0] == "PROVISIONAL"
    assert final.phase[0] == "ENGAGED"
    assert final.last_evidence_timestamp_ns[0] == 30
