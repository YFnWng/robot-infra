"""Homing evidence informs inference, without an MPPI transaction or writes."""
from types import SimpleNamespace

import numpy as np
import pytest

from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator


def evidence_session():
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(.1, .1, .1), engagement_confirmation_observations=3,
        width_learning_rate=.5))
    history = []
    model = SimpleNamespace(jacobian=np.eye(6, 3), state_scale=np.ones(6))

    def sample(i, encoder, deformation_mm=0., *, owned=False, translation=0.):
        motor = np.array([0., 0., encoder])
        stamp = i*50_000_000
        owner.advance_motor(motor, external_motion=not owned)
        history.append([stamp, motor.copy(), owner.clone_state(), owned])
        markers = np.array([[0., 0., 0.], [0., 0., .01],
                            [0., 0., .02], [0., 0., .03]])
        markers[-1, 2] += deformation_mm/1000.
        markers[:, 0] += translation
        snapshot, updated = owner.observe_at_timestamp(history, stamp, np.eye(4), model,
            control_owned=owned, observed_interface_position_m=markers[0],
            observed_marker_points_m=markers)
        history[:] = updated
        return snapshot

    return owner, history, model, sample


@pytest.mark.parametrize('direction', [-1., 1.])
def test_external_signed_distal_response_confirms_without_width_learning(direction):
    owner, history, model, sample = evidence_session()
    initial_positive = owner.width_positive.copy()
    initial_negative = owner.width_negative.copy()
    sample(0, 0.)
    sample(1, direction*.1)
    for i in range(2, 10):
        state = sample(i, direction*i, direction*.8)
    assert state.phase[2] == 'ENGAGED'
    assert state.motion_direction[2] == direction
    np.testing.assert_array_equal(owner.width_positive, initial_positive)
    np.testing.assert_array_equal(owner.width_negative, initial_negative)
    # An arbitrarily long unresponsive external move has no MPPI travel fault.
    # Arming invalidates that external transaction, including on a static sample.
    state = sample(10, direction*9, direction*.8, owned=True)
    assert state.phase[2] == 'UNKNOWN'
    assert state.confirmation_count[2] == 0
    before = owner.clone_state()
    owner.observe_at_timestamp(history, 9*50_000_000, np.eye(4), model,
        control_owned=True, observed_interface_position_m=np.zeros(3),
        observed_marker_points_m=np.zeros((4, 3)))
    assert owner.phase == before['phase']  # Delayed homing cannot re-confirm after arm.


def test_external_travel_without_nonrigid_response_never_confirms_or_faults():
    owner, _, _, sample = evidence_session()
    sample(0, 0.)
    for i in range(1, 20):
        state = sample(i, float(i), translation=i*.001)
        assert state.phase[2] == 'TAKEUP'
        assert state.confirmation_count[2] == 0


def test_external_reversal_discards_old_response_and_clone_preserves_ownership():
    owner, _, _, sample = evidence_session()
    sample(0, 0.)
    sample(1, .1)
    for i in range(2, 10):
        sample(i, float(i), .8)
    assert owner.phase[2] == 'ENGAGED'
    state = sample(10, 8., .8)
    assert state.phase[2] == 'TAKEUP'
    assert state.confirmation_count[2] == 0
    checkpoint = owner.clone_state()
    clone = BacklashStateEstimator(owner.config)
    clone.restore_state(checkpoint)
    assert clone.external_motion and clone.phase == owner.phase


def test_homing_confirmation_releases_useful_travel_gate_not_stationary_bound():
    from cr_meta_lnn.tests.test_joint_engagement_filter import coherent_confirmed
    from catheter_control.planning.engagement import tendon_confirmation
    value, _ = coherent_confirmed()
    owner, _, _, sample = evidence_session()
    runtime = SimpleNamespace(tendon_mm_per_shaft_rad=-.18949385411878789)
    useful = []
    for i in range(10):
        encoder = 0. if i == 0 else (.1 if i == 1 else float(i))
        state = sample(i, encoder, .8 if i >= 2 else 0.)
        decision = tendon_confirmation(runtime, state)
        value.advance((i+1)*.05, encoder*runtime.tendon_mm_per_shaft_rad,
                      confirmation=decision)
        useful.append(value._has_confirmed_tendon_travel())
        value.correct((i+1)*.05, value.points()[0])
    assert not any(useful[:3])
    assert useful[-1] and owner.phase[2] == 'ENGAGED'
    # Being physically engaged alone does not renew the held-input allowance.
    state = sample(10, encoder, .8)
    value.advance(.55, encoder*runtime.tendon_mm_per_shaft_rad,
                  confirmation=tendon_confirmation(runtime, state))
    assert not value._has_confirmed_tendon_travel()
