"""Session noise calibration is distinct from tendon transaction evidence."""
import numpy as np
import pytest

from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator


def observe(owner, i, *, motor=0., chord_delta_mm=0., external=False):
    raw = np.array([0., 0., motor])
    owner.advance_motor(raw, external_motion=external)
    points = np.array([[0., 0., z] for z in (0., .03, .06, .09)])
    points[-1, 2] += chord_delta_mm/1000.
    return owner.observe_response(raw, np.eye(4), np.zeros((6, 3)), np.ones(6),
        timestamp_ns=i*50_000_000, observed_marker_points_m=points)


def qualified():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    for i in range(12):
        observe(owner, i, chord_delta_mm=.02*(-1 if i % 2 else 1))
    return owner


def test_noise_qualifies_before_first_direction_change():
    owner = qualified()
    assert owner.distal_onset_noise is not None


def test_homing_ownership_resets_transaction_not_noise_calibration():
    owner = qualified()
    noise = owner.distal_onset_noise
    observe(owner, 12, motor=.1, external=True)
    assert owner.distal_onset_noise == noise
    for i in range(13, 17):
        state = observe(owner, i, motor=.1*(i-11), chord_delta_mm=.24, external=True)
    assert state.tendon_response_onset
    assert state.phase[2] == 'TAKEUP'
    observe(owner, 17, motor=.5, chord_delta_mm=.24)
    assert owner.distal_onset_noise == noise
    assert not owner.snapshot().tendon_response_onset
    assert owner.confirmation_count[2] == 0


def test_real_settling_cannot_inflate_qualified_onset_floor():
    owner = qualified()
    floor = owner.snapshot().tendon_response_onset_floor_mm
    for i in range(12, 30):
        observe(owner, i, chord_delta_mm=(i-11)*.06)
        assert owner.snapshot().tendon_response_onset_floor_mm == floor


def test_without_calibration_settling_does_not_qualify_as_noise():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    for i in range(20):
        observe(owner, i, chord_delta_mm=i*.06)
    assert owner.distal_onset_noise is None


@pytest.mark.parametrize('reason', ['gap', 'explicit_reset'])
def test_sensor_discontinuity_still_invalidates_calibration(reason):
    owner = qualified()
    if reason == 'gap':
        observe(owner, 100, motor=.1)
    else:
        owner.reset_observation_history()
    assert owner.distal_onset_noise is None


def test_calibration_and_thresholds_survive_checkpoint_restore():
    owner = qualified()
    observe(owner, 12, motor=.1)
    state = observe(owner, 13, motor=.2, chord_delta_mm=.24)
    checkpoint = owner.clone_state()
    owner.reset_observation_history()
    owner.restore_state(checkpoint)
    assert owner.snapshot().tendon_response_noise_mm == state.tendon_response_noise_mm
    assert owner.snapshot().tendon_response_onset_floor_mm == state.tendon_response_onset_floor_mm


def test_ownership_change_cannot_hide_sensor_gap():
    owner = qualified()
    observe(owner, 100, motor=.1, external=True)
    assert owner.distal_onset_noise is None
    assert not owner.snapshot().tendon_response_onset
