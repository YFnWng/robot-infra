from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest
import yaml

from catheter_control.transmission.backlash import (
    BacklashConfig, BacklashFeedforwardCompensator, BacklashSnapshot,
    BacklashStateEstimator, TakeupTransactionArbiter,
    rollout_backlash_state, rollout_raw_interface_play)
from catheter_control.safety.hardware_contract import (
    load_hardware_contract, knob_interface_translation_m)


def test_chassis_first_owned_motion_requires_new_proof_even_with_zero_width():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 0.)))
    owner.advance_motor([0., 0., 0.], external_motion=True)
    owner.phase[0] = 'ENGAGED'
    owner.engaged_direction[0] = 1
    owner.motion_direction[0] = 0
    owner.last_evidence_timestamp_ns[0] = 123
    owner.advance_motor([0., 0., 0.], external_motion=False)
    assert owner.phase[0] == 'UNKNOWN'
    assert owner.engaged_direction[0] == 0
    assert owner.last_evidence_timestamp_ns[0] == 0
    owner.advance_motor([-.2, 0., 0.], external_motion=False)
    assert owner.phase[0] == 'TAKEUP'
    assert owner.motion_direction[0] == -1
    assert owner.confirmation_count[0] == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_noise_qualified_early_tendon_response_does_not_confirm(direction):
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    def observe(index, motor, delta=0.):
        points = np.array([[0., 0., z] for z in (0., .03, .06, .09)])
        points[-1, 2] += delta
        raw = np.array([0., 0., motor*direction])
        owner.advance_motor(raw)
        return owner.observe_response(raw, _pose(), np.zeros((6, 3)),
            np.ones(6), timestamp_ns=index*50_000_000,
            observed_marker_points_m=points)
    for i in range(12):observe(i, 0.)
    observe(12, .1)
    for i in range(13, 17):state=observe(i, .1*(i-11), direction*.00024)
    assert state.tendon_response_onset
    assert state.phase[2] == 'TAKEUP'
    assert state.confirmation_count[2] == 0
    checkpoint = owner.clone_state()
    owner.reset_observation_history()
    assert not owner.snapshot().tendon_response_onset
    owner.restore_state(checkpoint)
    assert owner.snapshot().tendon_response_onset
    owner.advance_motor([0., 0., -direction])
    assert not owner.snapshot().tendon_response_onset
    observe(17, -1.)
    assert owner.distal_onset_noise is not None
    assert not owner.snapshot().tendon_response_onset


@pytest.mark.parametrize('response', ['spike', 'opposing', 'noisy', 'gap'])
def test_early_response_rejects_unqualified_raw_geometry(response):
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    def observe(index, motor, delta=0.):
        points = np.array([[0., 0., z] for z in (0., .03, .06, .09)])
        points[-1, 2] += delta
        raw = [0., 0., -motor]
        owner.advance_motor(raw)
        return owner.observe_response(raw, _pose(), np.zeros((6, 3)),
            np.ones(6), timestamp_ns=index*50_000_000,
            observed_marker_points_m=points)
    for i in range(12):
        observe(i, 0., .0002*(1 if i % 4 < 2 else -1)
                if response == 'noisy' else 0.)
    start = 100 if response == 'gap' else 12
    observe(start, .1)
    for i in range(start+1, start+5):
        delta = .00024 if response == 'opposing' else -.00024
        if response == 'spike' and i != start+2:
            delta = 0.
        state = observe(i, .1*(i-start+1), delta)
    assert not state.tendon_response_onset
    assert state.phase[2] == 'TAKEUP'


def test_early_response_requires_qualified_quiet_history():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    for i in range(6):
        raw = [0., 0., -.1*i]
        owner.advance_motor(raw)
        points=np.array([[0., 0., z] for z in (0., .03, .06, .09)])
        points[-1, 2] -= .00024 if i else 0.
        state=owner.observe_response(raw, _pose(), np.zeros((6, 3)),
            np.ones(6), timestamp_ns=i*50_000_000,
            observed_marker_points_m=points)
    assert not state.tendon_response_onset
    assert state.phase[2] == 'TAKEUP'


def test_early_response_hold_requires_post_stop_evidence_before_burst():
    contract = load_hardware_contract(LIMITS, 'imricor_test')
    now=[0.]
    arbiter=TakeupTransactionArbiter(BacklashFeedforwardCompensator(
        BacklashConfig(width_rad=(1., 1., 1.))), clock=lambda:now[0])
    state=BacklashSnapshot(width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.array([0, 0, -1]), engaged_direction=np.zeros(3),
        confidence=np.zeros(3), confirmation_count=np.zeros(3, dtype=int),
        phase=('UNKNOWN', 'UNKNOWN', 'TAKEUP'))
    position=np.array([20., 0., 7., 40., 0., 0.])
    arbiter.begin([3., 0., 3., 0., 0., 0.], position, contract, state)
    early=replace(state, tendon_response_onset=True, tendon_response_encoder_quiet=True)
    for time in (0., .1, .199):
        now[0]=time;decision=arbiter.advance(position, contract, early)
        assert decision.state == arbiter.RESPONSE_HOLD
        assert not decision.execute_plan and not decision.replan_required
        np.testing.assert_array_equal(decision.command_logical_velocity, 0.)
    now[0]=.21
    assert arbiter.advance(position, contract, early).state == arbiter.RESPONSE_HOLD
    arbiter.note_command_publication(1_000_000_000, False)
    # Duplicates, pre-stop frames, and elapsed time alone cannot add travel.
    for stamp in (900_000_000, 1_050_000_000, 1_050_000_000, 1_100_000_000):
        evidence=replace(early, tendon_response_timestamp_ns=stamp)
        assert arbiter.advance(position, contract, evidence).state == arbiter.RESPONSE_HOLD
    moving=replace(early,tendon_response_timestamp_ns=1_125_000_000,
                   tendon_response_encoder_quiet=False)
    assert arbiter.advance(position,contract,moving).state==arbiter.RESPONSE_HOLD
    assert arbiter.response_post_stop_observations==0
    for stamp in (1_130_000_000,1_140_000_000):
        assert arbiter.advance(position,contract,replace(early,tendon_response_timestamp_ns=stamp)).state==arbiter.RESPONSE_HOLD
    burst=arbiter.advance(position, contract, replace(early, tendon_response_timestamp_ns=1_150_000_000))
    assert burst.state == arbiter.TAKEUP_ACTIVE
    assert burst.command_logical_velocity[2] > 0.
    assert burst.realized_motor_radians_per_second[0] == 0.
    now[0]=.3
    assert arbiter.advance(position, contract, early).state == arbiter.RESPONSE_HOLD
    # A second burst requires a new published stop and three new images.
    now[0]=.5
    assert arbiter.advance(position, contract, evidence).state == arbiter.RESPONSE_HOLD
    provisional=replace(early, phase=('UNKNOWN', 'UNKNOWN', 'PROVISIONAL'),
        engaged_direction=np.array([0, 0, -1]))
    assert arbiter.advance(position, contract, provisional).state == arbiter.CONFIRMATION_HOLD
    engaged=replace(provisional, phase=('UNKNOWN', 'UNKNOWN', 'ENGAGED'))
    complete=arbiter.advance(position, contract, engaged)
    assert complete.replan_required and not complete.execute_plan
    arbiter.reset()
    assert not arbiter.response_onset_seen


def test_onset_preview_is_source_aligned_stop_only_and_does_not_mutate_owner():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(0., 0., 10.)))
    points = np.array([[0., 0., z] for z in (0., .03, .06, .09)])
    history=[]
    for i in range(14):
        motor=np.array([0., 0., max(0, i-11)*.1])
        owner.advance_motor(motor)
        observed=points.copy()
        if i>=13:observed[-1,2]+=.00024
        owner.observe_response(motor, _pose(), np.zeros((6,3)), np.ones(6),
                               timestamp_ns=i*50_000_000, observed_marker_points_m=observed)
        history.append([i*50_000_000, motor.copy(), owner.clone_state(), True])
    motor=np.array([0.,0.,.3]);owner.advance_motor(motor)
    history.append([700_000_000, motor, owner.clone_state(), True])
    before=owner.snapshot()
    assert not before.tendon_response_onset
    preview=owner.preview_tendon_onset(history, 700_000_000, observed, control_owned=True)
    assert preview.tendon_response_onset
    assert preview.phase[2]=='TAKEUP' and preview.confirmation_count[2]==0
    assert preview.tendon_response_timestamp_ns==700_000_000
    after=owner.snapshot()
    assert after.tendon_response_timestamp_ns==before.tendon_response_timestamp_ns
    assert not after.tendon_response_onset
    np.testing.assert_array_equal(after.confirmation_count, before.confirmation_count)
    assert history[-1][2]['distal_marker_timestamp_ns']==before.tendon_response_timestamp_ns
    assert owner.preview_tendon_onset(history, 725_000_000, observed, control_owned=True) is None
    assert owner.preview_tendon_onset(history, 700_000_000, observed, control_owned=False) is None


def test_response_hold_missing_evidence_times_out_without_more_motion():
    contract = load_hardware_contract(LIMITS, 'imricor_test')
    now=[0.]
    arbiter=TakeupTransactionArbiter(BacklashFeedforwardCompensator(
        BacklashConfig(width_rad=(1.,1.,1.))), clock=lambda:now[0])
    state=BacklashSnapshot(width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.array([0,0,-1]), engaged_direction=np.zeros(3),
        confidence=np.zeros(3), confirmation_count=np.zeros(3,dtype=int),
        phase=('UNKNOWN','UNKNOWN','TAKEUP'), tendon_response_onset=True)
    position=np.array([20.,0.,7.,40.,0.,0.])
    arbiter.begin([3.,0.,3.,0.,0.,0.], position, contract, state)
    assert arbiter.advance(position, contract, state).state==arbiter.RESPONSE_HOLD
    now[0]=1.01
    decision=arbiter.advance(position, contract, state)
    assert decision.state==arbiter.FAILED
    assert decision.reason=='takeup_response_evidence_timeout'
    assert not np.any(decision.command_logical_velocity)


def test_confirmed_tendon_does_not_block_other_pending_shaft_after_onset():
    contract=load_hardware_contract(LIMITS,'imricor_test')
    arbiter=TakeupTransactionArbiter(BacklashFeedforwardCompensator(
        BacklashConfig(width_rad=(1.,1.,1.))))
    state=BacklashSnapshot(width_rad=np.ones(3),width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),remaining_rad=np.ones(3),
        motion_direction=np.array([1,0,-1]),engaged_direction=np.zeros(3),
        confidence=np.zeros(3),confirmation_count=np.zeros(3,dtype=int),
        phase=('TAKEUP','UNKNOWN','TAKEUP'),tendon_response_onset=True)
    position=np.array([20.,0.,7.,40.,0.,0.])
    # Logical equal insertion/knob rates cancel chassis, so command insertion
    # faster than knob to obtain a genuine two-shaft transaction.
    arbiter.begin([6.,0.,3.,0.,0.,0.],position,contract,state)
    assert arbiter.advance(position,contract,state).state==arbiter.RESPONSE_HOLD
    engaged=replace(state,phase=('TAKEUP','UNKNOWN','ENGAGED'),
                    engaged_direction=np.array([0,0,-1]))
    decision=arbiter.advance(position,contract,engaged)
    assert decision.state==arbiter.TAKEUP_ACTIVE
    assert decision.realized_motor_radians_per_second[0]!=0
    assert decision.realized_motor_radians_per_second[2]==0


def test_onset_from_previous_transaction_cannot_stop_new_acquisition():
    contract=load_hardware_contract(LIMITS,'imricor_test')
    arbiter=TakeupTransactionArbiter(BacklashFeedforwardCompensator(
        BacklashConfig(width_rad=(1.,1.,1.))))
    state=BacklashSnapshot(width_rad=np.ones(3),width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),remaining_rad=np.ones(3),
        motion_direction=np.array([0,0,-1]),engaged_direction=np.zeros(3),
        confidence=np.zeros(3),confirmation_count=np.zeros(3,dtype=int),
        phase=('UNKNOWN','UNKNOWN','TAKEUP'),tendon_response_timestamp_ns=100)
    position=np.array([20.,0.,7.,40.,0.,0.])
    arbiter.begin([3.,0.,3.,0.,0.,0.],position,contract,state)
    assert not arbiter.note_tendon_onset(replace(state,tendon_response_onset=True))
    assert arbiter.note_tendon_onset(replace(state,tendon_response_onset=True,
                                            tendon_response_timestamp_ns=101))


def test_rotation_takeup_uses_same_synchronized_contract():
    contract = replace(load_hardware_contract(LIMITS, "imricor_test"),
                       synchronized_sheath_rotation=True)
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1., 1., 1.)))
    arbiter = TakeupTransactionArbiter(compensator)
    state = BacklashSnapshot(
        width_rad=np.ones(3), width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3), remaining_rad=np.ones(3),
        motion_direction=np.zeros(3, dtype=np.int8),
        engaged_direction=np.zeros(3, dtype=np.int8), confidence=np.zeros(3),
        confirmation_count=np.zeros(3, dtype=np.int32),
        phase=("UNKNOWN", "UNKNOWN", "UNKNOWN"))
    position = np.array([20., 0., 7., 40., 0., 0.])
    decision = arbiter.begin(np.array([0., 10., 0., 0., 10., -10.]), position, contract, state)
    assert decision.state == "TAKEUP_ACTIVE"
    projected = contract.project_velocity(decision.command_logical_velocity, position)
    assert projected.motor_rpm[1] != 0.
    assert projected.motor_rpm[1] == projected.motor_rpm[4]
    assert projected.motor_rpm[5] == 0.


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


def _observe_shape(estimator, motor, pose, jacobian, scale, strain, mode):
    estimator.advance_motor(motor)
    return estimator.observe_response(motor, pose, jacobian, scale, strain, mode)


@pytest.mark.parametrize("budget", [0., 100.])
def test_raw_tendon_confirmation_ignores_strain_and_rigid_motion(budget):
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3,
        tendon_unconfirmed_travel_limit_rad=budget))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    def observe(motor, markers, strain, stamp):
        owner.advance_motor(motor)
        return owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            [strain], [1.], timestamp_ns=stamp,
            observed_marker_points_m=markers)
    observe([0., 0., 0.], points, 0., 0)
    rotated = points[:, [2, 1, 0]] + [.01, .02, .03]
    state = observe([0., 0., .2], rotated, 100., 100_000_000)
    assert state.phase[2] == "TAKEUP"
    # A one-frame deformation spike must not even stop take-up provisionally.
    bent = rotated.copy(); bent[-1, 0] += .001
    state = observe([0., 0., .3], bent, 100., 200_000_000)
    assert state.phase[2] == "TAKEUP"
    state = observe([0., 0., .3], rotated, 100., 300_000_000)
    assert state.phase[2] == "TAKEUP"
    assert state.confirmation_count[2] == 0
    checkpoint = owner.clone_state()
    # Persistent raw support at held motor confirms; no per-frame deformation
    # or model-strain increase is required during confirmation hold.
    observe([0., 0., .3], bent, 100., 400_000_000)
    observe([0., 0., .3], bent, 100., 500_000_000)
    state = observe([0., 0., .3], bent, 100., 600_000_000)
    assert state.phase[2] == "ENGAGED"
    owner.restore_state(checkpoint)
    state = observe([0., 0., .3], rotated, 100., 400_000_000)
    assert state.phase[2] == "TAKEUP"


@pytest.mark.parametrize("reset", ["periodic", "chassis_reversal"])
def test_raw_tendon_anchor_survives_proximal_rebase(reset):
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), response_window_max_s=.3,
        engagement_confirmation_observations=3))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    motor = np.zeros(3)
    def observe(i, amplitude):
        markers = points.copy(); markers[-1, 2] += amplitude
        owner.advance_motor(motor)
        return owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            timestamp_ns=i*100_000_000, observed_marker_points_m=markers)
    for i in range(5):
        observe(i, 0.)
    motor[2] = .1
    observe(5, .0001)
    anchor = owner.distal_marker_anchor.copy()
    for i in range(6, 11):
        motor[2] += .1
        if reset == "chassis_reversal":
            motor[0] = .1 if i % 2 else -.1
        state = observe(i, min(i-4, 5)*.0001)
        np.testing.assert_array_equal(owner.distal_marker_anchor, anchor)
    for i in range(11, 15):
        state = observe(i, .001)
    assert state.phase[2] == "ENGAGED"
    assert len(owner.distal_marker_history) <= 5


@pytest.mark.parametrize("reset", ["gap", "rewind", "homing", "tendon_reversal"])
def test_raw_tendon_reference_does_not_bridge_discontinuities(reset):
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    motor = np.zeros(3)
    def observe(stamp, markers):
        owner.advance_motor(motor)
        return owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            timestamp_ns=stamp, observed_marker_points_m=markers)
    for i in range(5):
        observe(i*100_000_000, points)
    motor[2] = .2
    for i in range(5, 8):
        observe(i*100_000_000, points)
    bent = points.copy(); bent[-1, 2] += .002
    stamp = 800_000_000
    if reset == "gap":
        stamp = 2_000_000_000
    elif reset == "rewind":
        stamp = 50_000_000
    elif reset == "homing":
        owner.reset_observation_history()
    else:
        motor[2] = -.2
    state = observe(stamp, bent)
    assert state.phase[2] != "ENGAGED"
    assert len(owner.distal_marker_history) == 1
    assert owner.confirmation_count[2] == 0


def test_raw_tendon_threshold_validation():
    with pytest.raises(ValueError, match="marker_deformation"):
        BacklashConfig(width_rad=(1., 1., 1.), minimum_distal_marker_deformation_mm=0.)


@pytest.mark.parametrize("external", [False, True])
def test_homing_gap_preserves_reference_but_requires_fresh_signed_samples(external):
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    def observe(stamp, travel, chord):
        motor = np.array([0., 0., travel])
        markers = points.copy(); markers[-1, 2] += chord
        owner.advance_motor(motor, external_motion=external)
        return owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            timestamp_ns=stamp, observed_marker_points_m=markers)
    observe(0, 0., 0.)
    observe(100_000_000, .1, 0.)
    anchor = owner.distal_marker_anchor.copy()
    # The first resumed frame cannot confirm anything, despite large response.
    state = observe(2_000_000_000, .5, .002)
    assert state.phase[2] == "TAKEUP"
    assert owner.confirmation_count[2] == 0
    assert len(owner.distal_marker_history) == 1
    if external:
        np.testing.assert_array_equal(owner.distal_marker_anchor, anchor)
    for k in range(1, 7):
        state = observe(2_000_000_000+k*100_000_000, .5, .002)
    assert (state.phase[2] == "ENGAGED") == external


@pytest.mark.parametrize("break_kind", ["rewind", "reversal", "no_travel", "ownership"])
def test_external_homing_gap_cannot_bridge_transaction_breaks(break_kind):
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    owner.advance_motor(np.zeros(3), external_motion=True)
    owner.advance_motor([0., 0., .1], external_motion=True)
    descriptor = np.arange(6, dtype=float)*.01
    owner._observe_distal_descriptor(descriptor, [0., 0., .1], 100_000_000)
    stamp, travel, external = 2_000_000_000, .5, True
    if break_kind == "rewind":
        stamp = 50_000_000
    elif break_kind == "reversal":
        travel = -.5
    elif break_kind == "no_travel":
        travel = .1
    else:
        external = False
    owner.advance_motor([0., 0., travel], external_motion=external)
    owner._observe_distal_descriptor(descriptor+.002, [0., 0., travel], stamp)
    np.testing.assert_array_equal(owner.distal_marker_anchor, descriptor+.002)
    assert len(owner.distal_marker_history) == 1
    assert owner.confirmation_count[2] == 0


def test_compensated_calibration_tracks_external_chassis_without_changing_homing():
    def run(track):
        owner = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
        for i in range(10):
            motor = np.array([i*.1, 0., 0.])
            owner.advance_motor(motor, external_motion=True, track_external_response=track)
            pose = _pose()
            pose[2, 3] = i*.001
            state = owner.observe_response(motor, pose, _jacobian(), np.ones(6),
                timestamp_ns=i*50_000_000, observed_interface_position_m=pose[:3, 3])
        return owner, state
    plain, plain_state = run(False)
    tracked, state = run(True)
    assert plain_state.phase[0] == "UNKNOWN"
    assert state.phase[0] == "ENGAGED" and state.engaged_direction[0] == 1
    checkpoint = tracked.clone_state()
    assert checkpoint["track_external_response"]
    tracked.restore_state(checkpoint)
    assert tracked.track_external_response
    # Calibration never learns widths or imports an owned transaction budget.
    np.testing.assert_array_equal(tracked.width_positive, plain.width_positive)
    tracked.advance_motor([1., 0., 0.], external_motion=True)
    assert not tracked.track_external_response and tracked.phase[0] == "UNKNOWN"


def test_source_replay_retains_explicit_external_calibration_mode():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
    history = []
    for i in range(3):
        motor = np.array([i*.1, 0., 0.])
        owner.advance_motor(motor, external_motion=True, track_external_response=True)
        history.append([1_000_000_000+i*50_000_000, motor.copy(), owner.clone_state(), False])
    owner.observe_at_timestamp(history, 1_025_000_000, _pose(),
        type("Model", (), {"jacobian": _jacobian(), "state_scale": np.ones(6)})(),
        control_owned=False, observed_interface_position_m=np.zeros(3))
    assert owner.track_external_response and owner.external_motion
    assert owner.phase[0] == "TAKEUP"  # Replay does not reset to ordinary homing UNKNOWN.


def test_external_calibration_cannot_clear_a_failed_shaft():
    owner = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
    owner.advance_motor(np.zeros(3), external_motion=True)
    owner.phase[0] = "FAILED"
    owner.advance_motor([.1, 0., 0.], external_motion=True, track_external_response=True)
    assert owner.phase[0] == "FAILED"


@pytest.mark.parametrize("direction", [-1, 1])
def test_raw_tendon_opposed_deformation_cannot_confirm_new_direction(direction):
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    def observe(step, chord_delta, moving=True):
        motor = np.array([0., 0., direction*(step*.1 if moving else 1.)])
        markers = points.copy()
        markers[-1, 2] += chord_delta
        owner.advance_motor(motor)
        return owner.observe_response(motor, _pose(), -100.*_jacobian(), np.ones(6),
            [direction*100.], [1.], timestamp_ns=step*100_000_000,
            observed_marker_points_m=markers)
    observe(0, 0.)
    observe(1, 0.)
    for step in range(2, 7):
        state = observe(step, -direction*.002)
        assert state.phase[2] == "TAKEUP"
        assert state.confirmation_count[2] == 0
        assert not state.tendon_distal_response_confirmed
    # Genuine sign-consistent response can subsequently confirm, including a
    # stopped provisional hold. J and posterior strain stay deliberately wrong.
    for step in range(7, 13):
        state = observe(step, direction*.002, moving=step < 10)
    assert state.phase[2] == "ENGAGED"


def test_raw_tendon_new_direction_does_not_inherit_past_chord_motion():
    owner = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3))
    points = np.array([[0., 0., 0.], [0., 0., .01],
                       [0., 0., .02], [0., 0., .03]])
    for step in range(5):
        motor = np.zeros(3)
        owner.advance_motor(motor)
        owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            timestamp_ns=step*100_000_000, observed_marker_points_m=points)
    changed = points.copy(); changed[-1, 2] += .002
    for step in range(5, 11):
        motor = np.array([0., 0., (step-4)*.1])
        owner.advance_motor(motor)
        state = owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            timestamp_ns=step*100_000_000, observed_marker_points_m=changed)
        assert state.phase[2] == "TAKEUP"
        assert state.confirmation_count[2] == 0
    # The displacement on the first new-direction image is not causal proof
    # that subsequent same-direction motor travel produced any deformation.
    np.testing.assert_allclose(owner.distal_marker_anchor[2], 32.)


@pytest.mark.parametrize("sign", [-1., 1.])
def test_static_interface_with_opposing_knob_and_chassis_confirms_chassis(sign):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3))
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), _jacobian(), np.ones(6),
                               timestamp_ns=0)
    # Knob +0.5 mm and chassis backward: net interface position stays zero.
    motor = np.array([-sign, 0., -sign*3.])
    estimator.advance_motor(motor)
    first = estimator.observe_response(motor, _pose(), _jacobian(), np.ones(6),
                                        timestamp_ns=100_000_000)
    assert first.phase[0] == "PROVISIONAL"
    assert first.engaged_direction[0] == -sign
    for stamp in (200_000_000, 300_000_000):
        result = estimator.observe_response(motor, _pose(), _jacobian(), np.ones(6),
                                            timestamp_ns=stamp)
    assert result.phase[0] == "ENGAGED"
    assert result.phase[2] == "TAKEUP"  # Interface cancellation is not tendon evidence.


@pytest.mark.parametrize("sign", [-1., 1.])
def test_knob_only_interface_response_cannot_confirm_slack_chassis(sign):
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), _jacobian(), np.ones(6),
                               timestamp_ns=0)
    motor = np.array([sign*.2, 0., sign*3.])
    estimator.advance_motor(motor)
    pose = _pose(*knob_interface_translation_m(motor[2]))
    result = estimator.observe_response(motor, pose, _jacobian(), np.ones(6),
                                        timestamp_ns=100_000_000)
    assert result.phase[0] == "TAKEUP"
    assert result.confirmation_count[0] == 0
    assert result.response_classification[0] == "INCONCLUSIVE"


def test_knob_subtraction_uses_incremental_persistence_and_survives_restore():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 10.), engagement_confirmation_observations=3))
    initial = np.array([12., 0., -7.])
    estimator.advance_motor(initial)
    estimator.observe_response(initial, _pose(), _jacobian(), np.ones(6), timestamp_ns=0)
    for index in (1, 2):
        motor = initial+np.array([-index, 0., -3.*index])
        estimator.advance_motor(motor)
        state = estimator.observe_response(motor, _pose(), _jacobian(), np.ones(6),
                                           timestamp_ns=index*100_000_000)
        assert state.confirmation_count[0] == index
    checkpoint = estimator.clone_state()
    other = BacklashStateEstimator(estimator.config)
    other.restore_state(checkpoint)
    for owner in (estimator, other):
        state = owner.observe_response(motor, _pose(), _jacobian(), np.ones(6),
                                        timestamp_ns=300_000_000)
        assert state.phase[0] == "ENGAGED"
    np.testing.assert_allclose(other.marker_response_anchor, estimator.marker_response_anchor)


def test_interface_input_keeps_direct_knob_despite_tendon_takeup():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    estimator.advance_motor([0., 0., 0.])
    effective = estimator.advance_motor([.2, 0., -.3])
    np.testing.assert_allclose(effective, [0., 0., 0.])
    interface = estimator.interface_motor_angles([.2, 0., -.3])
    np.testing.assert_allclose(interface, [0., 0., -.3])
    interface[0] = 99.
    np.testing.assert_allclose(estimator.effective_motor, [0., 0., 0.])
    assert estimator.phase[0] == "TAKEUP" and estimator.phase[2] == "TAKEUP"
    # Knob travel cannot create chassis reversal when its shaft stays still.
    estimator.advance_motor([.2, 0., .3])
    assert estimator.motion_direction[0] == 1
    assert not estimator.reversal_observed[0]


def test_fixed_knob_conversion_matches_firmware_lead_and_sign():
    np.testing.assert_allclose(knob_interface_translation_m(2*np.pi),
                               [0., 0., -1.190625e-3])
    with pytest.raises(ValueError):
        knob_interface_translation_m(float("nan"))


def test_timed_median_anchor_subtracts_each_paired_knob_coordinate():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 10.)))
    seed = np.array([1., 0., -2.])
    estimator.advance_motor(seed)
    estimator.observe_response(seed, _pose(*knob_interface_translation_m(seed[2])),
                               _jacobian(), np.ones(6), timestamp_ns=0)
    for index in range(1, 6):
        motor = seed+np.array([index*.1, 0., index*.4])
        estimator.advance_motor(motor)
        estimator.observe_response(motor, _pose(*knob_interface_translation_m(motor[2])),
            _jacobian(), np.ones(6), timestamp_ns=index*200_000_000)
    # Raw marker median belongs to older knob positions; the chassis anchor
    # must be the median of already-subtracted, time-paired coordinates.
    np.testing.assert_allclose(estimator.marker_response_anchor, [0., 0., 0.], atol=1e-15)
    assert estimator.phase[0] == "TAKEUP"


def test_recorded_201215_noisy_search_anchor_does_not_handoff():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
    seed = np.array([1.77905208, .22987927, 25.95994808])*1e-3
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), _jacobian(), np.ones(6),
        observed_interface_position_m=seed, timestamp_ns=806_817_681)
    rows = [
        (1_606_820_681, 39808, [1.73656456, .19918282, 25.97323619]),
        (1_639_950_681, 39649, [1.78421731, .23038466, 25.94297007]),
        (1_706_818_681, 39344, [1.78464467, .22987927, 25.95994808]),
        (1_739_952_681, 39344, [1.77905208, .41187590, 25.99868178]),
        (1_806_817_681, 38892, [1.71872426, .14423519, 25.94525926]),
        (1_839_940_681, 38892, [1.79029745, .30564377, 25.97194910]),
        (1_906_809_681, 38404, [1.85818598, .26774395, 25.99684149]),
        (1_939_962_681, 38238, [1.77196483, .25628746, 26.00073256]),
        (2_006_810_681, 37959, [1.72510534, .16206273, 25.95987916]),
    ]
    for stamp, count, position in rows:
        motor = [(count-41000)*np.deg2rad(.045), 0., 0.]
        estimator.advance_motor(motor)
        state = estimator.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            observed_interface_position_m=np.array(position)*1e-3, timestamp_ns=stamp)
        assert state.phase[0] == 'TAKEUP'
        assert state.confirmation_count[0] == 0
    # Only the onset anchor is robustified: current pose/sample is still raw.
    np.testing.assert_allclose(estimator.response_sample_pose[:3, 3],
                               np.array(rows[-1][2])*1e-3)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_robust_search_anchor_preserves_strong_response_and_frozen_hold(sign):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
    def observe(t, motor, x):
        estimator.advance_motor([motor, 0., 0.])
        return estimator.observe_response([motor, 0., 0.], _pose(x),
            _jacobian(), np.ones(6), timestamp_ns=int(t*1e9))
    observe(0., 0., 0.)
    for t, x in [(.7, 0.), (.8, .00002), (.9, -.00002), (1., -.00003)]:
        assert observe(t, sign*.2, x).phase[0] == 'TAKEUP'
    anchor = estimator.marker_response_anchor.copy()
    assert observe(1.1, sign*.4, sign*.0005).phase[0] == 'PROVISIONAL'
    assert observe(1.2, sign*.4, sign*.0005).confirmation_count[0] == 2
    assert observe(1.3, sign*.4, sign*.0005).phase[0] == 'ENGAGED'
    # Successful completion rebases, but the provisional samples used the
    # unchanged past-only anchor and required three distinct observations.
    assert abs(anchor[0]) < .00003


def test_robust_anchor_history_is_bounded_duplicate_safe_and_checkpointed():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(10., 1., 1.)))
    for i in range(20):
        estimator.advance_motor([i*.02, 0., 0.])
        estimator.observe_response([i*.02, 0., 0.], _pose(), _jacobian(),
                                   np.ones(6), timestamp_ns=i*100_000_000)
    assert len(estimator.marker_anchor_history) == 5
    checkpoint = estimator.clone_state()
    estimator.observe_response([.38, 0., 0.], _pose(.1), _jacobian(),
                               np.ones(6), timestamp_ns=1_900_000_000)
    np.testing.assert_array_equal(estimator.marker_anchor_history,
                                  checkpoint['marker_anchor_history'])
    estimator.reset_observation_history()
    assert estimator.marker_anchor_history == []
    estimator.restore_state(checkpoint)
    np.testing.assert_array_equal(estimator.marker_response_anchor,
                                  checkpoint['marker_response_anchor'])
    estimator.observe_response([.38, 0., 0.], _pose(.2), _jacobian(),
                               np.ones(6), timestamp_ns=100)
    assert len(estimator.marker_anchor_history) == 1
    np.testing.assert_array_equal(estimator.marker_response_anchor, [.2, 0., 0.])


@pytest.mark.parametrize("sign", [-1., 1.])
def test_anisotropic_onset_remains_supported_by_identical_observations(sign):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
    covariance = np.array(estimator.config.marker_increment_covariance_m2).reshape(3, 3)
    displacement = sign*np.array([.03370026, .17506855, .09669736])*1e-3
    direction = displacement/np.linalg.norm(displacement)
    assert displacement@np.linalg.solve(covariance, displacement) > 16.
    # This same displacement failed the old Euclidean persistence metric.
    assert np.linalg.norm(displacement) < 3*np.sqrt(direction@covariance@direction)
    estimator.observe([0., 0., 0.], _pose(), _jacobian(), np.ones(6))
    estimator.advance_motor([sign, 0., 0.])
    for expected in ["PROVISIONAL", "PROVISIONAL", "ENGAGED"]:
        state = estimator.observe_response([sign, 0., 0.], _pose(*displacement),
                                           _jacobian(), np.ones(6))
        assert state.phase[0] == expected


def test_recorded_200540_short_window_does_not_promote_unresolved_onset():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
    anchor = np.array([1.8384451978, .2403537364, 25.4476554692])*1e-3
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), _jacobian(), np.ones(6),
                               observed_interface_position_m=anchor)
    rows = [
        (33330, [1.8721454544, .4154222843, 25.5443528295], "TAKEUP", 0),
        (33551, [1.8722494133, .4933985765, 25.5072824657], "TAKEUP", 0),
        (33814, [1.8635602901, .5085226730, 25.4877917469], "TAKEUP", 0),
        (33824, [1.8648428377, .4621341650, 25.4895742983], "TAKEUP", 0),
        (33824, [1.8270352157, .4418006865, 25.5730785429], "TAKEUP", 0),
        (33824, [1.8001475837, .4018632462, 25.5455374718], "TAKEUP", 0)]
    for count, marker_mm, phase, confirmations in rows:
        motor = [(count-32427)*np.deg2rad(.045), 0., 0.]
        estimator.advance_motor(motor)
        state = estimator.observe_response(motor, _pose(), _jacobian(), np.ones(6),
            observed_interface_position_m=np.array(marker_mm)*1e-3)
        assert state.phase[0] == phase
        assert state.confirmation_count[0] == confirmations


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("jacobian_multiplier", [0., -100., .001, 100.])
def test_engagement_is_independent_of_jacobian_prediction(direction, jacobian_multiplier):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), engagement_confirmation_observations=3))
    jacobian = _jacobian()*jacobian_multiplier
    scale = np.array([.02, .03, .09, .0002, .0003, .0007])
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), jacobian, scale,
                              observed_interface_position_m=[0., 0., 0.])
    estimator.advance_motor([.4*direction, 0., 0.])
    # Measured response deliberately differs from the predicted x direction.
    for expected in ["TAKEUP", "PROVISIONAL", "PROVISIONAL", "ENGAGED"]:
        state = estimator.observe_response(
            [.4*direction, 0., 0.], _pose(), jacobian, scale,
            observed_interface_position_m=[0., .0005*direction, 0.])
        assert state.phase[0] == expected
    assert state.confirmation_count[0] == 3


def test_recorded_held_cartesian_response_survives_wrong_predicted_direction():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
    jacobian = _jacobian()*.001
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], _pose(), jacobian, np.ones(6),
                              observed_interface_position_m=[0., 0., 0.])
    estimator.advance_motor([-1., 0., 0.])
    # 191351 repeat06: handoff displacement and median held displacement
    # relative to the raw-marker anchor. The old SE(3)/J cosine fell below .5.
    for position, expected in [
            ([.000188678, .000071320, -.000153985], "TAKEUP"),
            ([.000173340, .000012150, -.000127360], "PROVISIONAL"),
            ([.000173340, .000012150, -.000127360], "PROVISIONAL"),
            ([.000173340, .000012150, -.000127360], "ENGAGED")]:
        state = estimator.observe_response([-1., 0., 0.], _pose(), jacobian,
            np.ones(6), observed_interface_position_m=position)
        assert state.phase[0] == expected


@pytest.mark.parametrize("direction", [-1, 1])
def test_stationary_interface_marker_prevents_posterior_only_handoff(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), engagement_confirmation_observations=3))
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response(
        [0., 0., 0.], _pose(), _jacobian(), np.ones(6),
        observed_interface_position_m=np.zeros(3), timestamp_ns=100)
    for index, noise in enumerate([.00002, -.00002, .00003, 0.]):
        motor = [.4*direction, 0., 0.]
        estimator.advance_motor(motor)
        pose = _pose(.2*direction)
        original = pose.copy()
        state = estimator.observe_response(
            motor, pose, _jacobian(), np.ones(6),
            observed_interface_position_m=[noise, 0., 0.],
            timestamp_ns=200+index)
        assert state.phase[0] == "TAKEUP"
        assert state.confirmation_count[0] == 0
        np.testing.assert_array_equal(pose, original)


@pytest.mark.parametrize("direction", [-1, 1])
def test_measured_interface_response_confirms_despite_fitted_translation_bias(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), engagement_confirmation_observations=3))
    def observe(motor, position, stamp):
        estimator.advance_motor([motor, 0., 0.])
        return estimator.observe_response(
            [motor, 0., 0.], _pose(), _jacobian(), np.ones(6),
            observed_interface_position_m=[position, 0., 0.], timestamp_ns=stamp)
    observe(0., 0., 100)
    assert observe(.4*direction, .2*direction, 150).phase[0] == "TAKEUP"
    first = observe(.4*direction, .2*direction, 200)
    assert first.phase[0] == "PROVISIONAL"
    checkpoint = estimator.clone_state()
    duplicate = observe(.4*direction, .2*direction, 200)
    assert duplicate.confirmation_count[0] == 1
    estimator.reset_observation_history()
    estimator.restore_state(checkpoint)
    second = observe(.4*direction, .2*direction, 300)
    assert second.confirmation_count[0] == 2
    final = observe(.4*direction, .2*direction, 400)
    assert final.phase[0] == "ENGAGED"


def test_interface_marker_spike_cannot_confirm_while_posterior_offset_persists():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), engagement_confirmation_observations=3))
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response(
        [0., 0., 0.], _pose(), _jacobian(), np.ones(6),
        observed_interface_position_m=[0., 0., 0.], timestamp_ns=100)
    estimator.advance_motor([.4, 0., 0.])
    first = estimator.observe_response(
        [.4, 0., 0.], _pose(.2), _jacobian(), np.ones(6),
        observed_interface_position_m=[.2, 0., 0.], timestamp_ns=200)
    assert first.phase[0] == "TAKEUP"
    for stamp in [300, 400]:
        state = estimator.observe_response(
            [.4, 0., 0.], _pose(.2), _jacobian(), np.ones(6),
            observed_interface_position_m=[0., 0., 0.], timestamp_ns=stamp)
        assert state.phase[0] == "TAKEUP"
        assert state.confirmation_count[0] == 0
        assert state.response_classification[0] == "INCONCLUSIVE"


@pytest.mark.parametrize("position", [[0., 0.], [0., float("nan"), 0.]])
def test_invalid_observed_interface_position_is_rejected(position):
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
    with pytest.raises(ValueError, match="observed_interface_position_m"):
        estimator.observe_response(
            [0., 0., 0.], _pose(), _jacobian(), np.ones(6),
            observed_interface_position_m=position)


def test_recorded_repeat06_posterior_handoff_lacks_cartesian_support():
    # Exact source-paired 9.749138/10.349023 s observations from the second
    # repeat06 circle run. This conditions on recorded orientations/J, not a
    # filter or plant replay; only the response evidence translation changes.
    anchor = np.array([
        [-.6971526742, -.6954687834, .1740849316, .001916829147],
        [.7167904377, -.6808655858, .1504660845, .000290412019],
        [.0138841383, .2296785861, .9731681347, .027528775856],
        [0., 0., 0., 1.]])
    handoff = np.array([
        [-.6987984180, -.6934794784, .1754171401, .001926577999],
        [.7151858807, -.6821029782, .1524839699, .000269842189],
        [.0139082130, .2320096940, .9726145267, .027439083904],
        [0., 0., 0., 1.]])
    jacobian = np.array([
        [-9.04974804e-5, -3.21549072e-4, -5.36031742e-3],
        [-4.61077288e-5, .0105971051, 3.09661060e-4],
        [-.00166494423, .0610115193, 5.74825332e-4],
        [-7.73456031e-6, 7.97884713e-5, 5.23991093e-6],
        [6.23353626e-5, -1.40960685e-6, 1.44607275e-5],
        [4.23419173e-4, 4.19995786e-6, -1.36507224e-4]])
    scale = np.array([.02046605295, .02932424205, .09348398599,
                      .000265571466, .000199031829, .000739069325])
    markers = [[.001890404499, .000275148661, .027457254007],
               [.001927350997, .000300294341, .027457820252]]
    outcomes = []
    for use_marker in [False, True]:
        estimator = BacklashStateEstimator(BacklashConfig(
            width_rad=(10., 1., 1.), engagement_confirmation_observations=3))
        for motor, pose, position, stamp in [
                (0., anchor, markers[0], 100_000_000),
                (-2.2760838775, handoff, markers[1], 699_885_000)]:
            estimator.advance_motor([motor, 0., 0.])
            state = estimator.observe_response(
                [motor, 0., 0.], pose, jacobian, scale, timestamp_ns=stamp,
                observed_interface_position_m=position if use_marker else None)
        outcomes.append(state)
    assert outcomes[0].phase[0] == "TAKEUP"
    assert outcomes[0].inferred_transmitted_increment_rad[0] == pytest.approx(-.206769, abs=1e-6)
    assert outcomes[1].phase[0] == "TAKEUP"
    assert outcomes[1].confirmation_count[0] == 0
    assert outcomes[1].response_classification[0] == "INCONCLUSIVE"


def test_interface_marker_translation_preserves_rotation_response():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
    jacobian = np.zeros((6, 3))
    jacobian[2, 1] = 1.
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response(
        [0., 0., 0.], _pose(), jacobian, np.ones(6),
        observed_interface_position_m=[0., 0., 0.])
    pose = _pose(.5)
    angle = .2
    pose[:2, :2] = [[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]]
    estimator.advance_motor([0., .4, 0.])
    state = estimator.observe_response(
        [0., .4, 0.], pose, jacobian, np.ones(6),
        observed_interface_position_m=[0., 0., 0.])
    assert state.phase[1] == "ENGAGED"
    assert state.inferred_transmitted_increment_rad[1] == pytest.approx(.2)


def test_recorded_direct_marker_handoff_is_below_directional_noise_floor():
    # 1.302036 -> 1.335419 s in repeat06_20261006T190416967264Z.
    # Conditional recorded-input regression, not a closed-loop replay.
    anchor = np.array([
        [-.6994482279, -.6886680126, .1910848767, .002038216684],
        [.7145121694, -.6797022820, .1657790989, .000433463312],
        [.0157141741, .2524848878, .9674771428, .024428825825],
        [0., 0., 0., 1.]])
    jacobian = np.array([
        [-9.04974804e-5, -3.21549072e-4, -5.36031742e-3],
        [-4.61077288e-5, .0105971051, 3.09661060e-4],
        [-.00166494423, .0610115193, 5.74825332e-4],
        [-7.73456031e-6, 7.97884713e-5, 5.23991093e-6],
        [6.23353626e-5, -1.40960685e-6, 1.44607275e-5],
        [4.23419173e-4, 4.19995786e-6, -1.36507224e-4]])
    scale = np.array([.02046605295, .02932424205, .09348398599,
                      .000265571466, .000199031829, .000739069325])
    positions = [[.002009094460, .000438562507, .024321008474],
                 [.001946616103, .000422597717, .024359498173]]
    for resolved_noise, expected in [(False, "PROVISIONAL"), (True, "TAKEUP")]:
        config = dict(width_rad=(10., 1., 1.), engagement_confirmation_observations=3)
        if not resolved_noise:
            config['marker_increment_covariance_m2'] = tuple((np.eye(3)*1e-24).flat)
        estimator = BacklashStateEstimator(BacklashConfig(**config))
        for motor, position, stamp in [(0., positions[0], 100_000_000),
                                      (.117024326346, positions[1], 133_383_000)]:
            estimator.advance_motor([motor, 0., 0.])
            state = estimator.observe_response(
                [motor, 0., 0.], anchor, jacobian, scale, timestamp_ns=stamp,
                observed_interface_position_m=position)
        assert state.phase[0] == "TAKEUP"
        state = estimator.observe_response(
            [motor, 0., 0.], anchor, jacobian, scale, timestamp_ns=166_766_000,
            observed_interface_position_m=positions[1])
        assert state.phase[0] == expected
        assert state.inferred_transmitted_increment_rad[0] == pytest.approx(.117024326346)
        if resolved_noise:
            assert state.response_classification[0] == "INCONCLUSIVE"
            assert state.confirmation_count[0] == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_resolved_submillimeter_marker_response_persists_through_hold(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), engagement_confirmation_observations=3))
    jacobian = np.zeros((6, 3))
    jacobian[5, 0] = .0005
    def observe(motor, z, stamp):
        estimator.advance_motor([motor, 0., 0.])
        return estimator.observe_response(
            [motor, 0., 0.], _pose(), jacobian, np.ones(6),
            observed_interface_position_m=[0., 0., z], timestamp_ns=stamp)
    observe(0., 0., 100)
    # 0.03 mm directed noise cannot stop the burst, despite enough encoder travel.
    assert observe(.2*direction, .00003*direction, 200).phase[0] == 'TAKEUP'
    assert observe(.4*direction, .00012*direction, 250).phase[0] == 'TAKEUP'
    assert observe(.4*direction, .00012*direction, 300).phase[0] == 'PROVISIONAL'
    checkpoint = estimator.clone_state()
    assert observe(.4*direction, .00012*direction, 300).confirmation_count[0] == 1
    estimator.restore_state(checkpoint)
    assert observe(.4*direction, .00011*direction, 400).confirmation_count[0] == 2
    assert observe(.4*direction, .00013*direction, 500).phase[0] == 'ENGAGED'


def test_marker_floor_uses_world_covariance_under_rotated_anchor_and_scale():
    # Local x maps to noisy world y: 0.15 mm is below its 0.282 mm 3-sigma floor.
    pose = _pose()
    pose[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    jacobian = _jacobian()*.001
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1., 1., 1.)))
    scale = np.array([1., 1., 1., .001, .002, .003])
    estimator.advance_motor([0., 0., 0.])
    estimator.observe_response([0., 0., 0.], pose, jacobian, scale,
                              observed_interface_position_m=[0., 0., 0.])
    estimator.advance_motor([.4, 0., 0.])
    state = estimator.observe_response([.4, 0., 0.], pose, jacobian, scale,
                                      observed_interface_position_m=[0., .00015, 0.])
    assert state.phase[0] == 'TAKEUP'
    state = estimator.observe_response([.4, 0., 0.], pose, jacobian, scale,
                                      observed_interface_position_m=[0., .00045, 0.])
    assert state.phase[0] == 'TAKEUP'
    state = estimator.observe_response([.4, 0., 0.], pose, jacobian, scale,
                                      observed_interface_position_m=[0., .00045, 0.])
    assert state.phase[0] == 'ENGAGED'


@pytest.mark.parametrize('covariance', [tuple(np.zeros(9)), tuple(range(9)),
                                       (1.,), tuple([float('nan')]*9)])
def test_invalid_marker_increment_covariance_rejected(covariance):
    with pytest.raises(ValueError, match='marker_increment_covariance_m2'):
        BacklashConfig(width_rad=(1., 1., 1.), marker_increment_covariance_m2=covariance)


def test_insertion_takeup_suspends_tendon_gain_evidence():
    from unittest.mock import Mock
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1, 0, 1)))
    estimator.phase[:] = ["TAKEUP", "ENGAGED", "ENGAGED"]
    estimator.engaged_direction[2] = -1
    estimator.engaged_gain_estimator.observe = Mock()
    estimator.observe_response(
        np.zeros(3), _pose(), _jacobian(), np.ones(6),
        nominal_distal_lambda=1., observed_distal_lambda=1.)
    call = estimator.engaged_gain_estimator.observe.call_args
    assert call.kwargs["engaged"] is False
    assert call.kwargs["motor_position"] == 0.


def test_external_home_motion_invalidates_transaction_not_learned_widths():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1, 2, 3)))
    estimator.advance_motor([0, 0, 0])
    estimator.phase[0] = "FAILED"
    estimator.phase[2] = "ENGAGED"
    estimator.engaged_direction[2] = 1
    estimator.width_positive[0] = 1.2
    before = estimator.width_positive.copy()
    coordinate = estimator.advance_motor([.5, 0, 0], external_motion=True)
    assert estimator.phase == ["UNKNOWN", "UNKNOWN", "UNKNOWN"]
    np.testing.assert_array_equal(before, estimator.width_positive)
    assert coordinate[0] == pytest.approx(.5)
    assert estimator.confidence[0] == 0
    assert estimator.last_evidence_timestamp_ns[0] == 0
    assert estimator.snapshot().remaining_upper_rad[0] > 0
    estimator.advance_motor([.6, 0, 0])
    assert estimator.phase[0] == "TAKEUP"
    assert estimator.confirmation_count[0] == 0


def test_external_static_sample_does_not_clear_active_failure():
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1, 2, 3)))
    estimator.advance_motor([0, 0, 0])
    estimator.advance_motor([.2, 0, 0])
    estimator.phase[0] = "FAILED"
    estimator.advance_motor([.2, 0, 0], external_motion=True)
    assert estimator.phase[0] == "FAILED"
    estimator.advance_motor([.4, 0, 0])
    assert estimator.phase[0] == "FAILED"


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


def test_joint_fit_does_not_establish_simultaneous_axis_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))

    mixed = estimator.observe(
        [.4, -.3, 0], _pose(.4, -.3), _jacobian(), np.ones(6))

    assert mixed.phase[:2] == ("TAKEUP", "TAKEUP")
    assert mixed.engaged_direction.tolist() == [0, 0, 0]
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


def test_response_accumulates_when_raw_motor_is_ready_but_shape_is_not():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    for motor, shape in [(.2, .00002), (.4, .00004)]:
        state = estimator.observe([motor, 0, 0], _pose(shape),
                                  _jacobian(), np.ones(6))
        assert state.phase[0] == "TAKEUP"
        np.testing.assert_array_equal(estimator.previous_response_motor, [0, 0, 0])
    state = estimator.observe([.6, 0, 0], _pose(.0003), _jacobian(), np.ones(6))
    assert state.phase[0] == "ENGAGED"


def test_transient_shape_spike_is_not_confirmed_by_stationary_encoders():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    first = estimator.observe([.4, 0, 0], _pose(.1), _jacobian(), np.ones(6))
    assert first.phase[0] == "PROVISIONAL"
    for _ in range(4):
        state = estimator.observe_response([.4, 0, 0], _pose(),
                                           _jacobian(), np.ones(6))
        assert state.phase[0] == "PROVISIONAL"
        assert state.confirmation_count[0] == 1


@pytest.mark.parametrize("elapsed_ns", [1_000_000_000, -1])
def test_response_window_expires_or_resets_on_clock_rewind(elapsed_ns):
    estimator = BacklashStateEstimator(BacklashConfig(width_rad=(1, 1, 1)))
    estimator.observe_response([0, 0, 0], _pose(), _jacobian(), np.ones(6),
                               timestamp_ns=10)
    estimator.observe_response([.4, 0, 0], _pose(.02), _jacobian(), np.ones(6),
                               timestamp_ns=10+elapsed_ns)
    np.testing.assert_array_equal(estimator.previous_response_motor, [.4, 0, 0])
    assert estimator.snapshot().phase[0] == "UNKNOWN"


@pytest.mark.parametrize("coast", [False, True])
@pytest.mark.parametrize("persistent", [False, True])
def test_late_first_response_survives_search_expiry_during_hold(coast, persistent):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), minimum_transmitted_increment_rad=.05,
        engagement_confirmation_observations=3, response_window_max_s=1.0))

    def observe(seconds, motor, shape):
        estimator.advance_motor([motor, 0, 0])
        return estimator.observe_response(
            [motor, 0, 0], _pose(shape), _jacobian(), np.ones(6),
            timestamp_ns=int(seconds*1e9))

    observe(0, 0, 0)
    first = observe(.9, .4, .1)
    assert first.phase[0] == "PROVISIONAL"
    motor = .42 if coast else .4
    shape = .1 if persistent else 0
    observe(1.01, motor, shape)
    observe(1.1, motor, shape)
    final = observe(1.2, motor, shape)
    np.testing.assert_array_equal(estimator.previous_response_motor,
                                  [motor, 0, 0] if persistent else [0, 0, 0])
    assert final.phase[0] == ("ENGAGED" if persistent else "PROVISIONAL")
    if persistent:
        assert final.confirmation_count[0] >= 3
        assert final.engagement_anchor_motor_rad[0] == pytest.approx(.4)
    else:
        assert final.confirmation_count[0] == 1
        assert final.response_classification[0] == "INCONCLUSIVE"


@pytest.mark.parametrize("invalidate", [False, True])
def test_provisional_window_still_resets_on_rewind_or_invalidation(invalidate):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), engagement_confirmation_observations=3))
    estimator.advance_motor([0, 0, 0])
    estimator.observe_response([0, 0, 0], _pose(), _jacobian(), np.ones(6),
                               timestamp_ns=100)
    estimator.advance_motor([.4, 0, 0])
    first = estimator.observe_response([.4, 0, 0], _pose(.2), _jacobian(),
                                       np.ones(6), timestamp_ns=200)
    assert first.phase[0] == "PROVISIONAL"
    if invalidate:
        estimator.reset_observation_history()
    reset = estimator.observe_response([.4, 0, 0], _pose(.2), _jacobian(),
                                       np.ones(6), timestamp_ns=150 if not invalidate else 300)
    assert reset.phase[0] == "PROVISIONAL"
    assert reset.confirmation_count[0] == 1
    assert reset.response_classification[0] == "NOT_EVALUATED"


def test_duplicate_observation_does_not_confirm_provisional_engagement():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), engagement_confirmation_observations=3))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    estimator.observe([.4, 0, 0], _pose(.2), _jacobian(), np.ones(6))
    for _ in range(3):
        estimator.observe_response([.4, 0, 0], _pose(.2), _jacobian(),
                                   np.ones(6), timestamp_ns=20)
    assert estimator.snapshot().phase[0] == "PROVISIONAL"
    assert estimator.snapshot().confirmation_count[0] == 2


def test_ambiguous_coupled_window_rebases_before_isolated_response():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    first = estimator.observe([.2, .2, 0], _pose(.02, .2),
                              _jacobian(), np.ones(6))
    assert first.phase[:2] == ("TAKEUP", "TAKEUP")
    second = estimator.observe([.4, .4, 0], _pose(.04, .4),
                               _jacobian(), np.ones(6))
    assert second.phase[0] == "TAKEUP"
    third = estimator.observe([.6, .4, 0], _pose(.06, .4),
                              _jacobian(), np.ones(6))
    assert third.phase[:2] == ("ENGAGED", "TAKEUP")
    assert third.inferred_transmitted_increment_rad[0] == pytest.approx(.02)


def test_response_window_checkpoint_restores_coherent_evidence():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1, 1, 1), minimum_transmitted_increment_rad=.05))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    estimator.observe([.2, 0, 0], _pose(.02), _jacobian(), np.ones(6))
    checkpoint = estimator.clone_state()
    estimator.reset_observation_history()
    estimator.restore_state(checkpoint)
    result = estimator.observe([.4, 0, 0], _pose(.06), _jacobian(), np.ones(6))
    assert result.phase[0] == "ENGAGED"


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_response_window_duration_must_be_positive_finite(value):
    with pytest.raises(ValueError, match="response_window_max_s"):
        BacklashConfig(width_rad=(1, 1, 1), response_window_max_s=value)


def test_stopped_subthreshold_shaft_does_not_block_ready_shaft():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1.0, 1.0, 1.0),
        minimum_motor_increment_rad=.01,
        minimum_transmitted_increment_rad=.10))
    mode = np.array([1., 0., 0.])
    _observe_shape(estimator, [0, 0, 0], _pose(), _jacobian(), np.ones(6),
                      np.zeros(3), mode)

    # Axis 1 moved enough to enter the response window, but stopped just
    # below its own inference floor. Axis 2 then develops a fully observable
    # response. The inactive residue must not hold the entire joint window.
    _observe_shape(estimator, [0, .09, -.04], _pose(z=-.04), _jacobian(), np.ones(6),
                      np.zeros(3), mode)
    ready = _observe_shape(estimator,
        [0, .09, -.15], _pose(z=-.15), _jacobian(), np.ones(6),
        np.array([.1, 0., 0.]), mode)

    assert ready.phase[1] == "TAKEUP"
    assert ready.phase[2] == "ENGAGED"
    assert ready.inferred_transmitted_increment_rad[1] == 0.0
    assert ready.inferred_transmitted_increment_rad[2] == pytest.approx(
        -.15, abs=.01)


@pytest.mark.parametrize("axes", [(0,), (1,), (0, 1)])
@pytest.mark.parametrize("shape_response", [False, True])
def test_tendon_shape_evidence_is_separate_from_interface_axis_ambiguity(axes, shape_response):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), minimum_distal_bending_increment=.05))
    mode = np.array([1., 0., 0.])
    _observe_shape(estimator, [0., 0., 0.], _pose(), _jacobian(), np.ones(6),
                      np.zeros(3), mode)
    motor = np.zeros(3)
    motor[list(axes)] = .4
    motor[2] = -.4
    pose = _pose(.001)
    angle = .01
    pose[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.],
                        [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]]
    state = _observe_shape(estimator, motor, pose, _jacobian(), np.ones(6),
                              np.array([.1 if shape_response else 0., 0., 0.]), mode)
    assert state.phase[2] == ("ENGAGED" if shape_response else "TAKEUP")
    for axis in axes:
        assert state.phase[axis] == ("ENGAGED" if len(axes) == 1 else "TAKEUP")


@pytest.mark.parametrize("distal_enabled", [False, True])
def test_interface_motion_alone_never_confirms_tendon(distal_enabled):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), distal_confirmation_enabled=distal_enabled))
    estimator.observe([0., 0., 0.], _pose(), _jacobian(), np.ones(6))
    state = estimator.observe([0., 0., -.4], _pose(z=-.001),
                              _jacobian(), np.ones(6))
    assert state.phase[2] == "TAKEUP"
    assert state.confirmation_count[2] == 0


def test_tendon_only_shape_response_accumulates_without_rebasing():
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), minimum_distal_bending_increment=.05))
    mode = np.array([1., 0., 0.])
    _observe_shape(estimator, [0., 0., 0.], _pose(), _jacobian(), np.ones(6), np.zeros(3), mode)
    for motor, bend, expected in [(-.2, .02, "TAKEUP"), (-.4, .04, "TAKEUP"),
                                  (-.6, .06, "ENGAGED")]:
        state = _observe_shape(estimator, [0., 0., motor], _pose(), _jacobian(), np.ones(6),
                                  np.array([bend, 0., 0.]), mode)
        assert state.phase[2] == expected


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

    assert mixed.phase[:2] == ("TAKEUP", "TAKEUP")
    assert mixed.engaged_direction.tolist() == [0, 0, 0]
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
        [.4, 0, 0], _pose(.00001), _jacobian(), scale)

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

    first = estimator.observe([.4, 0, 0], _pose(.001), _jacobian(), scale)
    second = estimator.observe([.8, 0, 0], _pose(.002), _jacobian(), scale)
    small_same_direction = estimator.observe(
        [1.2, 0, 0], _pose(.00202), _jacobian(), scale)
    accepted_noop = estimator.observe(
        [1.6, 0, 0], _pose(.00202), _jacobian(), scale)
    confirmed = estimator.observe(
        [1.8, 0, 0], _pose(.00302), _jacobian(), scale)

    assert first.phase[0] == "PROVISIONAL"
    assert second.phase[0] == "PROVISIONAL"
    assert second.confirmation_count[0] == 2
    assert small_same_direction.phase[0] == "PROVISIONAL"
    assert small_same_direction.response_classification[0] == "INCONCLUSIVE"
    assert accepted_noop.phase[0] == "PROVISIONAL"
    assert accepted_noop.response_classification[0] == "INCONCLUSIVE"
    assert not accepted_noop.failed[0]
    # Missing shape support breaks persistence but never restarts a burst.
    assert confirmed.phase[0] == "PROVISIONAL"
    estimator.observe([2.0, 0, 0], _pose(.00402), _jacobian(), scale)
    persistent = estimator.observe([2.2, 0, 0], _pose(.00502), _jacobian(), scale)
    assert persistent.phase[0] == "ENGAGED"
    assert persistent.confirmation_count[0] == 3


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


@pytest.mark.parametrize("direction", [-1., 1.])
def test_explicit_tendon_budget_allows_long_takeup_but_remains_bounded(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), tendon_unconfirmed_travel_limit_rad=4.))
    estimator.observe([0, 0, 0], _pose(), _jacobian(), np.ones(6))
    snapshot = estimator.observe([0, 0, direction*2.], _pose(), _jacobian(), np.ones(6))
    assert snapshot.phase[2] == "TAKEUP"
    assert snapshot.engaged_direction[2] == 0
    snapshot = estimator.observe([0, 0, direction*4.1], _pose(), _jacobian(), np.ones(6))
    assert snapshot.phase[2] == "FAILED"


def test_explicit_tendon_budget_does_not_change_shaft_budgets_or_widths():
    config = BacklashConfig(width_rad=(1., 2., 3.), tendon_unconfirmed_travel_limit_rad=10.)
    assert config.unconfirmed_travel_limit_rad(1., axis=0) == 1.5
    assert config.unconfirmed_travel_limit_rad(2., axis=1) == 3.
    assert config.unconfirmed_travel_limit_rad(3., axis=2) == 10.
    estimator = BacklashStateEstimator(config)
    np.testing.assert_array_equal(estimator.snapshot().width_rad, [1., 2., 3.])


@pytest.mark.parametrize("value", [-1., float("nan"), float("inf")])
def test_explicit_tendon_budget_rejects_invalid_values(value):
    with pytest.raises(ValueError, match="tendon_unconfirmed_travel_limit_rad"):
        BacklashConfig(width_rad=(1., 1., 1.), tendon_unconfirmed_travel_limit_rad=value)


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


@pytest.mark.parametrize('direction', [[1, 0, -1], [-1, 0, 1],
                                       [1, 0, 1], [-1, 0, -1]])
def test_transaction_coupled_low_speed_directions_survive_floors(direction):
    contract = load_hardware_contract(LIMITS, "imricor_test")
    arbiter = TakeupTransactionArbiter(
        BacklashFeedforwardCompensator(BacklashConfig(
            width_rad=(1.0, 1.0, 1.0),
            takeup_velocity=(1.0, 1.0, 0.5))))
    home = np.array([20, 0, 7, 0, 0, 0], dtype=float)

    assert arbiter.physical_direction_is_feasible(0, 1, home, contract)
    assert arbiter.physical_direction_is_feasible(2, -1, home, contract)
    assert arbiter.physical_direction_vector_is_feasible(
        direction, home, contract)
    command, requested, realized, blocked, leakage = arbiter._project_takeup(
        home, contract, np.array(direction) != 0, np.array(direction))
    assert not np.any(blocked | leakage)
    np.testing.assert_array_equal(np.sign(realized), direction)
    again = contract.project_velocity(command, home)
    np.testing.assert_allclose(again.motor_radians_per_second[:3], realized)


def test_low_speed_takeup_preserves_explicit_zero_and_disabled_axis():
    from dataclasses import replace
    contract = load_hardware_contract(LIMITS, "imricor_test")
    contract = replace(contract, velocity_min=np.array([2., 0., 2., 0., 0., 0.]),
                       velocity_max=np.array([10., 0., 4.5, 4., 25., 25.]))
    compensator = BacklashFeedforwardCompensator(BacklashConfig(
        width_rad=(1., 1., 1.), takeup_velocity=(0., 2.5, .5)))
    rates = compensator.motor_radians_per_second(contract)
    assert rates[0] == rates[1] == 0.
    assert rates[2] > 0.


def test_recorded_xz_home_unknown_starts_acquisition_without_false_saturation():
    contract = load_hardware_contract(LIMITS, "imricor_test")
    config = BacklashConfig(width_rad=(1., 1., 1.),
                            takeup_velocity=(1., 2.5, .5))
    owner = BacklashStateEstimator(config)
    unknown = owner.snapshot()
    arbiter = TakeupTransactionArbiter(BacklashFeedforwardCompensator(config))
    position = np.array([19.995351791381836, 0., -.0007441406487487257,
                         0., -.013500001281499863, .013500001281499863])
    decision = arbiter.begin([4., 0., 2., 0., 0., 0.], position, contract, unknown)
    assert decision.state == "TAKEUP_ACTIVE"
    assert not decision.execute_plan
    assert not np.any(decision.saturated_mask)
    np.testing.assert_array_equal(np.sign(decision.realized_motor_radians_per_second),
                                  [1, 0, -1])
    # Starting an evidence-acquisition attempt is not engagement confirmation.
    assert owner.snapshot().phase == ("UNKNOWN", "UNKNOWN", "UNKNOWN")
    np.testing.assert_array_equal(owner.snapshot().confirmation_count, 0)


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
        / "legacy/causal_v2_fixed_hardware.yaml").read_text())
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


def test_recorded_marker_excursion_does_not_trigger_confirmation_hold():
    # Point 2, static-v171 trial 20261007T172112691016Z, 6.173--6.539 s.
    # Paired chassis marker = raw marker 0 minus direct knob translation.
    # Conditional recorded-input replay, not a simulated closed-loop outcome.
    samples = [
        (27.1724202591, [.00158145034, .000159718737, .01932738756]),
        (26.7380950747, [.00157783797, .000201053480, .01926676033]),
        (26.6917565831, [.00154318940, .000141454017, .01928014157]),
        (26.3383274095, [.00153012096, -.000027560329, .01923103107]),
        (26.1262699054, [.00155782793, .000051929972, .01927119715]),
        (25.9644778838, [.00161675189, .000198469701, .01927468030]),
        (25.9377743462, [.00156515569, .000232425125, .01929002849]),
        (25.9377743462, [.00160613167, .000149529573, .01927170751]),
    ]
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(2.3857563904, 0., 0.),
        engagement_confirmation_observations=3))
    for index, (motor, marker) in enumerate(samples):
        estimator.advance_motor([motor, 0., 0.])
        state = estimator.observe_response(
            [motor, 0., 0.], _pose(), np.zeros((6, 3)), np.ones(6),
            timestamp_ns=index*50_000_000,
            observed_interface_position_m=marker)
        assert state.phase[0] not in ("PROVISIONAL", "ENGAGED", "FAILED")
        assert state.confirmation_count[0] == 0


@pytest.mark.parametrize("direction", [-1, 1])
def test_robust_marker_onset_accepts_persistent_response_without_jacobian(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 0., 0.), engagement_confirmation_observations=3))
    def observe(motor, z, stamp):
        estimator.advance_motor([motor*direction, 0., 0.])
        return estimator.observe_response(
            [motor*direction, 0., 0.], _pose(), np.zeros((6, 3)), np.ones(6),
            timestamp_ns=stamp*50_000_000,
            observed_interface_position_m=[0., 0., z*direction])
    observe(0., 0., 0)
    for index in range(1, 4):
        assert observe(index*.2, 0., index).phase[0] == "TAKEUP"
    # A single resolved raw frame in a mature window is not sustained onset.
    assert observe(.8, .0003, 4).phase[0] == "TAKEUP"
    first = observe(1., .0003, 5)
    assert first.phase[0] == "PROVISIONAL"
    assert first.confirmation_count[0] == 1
    # Fresh, held observations still confirm; no extra travel is required.
    assert observe(1., .0003, 6).confirmation_count[0] == 2
    assert observe(1., .0003, 7).phase[0] == "ENGAGED"


@pytest.mark.parametrize("direction", [-1, 1])
def test_short_marker_window_rejects_spike_then_accepts_sustained_onset(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 0., 0.), engagement_confirmation_observations=3))
    def observe(motor, marker, stamp):
        motor = [direction*motor, 0., 0.]
        estimator.advance_motor(motor)
        return estimator.observe_response(
            motor, _pose(), np.zeros((6, 3)), np.ones(6),
            timestamp_ns=stamp*50_000_000,
            observed_interface_position_m=direction*np.asarray(marker))
    observe(0., [0., 0., 0.], 0)
    # Recorded 204938 point-3 onset: one 4.5-sigma frame after reset.
    spike = observe(.2, [.00015725, .00000398, .00007065], 1)
    assert spike.phase[0] == "TAKEUP"
    assert spike.confirmation_count[0] == 0
    assert observe(.3, [0., 0., 0.], 2).phase[0] == "TAKEUP"
    assert observe(.4, [0., 0., .0003], 3).phase[0] == "TAKEUP"
    first = observe(.5, [0., 0., .0003], 4)
    assert first.phase[0] == "PROVISIONAL"
    assert first.confirmation_count[0] == 1
    assert observe(.5, [0., 0., .0003], 5).confirmation_count[0] == 2
    assert observe(.5, [0., 0., .0003], 6).phase[0] == "ENGAGED"


@pytest.mark.parametrize("direction", [-1, 1])
def test_short_window_stationary_interface_confirms_opposing_knob_chassis(direction):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(10., 0., 0.), engagement_confirmation_observations=3))
    def observe(motor, stamp):
        estimator.advance_motor(motor)
        return estimator.observe_response(
            motor, _pose(), np.zeros((6, 3)), np.ones(6),
            timestamp_ns=stamp*50_000_000,
            observed_interface_position_m=np.zeros(3))
    observe([0., 0., 0.], 0)
    # Shaft-2's fixed negative lead makes its insertion oppose shaft 0.
    motor = np.array([.2, 0., .6])*direction
    assert observe(motor, 1).phase[0] == "TAKEUP"
    assert observe(motor, 2).phase[0] == "PROVISIONAL"
    assert observe(motor, 3).confirmation_count[0] == 2
    assert observe(motor, 4).phase[0] == "ENGAGED"


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


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("confirm_while_held", [False, True])
def test_distal_confirmed_reversal_updates_gap_below_prior_floor(
        direction, confirm_while_held):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 28.868268966674805),
        engagement_confirmation_observations=2))
    mode = np.array([1., 0., 0.])
    def observe(motor, strain):
        estimator.advance_motor([0., 0., motor])
        return estimator.observe_response(
            [0., 0., motor], _pose(), _jacobian(), np.ones(6),
            [strain, 0., 0.], mode)
    observe(0., 0.)
    observe(-direction, .08)
    observe(-direction, .08)  # Complete initial engagement, not a full gap fit.
    np.testing.assert_allclose(estimator.width_positive[2], 28.868268966674805)
    observe(-direction+.1*direction, .08)  # Reversal resets visual window.
    first_motor = -direction+.3*direction
    first = observe(first_motor, .16)
    assert first.phase[2] == "PROVISIONAL"
    assert first.width_rad[2] == pytest.approx(28.868268966674805)
    if confirm_while_held:
        final = observe(first_motor, .16)
    else:
        final = observe(-direction+.5*direction, .24)
    assert final.phase[2] == "ENGAGED"
    assert final.width_rad[2] == pytest.approx(.3)
    assert final.engagement_anchor_motor_rad[2] == pytest.approx(first_motor)
    assert final.inferred_transmitted_increment_rad[2] == 0.  # No interface response.
    lo = final.width_positive_lower_rad if direction > 0 else final.width_negative_lower_rad
    hi = final.width_positive_upper_rad if direction > 0 else final.width_negative_upper_rad
    assert 0. <= lo[2] <= final.width_rad[2] <= hi[2]
    assert hi[2] <= .31+1e-12
    other = final.width_negative_rad if direction > 0 else final.width_positive_rad
    assert other[2] == pytest.approx(28.868268966674805)
    # Persistent stationary observations must not relearn the boundary.
    again = observe(-direction+.5*direction if not confirm_while_held else first_motor,
                    .24 if not confirm_while_held else .16)
    assert again.width_rad[2] == pytest.approx(.3)


@pytest.mark.parametrize("learning_rate", [0., .05])
def test_confirmed_boundary_learning_requires_real_reversal_and_valid_anchor(learning_rate):
    estimator = BacklashStateEstimator(BacklashConfig(
        width_rad=(1., 1., 1.), width_learning_rate=learning_rate))
    estimator.reversal_start_motor[2] = 0.
    estimator.engagement_anchor_motor[2] = .2
    estimator.confirmation_direction[2] = 1
    estimator._learn_confirmed_boundary(2)
    assert estimator.width_positive[2] == 1.  # Startup is not a gap measurement.
    estimator.reversal_observed[2] = True
    estimator.engagement_anchor_motor[2] = -.2
    estimator._learn_confirmed_boundary(2)
    assert estimator.width_positive[2] == 1.  # Wrong-side observation.
    estimator.engagement_anchor_motor[2] = .2
    estimator._learn_confirmed_boundary(2)
    assert estimator.width_positive[2] == pytest.approx(.2 if learning_rate else 1.)
    if learning_rate:
        estimator.engagement_anchor_motor[2] = .6
        estimator._learn_confirmed_boundary(2)
        assert estimator.width_positive[2] == pytest.approx(.22)  # Increases still blend.
