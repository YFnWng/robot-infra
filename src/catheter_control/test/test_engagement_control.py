from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from cr_meta_lnn.tests.test_engagement_runtime import initialize_proximal
from catheter_control.planning.engagement import reconcile_engagement
from catheter_control.planning.mppi import CatheterMppi,MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.transmission.backlash import (BacklashSnapshot,BacklashConfig,
    BacklashFeedforwardCompensator,TakeupTransactionArbiter)


def interface():
    return BacklashSnapshot(width_rad=np.ones(3),width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),remaining_rad=np.zeros(3),motion_direction=np.ones(3,dtype=np.int8),
        engaged_direction=np.ones(3,dtype=np.int8),confidence=np.ones(3),confirmation_count=np.ones(3,dtype=np.int32),
        phase=('ENGAGED','UNKNOWN','ENGAGED'))


def composition(grouped=True):
    runtime=initialize_proximal(external_confirmation=True)
    contract=load_hardware_contract(Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml','imricor_test')
    # Disable physical rotation in the same contract used by the planner.
    maximum=contract.velocity_max.copy();maximum[1]=0.
    minimum=contract.velocity_min.copy();minimum[1]=0.
    contract=replace(contract,velocity_max=maximum,velocity_min=minimum)
    planner=CatheterMppi(runtime,contract,MppiConfig(samples=16,horizon_steps=4,step_s=.2,
        grouped_mode_sampling=grouped,planning_deadline_s=2.))
    arbiter=TakeupTransactionArbiter(BacklashFeedforwardCompensator(
        BacklashConfig((1.,1.,1.),takeup_velocity=(1.,10.,.5))))
    return SimpleNamespace(runtime=runtime,planner=planner,arbiter=arbiter)


def reconcile(c, snapshot):
    return reconcile_engagement(c.runtime, snapshot,
        backlash_config=c.arbiter.compensator.config)


def test_reconciliation_preserves_shared_owner_and_rejects_research_detector():
    c=composition();snapshot=interface();new=reconcile(c,snapshot)
    assert new.phase==snapshot.phase
    assert snapshot.phase[-1]=='ENGAGED'
    assert new is not snapshot
    c.runtime.estimator.external_confirmation=False
    with pytest.raises(ValueError,match='shared tendon confirmation'):
        reconcile(c,snapshot)


@pytest.mark.parametrize('grouped',[False,True])
def test_unknown_tendon_still_plans_motion_intent(grouped):
    c=composition(grouped)
    assert c.runtime.estimator.phase=='UNKNOWN'
    assert c.runtime.estimator.prediction_ready
    target=c.runtime.current_markers()[-1]+np.array([0.,0.,.003])
    belief=reconcile(c,replace(interface(),phase=('ENGAGED','UNKNOWN','UNKNOWN')))
    plan=c.planner.plan(c.runtime.clone_state(),[20.,0.,5.,40.,0.,0.],target,
                        transmission_state=belief)
    assert plan.valid,plan.reason
    assert plan.best_tip_sequence_m.shape==(4,3)
    assert plan.command_logical_velocity.any()


@pytest.mark.parametrize('grouped',[False,True])
def test_existing_joint_runtime_supports_shared_path_rollout(grouped):
    c=composition(grouped)
    tip=c.runtime.current_markers()[-1]
    target=tip+np.arange(1,5)[:,None]*np.array([0.,0.,.001])
    plan=c.planner.plan(c.runtime.clone_state(),[20.,0.,5.,40.,0.,0.],target,
        transmission_state=reconcile(c,interface()))
    assert plan.valid,plan.reason
    assert plan.command_tip_sequence_m.shape==(4,3)
    assert np.isfinite(plan.command_tip_sequence_m).all()


def test_insertion_intent_does_not_require_tendon_engagement():
    c=composition();belief=reconcile(c,interface())
    rates=np.array([.5,0.,0.,0.,0.,0.])
    axis=c.planner.contract.motor_radians_per_second_to_motor_axis_velocity(rates)
    intent=c.planner.contract.motor_axis_to_logical_velocity(axis)
    decision=c.arbiter.begin(intent,[20.,0.,5.,40.,0.,0.],c.planner.contract,belief)
    assert decision.execute_plan
    np.testing.assert_allclose(decision.command_logical_velocity,
        c.planner.contract.project_velocity(intent,[20.,0.,5.,40.,0.,0.]).logical_velocity)


def test_tendon_intent_uses_shared_takeup_then_zero_replan_boundary():
    c=composition();belief=reconcile(c,replace(interface(),phase=('ENGAGED','UNKNOWN','UNKNOWN')))
    rates=np.array([0.,0.,.3,0.,0.,0.])
    axis=c.planner.contract.motor_radians_per_second_to_motor_axis_velocity(rates)
    intent=c.planner.contract.motor_axis_to_logical_velocity(axis)
    decision=c.arbiter.begin(intent,[20.,0.,5.,40.,0.,0.],c.planner.contract,belief)
    assert not decision.execute_plan
    assert decision.command_logical_velocity.any()
    decision=c.arbiter.advance([20.,0.,5.,40.,0.,0.],c.planner.contract,
                              reconcile(c,interface()))
    assert decision.replan_required
    assert not decision.command_logical_velocity.any()


@pytest.mark.parametrize('phase', ['UNKNOWN','TAKEUP','PROVISIONAL','ENGAGED','FAILED'])
def test_model_latents_cannot_override_shared_evidence_or_travel_budget(phase):
    c=composition();snapshot=replace(interface(),phase=('ENGAGED','UNKNOWN',phase))
    snapshot.confirmation_count[2]=2
    snapshot.accumulated_takeup_rad[2]=37.5907269
    snapshot.engagement_anchor_motor_rad[2]=36.
    state=c.runtime.estimator
    state.phase='ENGAGED' if phase!='ENGAGED' else 'FAILED'
    state.widths[-1]=replace(state.widths[-1],location=1000.)
    belief=reconcile(c,snapshot)
    assert belief.phase==snapshot.phase
    for key in ('width_rad','width_positive_rad','width_negative_rad','remaining_rad',
                'confirmation_count','accumulated_takeup_rad','engagement_anchor_motor_rad',
                'engaged_direction','confidence','last_evidence_timestamp_ns'):
        np.testing.assert_array_equal(getattr(belief,key),getattr(snapshot,key))
