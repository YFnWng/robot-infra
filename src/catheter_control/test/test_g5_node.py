"""Real owner methods with deterministic sensors; no ROS node or actuation."""
from collections import deque
from threading import RLock
from types import SimpleNamespace
import numpy as np
import pytest
import torch

from catheter_control.node import CatheterControlNode
from catheter_control.orchestration.configuration import validate_experimental_composition
from catheter_control.planning.mppi import CatheterMppi,MppiConfig
from catheter_control.transmission.backlash import BacklashConfig,BacklashStateEstimator
from cr_meta_lnn.tests.test_engagement_runtime import proximal_runtime
from test_engagement_control import composition


@pytest.mark.parametrize('armed,quality_good,expected', [(True,True,1),(False,True,0),(True,False,0)])
def test_raw_onset_fast_path_only_publishes_zero_without_belief_commit(armed,quality_good,expected):
    from dataclasses import replace
    from catheter_control.safety.lifecycle import ControllerState
    from catheter_control.transmission.backlash import BacklashSnapshot
    c=composition()
    position=np.array([20.,0.,7.,40.,0.,0.])
    belief=BacklashSnapshot(width_rad=np.ones(3),width_positive_rad=np.ones(3),
        width_negative_rad=np.ones(3),remaining_rad=np.ones(3),
        motion_direction=np.array([0,0,-1]),engaged_direction=np.zeros(3),
        confidence=np.zeros(3),confirmation_count=np.zeros(3,dtype=int),
        phase=('UNKNOWN','UNKNOWN','TAKEUP'))
    c.arbiter.begin([3.,0.,3.,0.,0.,0.],position,c.planner.contract,belief)
    preview=replace(belief,tendon_response_onset=True,tendon_response_timestamp_ns=100)
    published=[]
    node=SimpleNamespace(takeup_transaction_enabled=True,armed=armed,fault_latched=False,
        mode_claimed=True,state=ControllerState.ACTIVE,_lock=RLock(),takeup_arbiter=c.arbiter,
        backlash_snapshot=belief,_backlash_history=[],_steady=lambda:1.,
        backlash_estimator=SimpleNamespace(preview_tendon_onset=lambda *a,**k:preview),
        last_command=np.ones(6),_publish_velocity_locked=lambda v:published.append(v.copy()),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=200)))
    quality=dict(confidence=np.full(4,float(quality_good)),source_rig_count=np.ones(4),
                 reprojection_error_px=np.zeros(4))
    CatheterControlNode._stop_on_raw_tendon_onset(node,100,np.zeros((4,3)),quality,.99)
    assert len(published)==expected
    assert node.backlash_snapshot is belief
    assert belief.phase[2]=='TAKEUP' and belief.confirmation_count[2]==0
    if expected:
        assert not np.any(published[0]) and not np.any(node.last_command)
        assert c.arbiter.response_onset_seen
        assert node.takeup_onset_timing['takeup_onset_arrival_to_detect_ms']==pytest.approx(10.)


def test_capture_feedback_requires_aligned_uninterrupted_response():
    from catheter_control.planning.tracking import TipForecastResult
    calls = []
    node = SimpleNamespace(planner=SimpleNamespace(
        observe_capture_response=lambda *args, **kwargs: calls.append((args, kwargs))))
    response = TipForecastResult(
        start_timestamp_ns=1_000_000_000,
        start_observation_timestamp_ns=1_000_000_000,
        due_timestamp_ns=1_200_000_000,
        observation_timestamp_ns=1_200_000_000,
        predicted_delta_mm=np.array([1., 0, 0]),
        measured_delta_mm=np.array([.5, 0, 0]),
        endpoint_error_mm=np.array([-1., 0, 0]),
        endpoint_error_norm_mm=1., direction_cosine=None,
        start_tip_m=np.zeros(3), predicted_terminal_tip_m=np.array([.0015, 0, 0]),
        observed_tip_m=np.array([.0005, 0, 0]),
        target_tip_m=np.array([.003, 0, 0]), capture_hold=True,
        comparison_valid=False)
    assert not CatheterControlNode._observe_capture_response_locked(node, response)
    assert not calls
    from dataclasses import replace
    CatheterControlNode._observe_capture_response_locked(
        node, replace(response, comparison_valid=True))
    assert len(calls) == 1
    # Progress is predicted motion anchored at measured root, not root-fit bias.
    assert calls[0][1]['predicted_progress_mm'] == pytest.approx(1.)


def test_aligned_response_publication_preserves_legacy_layout():
    from catheter_control.planning.tracking import TipForecastResult
    from rclpy.serialization import serialize_message, deserialize_message
    from control_interface.msg import MppiAlignedResponseTrace
    legacy, aligned = [], []
    node = SimpleNamespace(
        frame_id='robot_base',
        response_trace_pub=SimpleNamespace(publish=legacy.append),
        aligned_response_trace_pub=SimpleNamespace(publish=aligned.append))
    result = TipForecastResult(
        start_timestamp_ns=1_000_000_000,
        start_observation_timestamp_ns=1_000_000_000,
        due_timestamp_ns=1_200_000_000,
        observation_timestamp_ns=1_200_000_000,
        source_observation_timestamp_ns=1_230_000_000,
        source_start_observation_timestamp_ns=950_000_000,
        predicted_delta_mm=np.ones(3), measured_delta_mm=np.ones(3),
        endpoint_error_mm=np.zeros(3), endpoint_error_norm_mm=0.,
        direction_cosine=1., comparison_valid=True,
        comparison_reason='aligned_published_command',
        start_tip_m=np.zeros(3), model_start_tip_m=np.zeros(3),
        root_readout_error_mm=np.zeros(3),
        predicted_terminal_tip_m=np.ones(3)*.001,
        observed_tip_m=np.ones(3)*.001,
        command_logical_velocity=np.zeros(6), motor_radians_per_second=np.zeros(6),
        joint_position=np.zeros(6), model_motor_angle_rad=np.zeros(3),
        interface_pose=np.eye(4).ravel(), interface_jacobian=np.zeros(18),
        target_tip_m=np.ones(3))
    CatheterControlNode._publish_response_trace(node, result)
    assert len(legacy) == len(aligned) == 1
    read = deserialize_message(serialize_message(aligned[0]), MppiAlignedResponseTrace)
    assert read.comparison_valid
    assert read.source_start_observation_timestamp_ns == 950_000_000
    assert read.source_observation_timestamp_ns == 1_230_000_000
    assert read.comparison.observation_timestamp_ns == 1_200_000_000


@pytest.mark.parametrize('experimental',[False,True])
@pytest.mark.parametrize('scenario',['commit','cancel','marker_gap','final_hold','deadline','warmup'])
def test_matched_non_actuating_owner_scenarios(experimental,scenario):
    """Same inert owner/input/intent, actual production callback for both routes."""
    from dataclasses import replace
    from types import MethodType
    from test_path_plan_commit import fixture,recovery_fixture,tick
    if scenario == 'deadline':
        node,plan,faults,published=recovery_fixture(experimental)
        reference=node.path_reference
    else:
        node,reference,plan,faults,published=fixture()
    node.engagement_control_enabled=experimental
    node.model_diagnostics={'prediction_ready':True}
    node.backlash_snapshot.phase=['UNKNOWN']*3
    if scenario=='cancel':
        def solve(*a,**k):
            node.armed=False;node.reference_source='NONE';node.last_command.fill(0.)
            return plan
        node.planner.plan=solve
    elif scenario=='marker_gap':
        node.gates=replace(node.gates,marker_age_s=1.)
    elif scenario=='final_hold':
        node.path_reference=replace(reference,final_hold=True)
        def solve(*a,**k):
            node.path_reference=replace(node.path_reference,positions_m=reference.positions_m+5e-6)
            return plan
        node.planner.plan=solve
    elif scenario=='deadline':
        plan.valid=False;plan.reason='deadline_missed'
    elif scenario=='warmup':
        node.planner_warmed=False
        node.last_command.fill(0.);node.last_effective_command.fill(0.)
        node.planner.reset=lambda:None
        node._planner_warmup_supersessions=0
        node._complete_planner_warmup_locked=MethodType(CatheterControlNode._complete_planner_warmup_locked,node)
    tick(node)
    if scenario=='commit':
        np.testing.assert_array_equal(node.last_command,plan.command_logical_velocity)
        assert published==[plan] and not faults
    else:
        assert not np.any(node.last_command)
        assert published==([plan] if scenario=='deadline' else [])
        assert bool(faults)==(scenario=='marker_gap')
        if scenario=='deadline':
            assert node.state_reason == 'runtime_pause_zero:deadline_missed'
            assert node.estimator_catchup_started_at is not None
            assert node.last_plan is plan
            assert node.consecutive_deadline_misses == 1
            assert node.zeros and not np.any(node.zeros[-1])
        if scenario=='warmup':
            assert node.planner_warmed


@pytest.mark.parametrize('experimental',[False,True])
def test_matched_owner_takeup_exclusivity_and_completion(experimental):
    from dataclasses import replace
    from test_path_plan_commit import fixture,tick
    from test_engagement_control import interface
    from catheter_control.transmission.backlash import TakeupTransactionArbiter
    node,_,plan,faults,_=fixture()
    c=composition()
    node.engagement_control_enabled=experimental
    node.model_diagnostics={'prediction_ready':True}
    node.takeup_transaction_enabled=True
    node.takeup_arbiter=c.arbiter;node.contract=c.planner.contract
    node.position=np.array([20.,0.,5.,40.,0.,0.])
    belief=replace(interface(),phase=('ENGAGED','UNKNOWN','UNKNOWN'))
    belief.engaged_direction[2]=0
    node.backlash_snapshot=belief
    rates=np.array([0.,0.,.3,0.,0.,0.])
    plan.command_logical_velocity=node.contract.motor_axis_to_logical_velocity(
        node.contract.motor_radians_per_second_to_motor_axis_velocity(rates))
    node.reversal_scheduler.mark_transaction_started=lambda *a:None
    tick(node)
    assert node.takeup_arbiter.state==TakeupTransactionArbiter.TAKEUP_ACTIVE
    assert not faults and np.any(node.last_command)
    exclusive=node.last_command.copy()
    node.planner.plan=lambda *a,**k:pytest.fail('MPPI cannot solve during owned takeup')
    tick(node)
    np.testing.assert_array_equal(node.last_command,exclusive)
    node.backlash_snapshot=replace(belief, tendon_response_onset=True,
                                  phase=('ENGAGED','UNKNOWN','TAKEUP'),
                                  motion_direction=np.array([0, 0, 1]))
    tick(node)
    assert node.takeup_arbiter.state == TakeupTransactionArbiter.RESPONSE_HOLD
    assert not np.any(node.last_command) and not faults
    tick(node)
    assert node.takeup_arbiter.state == TakeupTransactionArbiter.RESPONSE_HOLD
    assert not np.any(node.last_command) and not faults
    node.backlash_snapshot=replace(belief,phase=('ENGAGED','UNKNOWN','ENGAGED'))
    node.backlash_snapshot.engaged_direction[2]=1
    tick(node)
    assert not np.any(node.last_command) and not faults
    assert node.takeup_arbiter.state==TakeupTransactionArbiter.REPLAN_REQUIRED


def test_unconfirmed_path_commit_keeps_scored_command_without_scaling():
    from test_path_plan_commit import fixture, tick
    node, _, plan, faults, published = fixture()
    node.engagement_control_enabled = True
    node.model_diagnostics = {'prediction_ready': True}
    node.backlash_snapshot.phase = ['UNKNOWN'] * 3
    node.backlash_snapshot.confidence = np.zeros(3)
    tick(node)
    np.testing.assert_array_equal(node.last_command, plan.command_logical_velocity)
    np.testing.assert_array_equal(node.last_effective_command, plan.command_logical_velocity)
    assert published == [plan] and not faults


def test_invalid_posterior_stops_before_planning():
    from test_path_plan_commit import fixture, tick
    node, _, _, faults, published = fixture()
    node.engagement_control_enabled = True
    node.model_diagnostics = {'prediction_ready': False}
    node.planner.plan = lambda *a, **k: pytest.fail('invalid posterior reached planner')
    tick(node)
    assert not np.any(node.last_command) and not published and not faults
    assert node.state_reason == 'model_posterior_not_ready'


def test_validation_checks_features_not_sample_budget_or_probe_policy():
    c=composition()
    planner=CatheterMppi(c.runtime,c.planner.contract,MppiConfig(samples=512,
        point_rollout_step_s=.2,point_rollout_coarse_steps=True))
    kwargs=dict(isolated=True,planner_device=torch.device('cuda'),marker_estimator='ukf',
                gain_enabled=False,scheduler_enabled=False)
    validate_experimental_composition(c.runtime,planner,c.arbiter,**kwargs)
    validate_experimental_composition(c.runtime,c.planner,c.arbiter,**kwargs)
    with pytest.raises(ValueError,match='isolated CUDA'):
        validate_experimental_composition(c.runtime,planner,c.arbiter,**dict(kwargs,planner_device=torch.device('cpu')))
    with pytest.raises(ValueError,match='fixed profile'):
        validate_experimental_composition(c.runtime,planner,c.arbiter,**dict(kwargs,gain_enabled=True))


def test_owner_initializes_from_bracketed_markers_and_publishes_joint_snapshots():
    runtime=proximal_runtime()
    runtime.use_external_tendon_confirmation()
    observer=BacklashStateEstimator(BacklashConfig((1.,1.,1.)))
    snapshots=[];traces=[]
    node=SimpleNamespace(runtime=runtime,engagement_control_enabled=True,armed=False,
        takeup_transaction_enabled=True,
        _lock=RLock(),_steady=lambda:0.,_timing=SimpleNamespace(record_seconds=lambda *a:None),
        _raw_encoder_history=deque(maxlen=256),_backlash_history=deque(maxlen=256),
        backlash_compensation_enabled=True,backlash_estimator=observer,backlash_snapshot=observer.snapshot(),
        backlash_config=observer.config,
        _fault_locked=lambda reason:pytest.fail(reason),
        _publish_planner_snapshot=lambda *a:snapshots.append(a),
        _publish_estimator_trace=lambda *a,**k:traces.append(a),
        tip_forecast_monitor=SimpleNamespace(observe=lambda *a:None),
        get_logger=lambda:SimpleNamespace(error=pytest.fail,warn=lambda *a:None))
    node._observe_backlash_at_timestamp=lambda *a,**k:CatheterControlNode._observe_backlash_at_timestamp(node,*a,**k)
    node._engagement_confirmation_at_source=lambda *a:CatheterControlNode._engagement_confirmation_at_source(node,*a)
    node._stop_on_raw_tendon_onset=lambda *a:CatheterControlNode._stop_on_raw_tendon_onset(node,*a)
    counts=np.zeros(6)
    assert not CatheterControlNode._process_encoder_sample(node,(100_000_000,counts,0.))
    assert not CatheterControlNode._process_encoder_sample(node,(200_000_000,counts,0.))
    assert runtime.estimator is None and not snapshots
    with torch.no_grad():
        points=runtime.kinematics(torch.eye(4,dtype=torch.float64),
            runtime.mechanics.natural_strain+10*runtime.mechanics.mode).numpy()
    quality=dict(confidence=np.ones(4),source_rig_count=np.ones(4),reprojection_error_px=np.zeros(4))
    assert CatheterControlNode._process_marker_sample(node,(150_000_000,points,quality,0.))
    assert node.estimator_health=='TRACKING' and node.accepted_observations==1
    root,_,belief=snapshots[-1]
    assert root.timestamp_ns==200_000_000 and root.estimator.events==[]
    assert traces[-1][1].timestamp_ns==150_000_000
    source_tip = runtime.markers_for_state(traces[-1][1])[-1]
    np.testing.assert_allclose(node._tip_readout_offset_m, points[-1]-source_tip)
    assert belief.phase[2]=='UNKNOWN'  # Static observations cannot confirm tendon.
    assert CatheterControlNode._process_encoder_sample(node,(300_000_000,counts,0.))
    assert CatheterControlNode._process_marker_sample(node,(250_000_000,points,quality,0.))
    assert snapshots[-1][0].accepted_observations==2
    assert snapshots[-1][2].phase[2]=='UNKNOWN'
