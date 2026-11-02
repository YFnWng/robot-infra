"""Historical detector mismatch and repaired shared-owner confirmation parity."""
import numpy as np
import torch

from catheter_control.transmission.backlash import BacklashConfig, BacklashStateEstimator
from cr_meta_lnn.networks.hybrid.cartesian_engagement import CartesianEngagementEvidence
from cr_meta_lnn.tests.test_joint_engagement_filter import fixture


def test_small_resolved_bending_confirms_baseline_but_not_pair_distance_policy():
    mechanics, kinematics, _ = fixture()
    mode = mechanics.mode.numpy()
    eye = np.eye(4)
    q = torch.zeros(24, dtype=torch.float64)
    def points(strain):
        return kinematics(torch.eye(4).to(strain), strain)[[0, 5, 10, 15]].numpy()
    baseline = BacklashStateEstimator(BacklashConfig(width_rad=(.1, .1, .1),
        engagement_confirmation_observations=3, width_learning_rate=0.))
    detector = CartesianEngagementEvidence(.3, .0003, 3.)
    raw = np.zeros(3)
    jacobian = np.eye(6, 3)
    scale = np.ones(6)
    baseline.advance_motor(raw)
    baseline.observe_response(raw, eye, jacobian, scale, distal_strain=q.numpy(),
        distal_bending_mode=mode, timestamp_ns=0, observed_interface_position_m=points(q)[0])
    for t in [0., .1, .2, .3]:
        detector.observe(t, points(q))
    detector.reverse(.35)
    for i in range(1, 4):
        raw[2] = .02*i
        q = mechanics.mode*(.2*i)
        baseline.advance_motor(raw)
        baseline.observe_response(raw, eye, jacobian, scale, distal_strain=q.numpy(),
            distal_bending_mode=mode, timestamp_ns=int((.3+.05*i)*1e9),
            observed_interface_position_m=points(q)[0])
        response, detected = detector.observe(.3+.05*i, points(q))
        assert response < .0003
        assert not detected
    assert baseline.phase[2] == 'ENGAGED'
    assert baseline.confirmation_count[2] == 3


def test_control_routes_use_identical_confirmation_policy_and_homing_lifecycle():
    from collections import deque
    from types import SimpleNamespace
    from catheter_control.node import CatheterControlNode
    from catheter_control.planning.engagement import confirmation_from_checkpoint
    mechanics, _, _ = fixture()
    mode=mechanics.mode
    configs=BacklashConfig(width_rad=(.1,.1,.1),engagement_confirmation_observations=3,
                          width_learning_rate=.1)
    nodes=[]
    for new in [False,True]:
        owner=BacklashStateEstimator(configs)
        nodes.append(SimpleNamespace(armed=True,engagement_control_enabled=new,
            runtime=SimpleNamespace(mechanics=SimpleNamespace(mode=mode),port=SimpleNamespace(mode=mode),
                                    tendon_mm_per_shaft_rad=-.18949385411878789),
            backlash_estimator=owner,backlash_snapshot=owner.snapshot(),_backlash_history=deque(maxlen=256)))
    # Identical supplied posterior strains isolate the decision contract, not
    # differences in the two estimators' inferred shapes.
    samples=[(0.,0.,True),(.02,0.,True)]+[(.04,.2,True)]*4+[
             (-.02,.2,True)]+[(-.04,0.,True)]*5+[
             (-.05,0.,False),(-.05,0.,True),(-.08,0.,True)]
    phases=[]
    model=SimpleNamespace(jacobian=np.eye(6,3),state_scale=np.ones(6))
    for i,(angle,amplitude,owned) in enumerate(samples):
        stamp=1791320000000000000+i*50_000_000
        for node in nodes:
            node.armed=owned
            raw=np.array([0.,0.,angle])
            owner=node.backlash_estimator
            owner.advance_motor(raw,external_motion=not owned)
            node._backlash_history.append([stamp,raw,owner.clone_state(),owned])
            node.backlash_snapshot=CatheterControlNode._observe_backlash_at_timestamp(node,
                stamp,np.eye(4),model,amplitude*mode.numpy(),observed_interface_position_m=np.zeros(3),
                observed_marker_points_m=np.array([[0.,0.,0.],[0.,0.,.01],
                    [0.,0.,.02],[0.,0.,.03+amplitude*.01]]))
        a,b=[node.backlash_snapshot for node in nodes]
        assert a.phase==b.phase
        for key in ('confirmation_count','engaged_direction','remaining_rad','width_rad',
                    'engagement_anchor_motor_rad','reversal_start_motor_rad','confidence'):
            np.testing.assert_array_equal(getattr(a,key),getattr(b,key))
        view=confirmation_from_checkpoint(nodes[1].runtime,nodes[1]._backlash_history[-1][2])
        assert view.phase==b.phase[2]
        assert view.direction==-b.motion_direction[2]
        phases.append(b.phase[2])
    assert phases[:6]==['UNKNOWN','TAKEUP','TAKEUP','PROVISIONAL','PROVISIONAL','ENGAGED']
    assert phases[6]=='TAKEUP' and phases[11]=='ENGAGED'
    assert phases[12]=='TAKEUP'  # External motion starts fresh physical evidence.
    assert phases[13]=='UNKNOWN'  # No homing transaction transfers on arm.
