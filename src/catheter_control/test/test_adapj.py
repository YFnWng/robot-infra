"""Measured-tip controller initialization and closed-loop toy-plant tests."""
import json
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from types import MethodType
import threading
from collections import deque

import numpy as np
import pytest
import torch

from catheter_control.planning.adapj import TipAdapJ
from catheter_control.safety.hardware_contract import HardwareContract, MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
from catheter_control.orchestration.configuration import resolve_stack, locate_stack
from catheter_control.node import CatheterControlNode
from test_path_plan_commit import fixture, tick
from builtin_interfaces.msg import Time
from catheter_control.transmission.backlash import TakeupTransactionArbiter


def test_position_completion_requires_measured_post_deadline_standstill():
    owner = TipAdapJ(contract(), .2)
    owner.pending_command_ns = 0
    owner.note_position_execution(200_000_000, 0)
    history = [(t*1_000_000, [91, 0, 0, 0, 0, 0]) for t in range(200, 300, 10)]
    owner.confirm_position_standstill(history)
    assert owner.execution_stopped_ns is None
    history.append((300_000_000, [91, 0, 0, 0, 0, 0]))
    owner.confirm_position_standstill(history)
    assert owner.execution_stopped_ns == 300_000_000
    with pytest.raises(ValueError, match='deadline changed'):
        owner.note_position_execution(400_000_000, 100_000_000)
    owner.invalidate()
    assert owner.position_deadline_ns is None


def test_position_completion_rejects_motion_and_feedback_gaps():
    owner = TipAdapJ(contract(), .2)
    owner.pending_command_ns = 0
    owner.note_position_execution(200_000_000, 0)
    moving = [(t*1_000_000, [t, 0, 0, 0, 0, 0]) for t in range(200, 310, 10)]
    owner.confirm_position_standstill(moving)
    assert owner.execution_stopped_ns is None
    owner.confirm_position_standstill([(200_000_000, [91]*6), (300_000_000, [91]*6)])
    assert owner.execution_stopped_ns is None


def test_raw_callback_confirms_standstill_without_estimator_ticks():
    from control_interface.msg import DeviceStream
    owner = TipAdapJ(contract(), 1/15)
    owner.pending_command_ns = 900_000_000
    owner.note_position_execution(1_000_000_000, 900_000_000)
    faults = []
    node = SimpleNamespace(_lock=threading.Lock(), _steady=lambda: 1.,
        _record_header_age=lambda *a: None,
        _message_time_ns=lambda m: m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec,
        _simulation_reset_source_floor_ns=0,
        contract=SimpleNamespace(model_encoder_counts_are_valid=lambda _: True),
        encoder_source_timestamp_ns=None, encoder_receive_time=None,
        _adapj_encoder_history=deque(maxlen=1024), _raw_encoder_history=[],
        adapj=owner, mppi_execution_mode='position_stream',
        pending_encoder_timestamp_ns=None, _fault_locked=faults.append,
        _fault_if_active=faults.append)
    for ms in range(0, 111, 10):
        message = DeviceStream()
        message.header.stamp = Time(sec=1, nanosec=ms*1_000_000)
        message.predicate = DeviceStream.ENC
        message.data = [91., 0., 0., 0., 0., 0.]
        CatheterControlNode._device_state_cb(node, message)
    assert not faults and not node._raw_encoder_history
    assert len(node._adapj_encoder_history) == 12
    assert owner.execution_stopped_ns == 1_100_000_000
    # Duplicate/backward source frames cannot manufacture elapsed standstill.
    message.header.stamp = Time(sec=1, nanosec=50_000_000)
    CatheterControlNode._device_state_cb(node, message)
    assert len(node._adapj_encoder_history) == 12
    message.header.stamp = Time(sec=1, nanosec=120_000_000)
    message.data[0] = 91.5
    CatheterControlNode._device_state_cb(node, message)
    assert faults == ['malformed_adapj_encoder_feedback']
    assert len(node._adapj_encoder_history) == 12


def test_adapj_encoder_gate_uses_raw_receipt_without_relaxing_freshness():
    from catheter_control.safety.lifecycle import FreshnessLimits, readiness
    node = SimpleNamespace(adapj=object(), manager_ready=True,
        manager_time=9.99, position_time=9.99, encoder_time=9.,
        encoder_receive_time=9.99, position_source_timestamp_ns=9_990_000_000,
        encoder_source_timestamp_ns=9_990_000_000, marker_time=9.99,
        marker_diagnostic_time=9.99, marker_diagnostic_error=False,
        accepted_observations=5, required_observations=5,
        consecutive_rejections=0, maximum_rejections=3, target=object(),
        collection_present=False, position_valid=True, encoder_valid=True,
        model_valid=True, estimator_health='TRACKING', last_marker_result=None)
    gate = CatheterControlNode._gate_inputs_locked(node, 10.)
    assert gate.encoder_age_s == pytest.approx(.01)
    assert readiness(gate, FreshnessLimits())[1] == 'ready'
    node.adapj = None
    assert readiness(CatheterControlNode._gate_inputs_locked(node, 10.),
                     FreshnessLimits())[1] == 'encoder_stale'
    node.adapj = object()
    node.encoder_receive_time = 9.
    assert readiness(CatheterControlNode._gate_inputs_locked(node, 10.),
                     FreshnessLimits())[1] == 'encoder_stale'
    node.encoder_receive_time = 9.99
    node.marker_time = 9.
    assert readiness(CatheterControlNode._gate_inputs_locked(node, 10.),
                     FreshnessLimits())[1] == 'accepted_marker_stale'


def contract():
    return HardwareContract(np.zeros(6)-100., np.ones(6)*100.,
                            np.zeros(6), np.array([10., 0., 4.5, 0., 0., 0.]))


def counts(q):
    logical = np.zeros(6)
    logical[[0, 2]] = q
    return HardwareContract.logical_position_to_motor_axis_position(logical)/MOTOR_AXIS_UNITS_PER_ENCODER_COUNT


def plant(q):
    return np.array([[0., .001], [0., .0003], [.001, -.0002]])@q


def initialized(tmp_path):
    owner = TipAdapJ(contract(), 1/15)
    owner.begin()
    for i in range(80):
        stamp = 1_000_000_000+i*50_000_000
        q = np.array([20+2*np.sin(i*.23), 3+2*np.sin(i*.37)])
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False)
    path = owner.finish(tmp_path)
    return owner, path


def test_fit_records_achieved_inputs_and_fitted_parameters(tmp_path):
    owner, path = initialized(tmp_path)
    evidence = json.loads(path.read_text())
    assert owner.initialized and not owner.capturing
    assert len(evidence["timestamp_ns"]) == 80
    assert set(evidence["parameters"]) == {"A0", "A1", "B0"}
    assert len(evidence["implementation"]["sha256"]) == 64
    np.testing.assert_array_equal(
        np.array(evidence["fit_interval_end_ns"])-evidence["fit_interval_start_ns"],
        round(owner.step_s*1e9))
    with pytest.raises(FileExistsError):
        owner.finish(tmp_path)


def calibration_with_takeup(tmp_path, excluded=range(20, 40)):
    owner = TipAdapJ(contract(), 1/15, exclude_takeup_from_initialization=True)
    owner.begin()
    history = []
    for i in range(80):
        stamp = 1_000_000_000+i*50_000_000
        q = np.array([20+2*np.sin(i*.23), 3+2*np.sin(i*.37)])
        phase = "TAKEUP" if i in excluded else "ENGAGED"
        history.append([stamp, np.zeros(3),
                        {"phase": ("UNKNOWN", "UNKNOWN", phase)}, False])
        tip = plant(q) if i not in excluded else np.ones(3)*100.
        owner.observe(stamp, tip, [(stamp, counts(q))], update_allowed=False,
                      initialization_history=history)
    return owner, owner.finish(tmp_path)


def test_initialization_excludes_takeup_and_does_not_bridge_it(tmp_path):
    owner, path = calibration_with_takeup(tmp_path)
    evidence = json.loads(path.read_text())
    allowed = np.asarray(evidence["initialization_fit_allowed"], dtype=bool)
    assert len(evidence["timestamp_ns"]) == 80  # Excluded evidence is retained.
    assert not allowed[20:40].any()
    assert allowed[:20].all() and allowed[41:].all()
    assert evidence["initialization_excludes_takeup"]
    assert np.max(owner.tip_scale) < .01  # Takeup outliers do not set fit scales.
    for start, end in zip(evidence["fit_interval_start_ns"],
                          evidence["fit_interval_end_ns"]):
        assert end < 2_000_000_000 or start > 3_000_000_000


def test_initialization_normalization_origin_excludes_initial_takeup(tmp_path):
    owner, path = calibration_with_takeup(tmp_path, range(20))
    evidence = json.loads(path.read_text())
    first = np.flatnonzero(evidence["initialization_fit_allowed"])[0]
    np.testing.assert_array_equal(owner.tip_origin, evidence["tip_m"][first])
    np.testing.assert_array_equal(owner.q_origin, evidence["achieved_logical_position"][first])
    assert np.max(owner.tip_scale) < .01


def test_initialization_does_not_bridge_control_ownership_change():
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    q = np.array([20., 2.])
    history = []
    for i in range(3):
        stamp = 1_000_000_000+i*50_000_000
        history.append([stamp, np.zeros(3), {"phase": ("ENGAGED",)*3}, i > 0])
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False,
                      initialization_history=history)
    assert list(owner.initialization_fit_allowed) == [True, False, True]


@pytest.mark.parametrize("phase", ["TAKEUP", "PROVISIONAL", "FAILED"])
def test_initialization_rejects_blocked_history_between_camera_samples(phase):
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    history = [[1_000_000_000, np.zeros(3), {"phase": ("ENGAGED",)*3}, False]]
    q = np.array([20., 2.])
    owner.observe(1_000_000_000, plant(q), [(1_000_000_000, counts(q))],
                  update_allowed=False, initialization_history=history)
    history += [[1_025_000_000, np.zeros(3), {"phase": (phase, "UNKNOWN", "ENGAGED")}, False],
                [1_050_000_000, np.zeros(3), {"phase": ("ENGAGED",)*3}, False],
                [1_100_000_000, np.zeros(3), {"phase": ("TAKEUP",)*3}, False]]
    owner.observe(1_050_000_000, plant(q), [(1_050_000_000, counts(q))],
                  update_allowed=False, initialization_history=history)
    assert list(owner.initialization_fit_allowed) == [True, False]


def test_initialization_uses_source_history_for_deferred_observation():
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    q = np.array([20., 2.])
    history = [[1_000_000_000, np.zeros(3), {"phase": ("UNKNOWN", "UNKNOWN", "TAKEUP")}, False],
               [1_050_000_000, np.zeros(3), {"phase": ("ENGAGED",)*3}, False]]
    owner.observe(1_000_000_000, plant(q), [(999_000_000, counts(q))],
                  update_allowed=False, initialization_history=history)
    owner.observe(1_060_000_000, plant(q),
                  [(999_000_000, counts(q)), (1_050_000_000, counts(q))],
                  update_allowed=False, initialization_history=history)
    assert owner.samples[0][0] == 1_000_000_000
    assert list(owner.initialization_fit_allowed) == [False]


def test_initialization_does_not_use_future_takeup_phase():
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    q = np.array([20., 2.])
    history = [[1_000_000_000, np.zeros(3), {"phase": ("ENGAGED",)*3}, False],
               [1_050_000_000, np.zeros(3), {"phase": ("TAKEUP",)*3}, False]]
    owner.observe(1_000_000_000, plant(q), [(1_000_000_000, counts(q))],
                  update_allowed=False, initialization_history=history)
    assert list(owner.initialization_fit_allowed) == [True]


def test_ros_initialization_observes_corrected_source_confirmation_before_capture():
    calls = []
    history = [[1_000_000_000, np.zeros(3),
                {"phase": ("UNKNOWN", "UNKNOWN", "PROVISIONAL")}, False]]
    snapshot = SimpleNamespace(interface_pose=torch.eye(4), strain=torch.zeros(24),
                               adaptive_jacobian=object())
    runtime = SimpleNamespace(
        state=SimpleNamespace(accepted_observations=1, consecutive_rejections=0,
                              health="TRACKING"),
        observe_markers=lambda *a: SimpleNamespace(accepted=True),
        diagnostics=lambda: {}, clone_state=lambda: snapshot,
        clone_state_at_or_before=lambda *a: snapshot,
        markers_for_state=lambda *a: np.zeros((4, 3)),
        engaged_input_observation=lambda *a: (None, None))
    def confirm(*args, **kwargs):
        calls.append("confirmation")
        history[0][2]["phase"] = ("UNKNOWN", "UNKNOWN", "ENGAGED")
        return SimpleNamespace(failed=np.zeros(3, dtype=bool))
    def capture(*args, **kwargs):
        calls.append("capture")
        assert kwargs["initialization_history"] is history
        assert history[0][2]["phase"][2] == "ENGAGED"
        assert not kwargs["update_allowed"]  # Initialization remains disarmed.
    node = SimpleNamespace(
        runtime=runtime, engagement_control_enabled=False, armed=False,
        fault_latched=False, estimator_catchup_started_at=None,
        takeup_transaction_enabled=True, backlash_compensation_enabled=True,
        _lock=threading.RLock(), _steady=lambda: 0.,
        _timing=SimpleNamespace(record_seconds=lambda *a: None),
        _raw_encoder_history=[(1_000_000_000, counts([20., 2.]))],
        _adapj_encoder_history=[(1_000_000_000, counts([20., 2.]))],
        _backlash_history=history, _observe_backlash_at_timestamp=confirm,
        _stop_on_raw_tendon_onset=lambda *a: None,
        _fault_locked=pytest.fail, encoder_time=0.,
        tip_forecast_monitor=SimpleNamespace(observe=lambda *a: None),
        adapj=SimpleNamespace(observe=capture),
        _publish_estimator_trace=lambda *a, **k: None,
        _publish_planner_snapshot=lambda *a: None)
    assert CatheterControlNode._process_marker_sample(
        node, (1_000_000_000, np.zeros((4, 3)), {}, 0.))
    assert calls == ["confirmation", "capture"]


def test_compensated_initialization_missing_evidence_cannot_fit(tmp_path):
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    for i in range(80):
        stamp = 1_000_000_000+i*50_000_000
        q = np.array([20+2*np.sin(i*.23), 3+2*np.sin(i*.37)])
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False)
    with pytest.raises(ValueError, match="eligible initialization"):
        owner.finish(tmp_path)
    assert not owner.initialized and not (tmp_path/"initialization.json").exists()


def test_excluded_takeup_cannot_supply_missing_axis_excitation(tmp_path):
    owner = TipAdapJ(contract(), .05, exclude_takeup_from_initialization=True)
    owner.begin()
    history = []
    for i in range(80):
        stamp = 1_000_000_000+i*50_000_000
        excluded = 20 <= i < 40
        q = np.array([20+2*np.sin(i*.23), 2*np.sin(i*.37) if excluded else 0.])
        history.append([stamp, np.zeros(3),
                        {"phase": ("UNKNOWN", "UNKNOWN", "TAKEUP" if excluded else "ENGAGED")}, False])
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False,
                      initialization_history=history)
    with pytest.raises(ValueError, match="excite both axes"):
        owner.finish(tmp_path)
    assert not owner.initialized and not (tmp_path/"initialization.json").exists()


def test_offline_calibration_audit_retains_recorded_exclusions(tmp_path):
    owner, path = calibration_with_takeup(tmp_path)
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        "adapj_initialization_audit", root/"audits/ros-realtime/tools/audit_adapj_initialization.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    evidence = json.loads(path.read_text())
    result = module.audit(path, root/"src/control_interface/config/catheter_limits.yaml", owner.step_s)
    assert result["initialization_excludes_takeup"]
    assert result["eligible_initialization_samples"] == sum(evidence["initialization_fit_allowed"])
    assert result["refit_intervals"] == len(evidence["fit_interval_start_ns"])
    assert json.loads(path.read_text()) == evidence  # Read-only recording input.


def test_plain_initialization_does_not_exclude_by_compensation_phase(tmp_path):
    owner = TipAdapJ(contract(), .05)
    owner.begin()
    for i in range(80):
        stamp = 1_000_000_000+i*50_000_000
        q = np.array([20+2*np.sin(i*.23), 3+2*np.sin(i*.37)])
        history = [[stamp, np.zeros(3), {"phase": ("TAKEUP",)*3}, False]]
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False,
                      initialization_history=history)
    evidence = json.loads(owner.finish(tmp_path).read_text())
    assert all(evidence["initialization_fit_allowed"])
    assert not evidence["initialization_excludes_takeup"]


def test_simulation_adapj_profile_reuses_algorithm_and_disables_other_axes():
    root = Path(__file__).resolve().parents[1]/"config"
    stack = resolve_stack(locate_stack("simulation_adapj_plain_twoaxis", root),
                          config_root=root, expected_node_name="sim_catheter_mppi")
    assert stack.parameters["control_method"] == "adapj"
    assert stack.parameters["task_gc_enabled"]
    assert not stack.parameters["takeup_transaction_enabled"]
    assert not stack.parameters["backlash_compensation_enabled"]
    assert not stack.parameters["planner_process_enabled"]
    assert stack.parameters["controller_velocity_max"] == [10., 0., 4.5, 0., 0., 0.]


def test_no_excitation_or_missing_initialization_cannot_command(tmp_path):
    owner = TipAdapJ(contract(), .05)
    with pytest.raises(ValueError):
        owner.command(np.zeros(3), np.zeros(6), 1)
    owner.begin()
    for i in range(25):
        q = np.array([20., 0.])
        stamp = 1_000_000_000+i*50_000_000
        owner.observe(stamp, plant(q), [(stamp, counts(q))], update_allowed=False)
    with pytest.raises(ValueError, match="excite"):
        owner.finish(tmp_path)
    assert not owner.initialized and not (tmp_path/"initialization.json").exists()


def test_updates_require_post_command_source_timestamp_and_actual_position(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    command = owner.command(tip+[0, 0, .001], position, stamp+100_000_000)
    owner.execution_velocity(command, stamp+100_000_000)
    assert np.isfinite(command).all()
    assert np.all(command[[1, 3, 4, 5]] == 0)
    assert owner.command(tip, position, stamp+110_000_000) is None
    new = stamp+50_000_000
    owner.observe(new, tip, [(new, counts(q))], update_allowed=True)
    assert owner.updates == 0
    new = stamp+120_000_000
    owner.observe(new, tip, [(new, counts(q))], update_allowed=True)
    assert owner.updates == 0
    owner.execution_velocity(command, stamp+170_000_000)
    new = stamp+180_000_000
    owner.observe(new, tip, [(new, counts(q))], update_allowed=True)
    assert owner.updates == 1 and owner.pending_command_ns is None


def test_new_interval_seeds_current_achieved_action_after_history_reset(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    owner.invalidate()
    position = np.zeros(6)
    position[[0, 2]] = q
    owner.command(tip+[0, 0, .001], position, stamp+1)
    np.testing.assert_allclose(owner.controller._pending_prev_actuation_enc,
                               owner.coordinates(tip, q)[1])


def test_position_adaptation_uses_achieved_encoders_after_standstill(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6); position[[0, 2]] = q
    owner.command(tip+[0, 0, .001], position, stamp+1)
    deadline = stamp+50_000_000
    owner.note_position_execution(deadline, stamp+1)
    achieved = counts(q).copy(); achieved[0] += 91
    captured = []
    owner.controller.update = lambda state, action: captured.append(action.copy())
    # Observations during motion cannot train, even if they have real encoders.
    owner.observe(stamp+30_000_000, tip, [(stamp+30_000_000, achieved)], update_allowed=True)
    assert not captured
    history = [(t, achieved) for t in range(deadline, deadline+110_000_000, 10_000_000)]
    owner.confirm_position_standstill(history)
    observed = deadline+110_000_000
    owner.observe(observed, tip, [(observed, achieved)], update_allowed=True)
    expected_q = HardwareContract.encoder_counts_to_logical_position(achieved)[owner.axes]
    np.testing.assert_allclose(captured[0], owner.coordinates(tip, expected_q)[1])
    assert owner.updates == 1


def test_unbracketed_observation_cannot_reuse_previous_tip_for_motion(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    owner.observe(stamp+50_000_000, tip, [], update_allowed=True)
    assert owner.latest is None
    assert owner.command(tip+[0, 0, .001], np.zeros(6), stamp+60_000_000) is None
    fresh = stamp+100_000_000
    owner.observe(fresh, tip, [(fresh, counts(q))], update_allowed=True)
    position = np.zeros(6)
    position[[0, 2]] = q
    assert owner.command(tip, position, fresh+1) is not None


def test_zero_latency_marker_is_deferred_until_encoder_bracket_available():
    owner = TipAdapJ(contract(), 1/15)
    owner.begin()
    q = np.array([20., 2.])
    stamp = 1_000_000_000
    owner.observe(stamp, plant(q), [(stamp-1_000_000, counts(q))], update_allowed=False)
    assert not owner.samples and owner.latest is None
    owner.observe(stamp+30_000_000, plant(q),
                  [(stamp-1_000_000, counts(q)), (stamp+20_000_000, counts(q))],
                  update_allowed=False)
    assert len(owner.samples) == 1 and owner.latest[0] == stamp
    owner.invalidate()
    assert owner.deferred_observation is None


@pytest.mark.parametrize("adapj, expected_width", [(None, 3), (object(), 6)])
def test_ros_encoder_owner_retains_full_counts_for_adapj_only(adapj, expected_width):
    state = SimpleNamespace(health="TRACKING", accepted_observations=1,
                            consecutive_rejections=0)
    node = SimpleNamespace(
        adapj=adapj, armed=False, engagement_control_enabled=False,
        backlash_compensation_enabled=False, _lock=threading.Lock(),
        _raw_encoder_history=deque(maxlen=256), _steady=lambda: 0.,
        _timing=SimpleNamespace(record_seconds=lambda *args: None),
        runtime=SimpleNamespace(state=None, initialize=lambda *args, **kwargs: state,
                                diagnostics=lambda: {}),
        _publish_planner_snapshot=lambda *args: None)
    assert CatheterControlNode._process_encoder_sample(node, (100, np.arange(6.), 0.))
    np.testing.assert_array_equal(node._raw_encoder_history[-1][1], np.arange(expected_width))


def test_update_freezes_and_discards_pending_across_gap(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    owner.command(tip+[.001, 0, 0], position, stamp)
    prior = owner.controller.matrices
    new = stamp+200_000_000
    owner.observe(new, tip, [(new, counts(q))], update_allowed=True)
    assert owner.updates == 0 and owner.pending_command_ns is None
    for key, value in prior.items():
        np.testing.assert_array_equal(value, owner.controller.matrices[key])


def test_raw_encoder_pairing_interpolates_chassis_plus_knob():
    owner = TipAdapJ(contract(), .05)
    owner.begin()
    history = [(1_000_000_000, counts([20., 2.])), (1_100_000_000, counts([24., 4.]))]
    owner.observe(1_050_000_000, np.zeros(3), history, update_allowed=False)
    np.testing.assert_allclose(owner.latest[2], [22., 3.])
    owner.observe(1_200_000_000, np.zeros(3), history, update_allowed=False)
    assert len(owner.samples) == 1 and owner.last_reason == "awaiting_encoder_bracket"


def test_closed_loop_linear_plant_reduces_tip_error(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    target = tip+np.array([0.001, .0003, .0028])
    initial = np.linalg.norm(target-tip)
    for _ in range(60):
        position = np.zeros(6)
        position[[0, 2]] = q
        velocity = owner.command(target, position, stamp)
        owner.execution_velocity(velocity, stamp)
        q = q+velocity[[0, 2]]*owner.step_s
        owner.execution_velocity(velocity, stamp+round(owner.step_s*1e9))
        stamp += 100_000_000
        tip = plant(q)
        owner.observe(stamp, tip, [(stamp, counts(q))], update_allowed=True)
    assert np.linalg.norm(target-tip) < .3*initial


def test_native_adapj_scales_all_joints_by_one_ratio(tmp_path):
    owner, _ = initialized(tmp_path)
    owner.position_mode = True
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[owner.axes] = q
    desired_velocity = np.array([20., 10.])
    owner.controller.compute = lambda *_: (
        q+desired_velocity*owner.step_s-owner.q_origin)/owner.q_scale
    command = owner.command(tip+[0, 0, .001], position, stamp)
    ratio = min(1., np.min(owner.contract.velocity_max[owner.axes]/desired_velocity))
    np.testing.assert_allclose(command[owner.axes], desired_velocity*ratio)


def test_node_route_uses_shared_shell_and_never_calls_mppi():
    node, _, _, faults, _ = fixture()
    node.control_method = "adapj"
    node.reference_source = "POINT"
    node.adapj = SimpleNamespace(command=lambda *_: np.array([1., 0., .1, 0., 0., 0.]))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    node.plan_pub = SimpleNamespace(publish=lambda _: None)
    node._adapj_plan_locked = MethodType(CatheterControlNode._adapj_plan_locked, node)
    node.planner.plan = lambda *_args, **_kwargs: pytest.fail("AdapJ must not run MPPI")
    tick(node)
    assert not faults and node.state_reason == "active"
    np.testing.assert_array_equal(node.last_command, [1., 0., .1, 0., 0., 0.])


def test_node_adapj_builds_one_position_endpoint_then_preserves_it():
    node, _, _, faults, _ = fixture()
    node.reference_source = 'POINT'
    node.mppi_execution_mode = 'position_stream'
    node.position_execution = None
    node.position_stream_command_id = 0
    from catheter_control.safety.hardware_contract import load_hardware_contract
    node.contract = load_hardware_contract(
        Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml',
        'imricor_test')
    node.position_stream_encoder_counts = np.array([30000, 0, -10000, 10000, 0, 0])
    node.position = node.contract.encoder_counts_to_logical_position(node.position_stream_encoder_counts)
    node.adapj = SimpleNamespace(command=lambda *_: np.array([5., 0, 2., 0, 0, 0]),
                                step_s=.2, invalidate=lambda: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    node.plan_pub = SimpleNamespace(publish=lambda _: None)
    CatheterControlNode._adapj_plan_locked(node, 100.04)
    assert not faults and node.position_execution.command.active_mask == 5
    execution = node.position_execution
    node.adapj.command = lambda *_: pytest.fail('in-flight endpoint cannot be replaced')
    CatheterControlNode._adapj_plan_locked(node, 100.05)
    assert node.position_execution is execution
    assert node.position_stream_command_id == 1


def test_native_adapj_acquisition_uses_quantized_endpoint_mask():
    node, _, _, faults, _ = fixture()
    node.reference_source = 'POINT'
    node.mppi_execution_mode = 'position_stream'
    node.position_execution = None
    node.position_stream_command_id = 0
    from catheter_control.safety.hardware_contract import load_hardware_contract
    node.contract = load_hardware_contract(
        Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml',
        'imricor_test')
    node.position_stream_encoder_counts = np.array([47537, 0, -10000, 0, 0, 0])
    node.position = node.contract.encoder_counts_to_logical_position(node.position_stream_encoder_counts)
    node.adapj = SimpleNamespace(command=lambda *_: np.array([9., 0, .05, 0, 0, 0]),
                                step_s=1/15, invalidate=lambda: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    node.plan_pub = SimpleNamespace(publish=lambda _: None)
    node.takeup_transaction_enabled = True
    def begin(command, position, contract, snapshot, *, physical_direction):
        np.testing.assert_array_equal(physical_direction, [1, 0, 0])
        assert command[2] == 0
        return SimpleNamespace(state=TakeupTransactionArbiter.TAKEUP_ACTIVE,
            execute_plan=False, command_logical_velocity=np.array([2.,0,0,0,0,0]),
            reason='takeup_active')
    node.takeup_arbiter = SimpleNamespace(begin=begin)
    CatheterControlNode._adapj_plan_locked(node, 100.04)
    assert not faults and node.position_execution is None


def test_plain_native_adapj_heartbeat_does_not_reproject_endpoint_token():
    from inspect import unwrap
    node, _, _, faults, _ = fixture()
    node.adapj = object()
    node.mppi_execution_mode = 'position_stream'
    node.command_timeout_s = 1.
    node.last_command = np.array([.7, 0, .003, 0, 0, 0])
    sent = []
    node._publish_velocity_locked = lambda command: sent.append(command.copy())
    node._compensated_command_locked = lambda *_: pytest.fail('native distance cannot be velocity-projected')
    node.contract = SimpleNamespace(project_publishable_velocity=lambda *_: pytest.fail('speed floor'))
    unwrap(CatheterControlNode._heartbeat_tick)(node)
    assert not faults
    np.testing.assert_array_equal(sent[0], node.last_command)


def test_shared_arbiter_accepts_native_shaft_mask_without_speed_floor_activation():
    from catheter_control.transmission.backlash import (
        BacklashConfig, BacklashStateEstimator, BacklashFeedforwardCompensator)
    from catheter_control.safety.hardware_contract import load_hardware_contract
    hardware = load_hardware_contract(
        Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml',
        'imricor_test')
    config = BacklashConfig(width_rad=(1., 1., 1.))
    arbiter = TakeupTransactionArbiter(BacklashFeedforwardCompensator(config))
    state = BacklashStateEstimator(config).snapshot()
    result = arbiter.begin([9.,0,.05,0,0,0], [20.,0,7.,0,0,0], hardware,
                           state, physical_direction=[1,0,0])
    np.testing.assert_array_equal(arbiter.active_mask, [True,False,False])
    motor = hardware.logical_position_to_motor_axis_position(result.command_logical_velocity)
    assert motor[0] > 0 and motor[2] == 0


def test_compensation_freezes_matrices_and_requires_post_handoff_observation(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    owner.command(tip+[0, 0, .001], position, stamp)
    original = owner.controller.matrices
    owner.invalidate()  # arbiter intercepts the unexecuted AdapJ proposal
    for i in range(1, 4):
        t = stamp+i*50_000_000
        moved = q+[i, i]
        owner.observe(t, plant(moved), [(t, counts(moved))], update_allowed=False)
    owner.resume_after(stamp+200_000_000)
    assert owner.command(tip, position, stamp+200_000_001) is None
    for key, value in original.items():
        np.testing.assert_array_equal(value, owner.controller.matrices[key])
    t = stamp+250_000_000
    moved = q+[3, 3]
    owner.observe(t, plant(moved), [(t, counts(moved))], update_allowed=True)
    assert owner.updates == 0
    position[[0, 2]] = moved
    command = owner.command(plant(moved)+[0, 0, .001], position, t)
    assert command is not None
    owner.execution_velocity(command, t)
    owner.execution_velocity(command, t+70_000_000)
    t += 100_000_000
    owner.observe(t, plant(moved), [(t, counts(moved))], update_allowed=True)
    assert owner.updates == 1


def test_adapj_proposal_is_intercepted_by_existing_arbiter_then_shared_advance():
    node, _, _, faults, _ = fixture()
    node.control_method = "adapj"
    node.reference_source = "POINT"
    node.takeup_transaction_enabled = True
    invalidated = []
    proposed = np.array([1., 0., .1, 0., 0., 0.])
    takeup = np.array([2., 0., .2, 0., 0., 0.])
    node.adapj = SimpleNamespace(command=lambda *_: proposed,
                                invalidate=lambda: invalidated.append(True))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1)))
    node.plan_pub = SimpleNamespace(publish=lambda _: None)
    arbiter = SimpleNamespace(state=TakeupTransactionArbiter.READY_TO_PLAN)
    def begin(*args):
        np.testing.assert_array_equal(args[0], proposed)
        arbiter.state = TakeupTransactionArbiter.TAKEUP_ACTIVE
        return SimpleNamespace(state=arbiter.state, reason="takeup_active",
                               execute_plan=False, command_logical_velocity=takeup)
    arbiter.begin = begin
    arbiter.advance = lambda *args: SimpleNamespace(
        state=TakeupTransactionArbiter.TAKEUP_ACTIVE, reason="takeup_active",
        command_logical_velocity=takeup)
    node.takeup_arbiter = arbiter
    node._adapj_plan_locked = MethodType(CatheterControlNode._adapj_plan_locked, node)
    tick(node)
    np.testing.assert_array_equal(node.last_command, takeup)
    assert invalidated and not faults
    node.adapj.command = lambda *_: pytest.fail("AdapJ cannot plan during takeup")
    tick(node)
    np.testing.assert_array_equal(node.last_command, takeup)


@pytest.mark.parametrize("prefix,node_name", [("hardware", "catheter_mppi"), ("simulation", "sim_catheter_mppi")])
def test_compensated_profiles_change_only_compensation(prefix, node_name):
    root = Path(__file__).resolve().parents[1]/"config"
    def parameters(name):
        return resolve_stack(locate_stack(name, root), config_root=root,
                             expected_node_name=node_name).parameters
    plain = parameters(f"{prefix}_adapj_plain_twoaxis")
    compensated = parameters(f"{prefix}_adapj_takeup_twoaxis")
    changed = {key for key in plain if plain[key] != compensated[key]}
    assert changed == {"backlash_compensation_enabled", "takeup_transaction_enabled"}
    assert all(compensated[key] for key in changed)
    assert compensated['backlash_tendon_takeup_joint_range_enabled']


def test_profile_resolves_cpu_no_compensation_and_task_gc():
    config = Path(__file__).resolve().parents[1]/"config"
    profile = resolve_stack(locate_stack("hardware_adapj_plain_twoaxis", config), config_root=config)
    values = profile.parameters
    assert values["control_method"] == "adapj"
    assert values["device"] == "cpu" and not values["planner_process_enabled"]
    assert not values["takeup_transaction_enabled"]
    assert values["task_gc_enabled"]


def test_node_waiting_response_commands_zero_without_refreshing_decision():
    node, _, _, faults, _ = fixture()
    node.reference_source = "POINT"
    node.adapj = SimpleNamespace(command=lambda *_: None, interval_active=lambda _: False)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    CatheterControlNode._adapj_plan_locked(node, 100.04)
    assert not faults and not np.any(node.last_command)
    assert node.state_reason == "adapj_response_hold"


def test_interval_requires_published_zero_and_source_time_after_it(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    command = owner.command(tip+[0, 0, .001], position, stamp)
    start = stamp+10_000_000
    np.testing.assert_array_equal(owner.execution_velocity(command, start), command)
    early = start+40_000_000
    owner.observe(early, tip, [(early, counts(q))], update_allowed=True)
    assert owner.updates == 0 and owner.interval_active(early)
    stop = start+70_000_000
    assert not np.any(owner.execution_velocity(command, stop))
    assert owner.execution_stopped_ns == stop
    old = stop-1
    owner.observe(old, tip, [(old, counts(q))], update_allowed=True)
    assert owner.updates == 0
    assert not np.any(owner.execution_velocity(command, stop+20_000_000))
    fresh = stop+30_000_000
    owner.observe(fresh, tip, [(fresh, counts(q))], update_allowed=True)
    assert owner.updates == 1 and owner.pending_command_ns is None
    assert not np.any(owner.execution_velocity(command, fresh+1))


def test_interrupted_pulse_discards_adaptation(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    command = owner.command(tip+[0, 0, .001], position, stamp)
    owner.execution_velocity(command, stamp)
    owner.execution_velocity(np.zeros(6), stamp+20_000_000)
    assert owner.pending_command_ns is None
    fresh = stamp+100_000_000
    owner.observe(fresh, tip, [(fresh, counts(q))], update_allowed=True)
    assert owner.updates == 0
    assert not np.any(owner.execution_velocity(command, fresh+1))


def test_ros_publication_bounds_adapj_pulse(tmp_path):
    owner, _ = initialized(tmp_path)
    stamp, tip, q = owner.latest
    position = np.zeros(6)
    position[[0, 2]] = q
    command = owner.command(tip+[0, 0, .001], position, stamp)
    clock_ns = [stamp]
    published = []
    node = SimpleNamespace(command_output_enabled=True, adapj=owner, source="test",
        takeup_transaction_enabled=False,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=clock_ns[0]//1_000_000_000,
                               nanosec=clock_ns[0]%1_000_000_000))),
        tip_forecast_monitor=SimpleNamespace(note_command=lambda *_: None),
        control_pub=SimpleNamespace(publish=lambda m: published.append(m.joint_vel)))
    CatheterControlNode._publish_velocity_locked(node, command)
    clock_ns[0] += 70_000_000
    CatheterControlNode._publish_velocity_locked(node, command)
    np.testing.assert_array_equal(published[0], command)
    assert not np.any(published[1])
    assert owner.execution_stopped_ns == clock_ns[0]
    owner.invalidate()
    node.takeup_transaction_enabled = True
    node.takeup_arbiter = SimpleNamespace(state=TakeupTransactionArbiter.TAKEUP_ACTIVE,
                                        note_command_publication=lambda *args:None)
    CatheterControlNode._publish_velocity_locked(node, command)
    np.testing.assert_array_equal(published[-1], command)


def test_initialization_service_refuses_when_armed():
    from threading import RLock
    node = SimpleNamespace(_lock=RLock(), armed=True)
    response = SimpleNamespace(success=None, message="")
    CatheterControlNode._adapj_initialization_cb(node, SimpleNamespace(data=True), response)
    assert response.success is False and "disarmed" in response.message


@pytest.mark.parametrize("step,reference", [(0., 1.), (.1, 0.), (.1, float("nan"))])
def test_adapj_configuration_rejects_invalid_step(step, reference):
    with pytest.raises(ValueError):
        TipAdapJ(contract(), step, reference)
