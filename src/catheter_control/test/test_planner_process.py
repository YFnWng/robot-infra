"""Non-actuating IPC and spawned planner qualification."""
from pathlib import Path
from dataclasses import replace
import socket
import pickle
import json
import os
import struct
import time

import numpy as np
import pytest

from catheter_control.orchestration import planner_process as ipc
from catheter_control.orchestration.runtime import load_runtime
from catheter_control.planning.mppi import CatheterMppi, MppiConfig
from catheter_control.safety.hardware_contract import load_hardware_contract

WORKSPACE = Path(__file__).resolve().parents[4]
MANIFEST = WORKSPACE / "cr_meta_lnn/artifacts/manifests/20260929_175554_grouped_no_rotation_v2.json"
LIMITS = WORKSPACE / "robot-infra/src/control_interface/config/catheter_limits.yaml"


def engagement_test_manifest():
    """Use the suite's selected artifact; never silently test a historic model."""
    configured = os.environ.get('ENGAGEMENT_TEST_MANIFEST')
    if not configured:
        pytest.skip('ENGAGEMENT_TEST_MANIFEST not configured for artifact integration')
    path = Path(configured)
    if not path.is_file():
        pytest.fail(f'configured ENGAGEMENT_TEST_MANIFEST missing: {path}')
    return path


def test_engagement_manifest_selection_is_explicit(tmp_path, monkeypatch):
    path = tmp_path/'selected.json'
    path.write_text('{}')
    monkeypatch.setenv('ENGAGEMENT_TEST_MANIFEST', str(path))
    assert engagement_test_manifest() == path
    monkeypatch.setenv('ENGAGEMENT_TEST_MANIFEST', str(tmp_path/'missing.json'))
    with pytest.raises(pytest.fail.Exception, match='configured ENGAGEMENT_TEST_MANIFEST missing'):
        engagement_test_manifest()
    monkeypatch.delenv('ENGAGEMENT_TEST_MANIFEST')
    with pytest.raises(pytest.skip.Exception, match='not configured'):
        engagement_test_manifest()


@pytest.mark.parametrize('record_evidence', [False, True])
def test_g5_manifest_and_snapshot_cross_real_spawned_worker(tmp_path, monkeypatch, record_evidence):
    """CPU integration only; production G5 still requires a CUDA worker."""
    import torch
    from cr_meta_lnn.deployment.experimental.engagement_bundle import make_control_trial_manifest
    shadow=engagement_test_manifest()
    path=tmp_path/'trial.json';path.write_text(json.dumps(make_control_trial_manifest(shadow)))
    bundle=load_runtime(path,device='cpu')
    runtime=bundle.runtime
    runtime.use_external_tendon_confirmation()
    u=sum(runtime.profile.input_knots)/len(runtime.profile.input_knots)
    with torch.no_grad():
        points=runtime.kinematics(torch.eye(4).to(runtime.mechanics.mode),
            runtime.mechanics.natural_strain+u*runtime.mechanics.mode).cpu().numpy()
    from cr_meta_lnn.deployment.experimental.joint_engagement_filter import TendonConfirmation
    runtime.initialize(1_000_000_000,np.zeros(6),points,
        tendon_confirmation=TendonConfirmation('UNKNOWN',0))
    root=runtime.clone_state(include_history=False)
    contract=load_hardware_contract(LIMITS,'imricor_test')
    config=MppiConfig(samples=8,horizon_steps=4,point_rollout_step_s=.2,
        point_rollout_coarse_steps=True,planning_deadline_s=2.)
    local=CatheterMppi(runtime,contract,config)
    if record_evidence:
        monkeypatch.setenv('CR_MPPI_EVIDENCE_DIR', str(tmp_path/'evidence'))
    else:
        monkeypatch.delenv('CR_MPPI_EVIDENCE_DIR', raising=False)
    remote=ipc.ProcessPlanner(runtime,contract,config,manifest=str(path),device='cpu',
        options={},identity=bundle.identity.manifest_sha256)
    try:
        plan=remote.plan(root,[20.,0,5.,0.,0.,0.],points[-1]+np.array([0,0,.003]))
        assert plan.valid, (plan.reason,remote.last_worker_error)
        assert plan.best_tip_sequence_m.shape==(4,3)
        reference=local.plan(root,[20.,0,5.,0.,0.,0.],points[-1]+np.array([0,0,.003]))
        assert reference.valid
        np.testing.assert_allclose(plan.command_logical_velocity,reference.command_logical_velocity,rtol=0,atol=1e-10)
        np.testing.assert_allclose(plan.best_tip_sequence_m,reference.best_tip_sequence_m,rtol=0,atol=1e-10)
    finally:
        remote.close()
    if record_evidence:
        records=list((tmp_path/'evidence').glob('worker_*/plan_*.pkl'))
        assert records
        record=pickle.loads(records[-1].read_bytes())
        assert record['input_root'] is not None
        assert record['input_execution_memory'] is not None
        assert record['candidate_ledgers']
        assert record['plan'].valid
    else:
        assert not (tmp_path/'evidence').exists()


def test_pre_send_expiry_does_not_reuse_prior_transport_failure(planners):
    _, remote, root = planners
    remote.last_transport_failure = {'request_started_s': -1, 'sequence': -1}
    remote._warmup_complete = True
    # Expire by supplying an old request timestamp, without altering transport.
    plan = remote.plan(root,[20,0,2,0,0,0],[.002,0,.07],deadline_started_s=time.monotonic()-2.)
    assert plan.reason == 'deadline_missed'
    assert remote.last_transport_failure is None


def pair():
    a, b = socket.socketpair()
    a.setblocking(False)
    b.setblocking(False)
    return a, b


def test_packet_roundtrip_and_eof():
    a, b = pair()
    try:
        timing = {}
        ipc.send_packet(a, {"sequence": 3, "state": np.arange(20)}, time.monotonic()+1,
                        timing=timing)
        reader = ipc.PacketReader(b)
        reply = reader.receive(time.monotonic()+1)
        assert timing['serialization_wall_s'] >= 0
        assert timing['packet_bytes'] > 0
        assert timing['send_completed_s'] <= reader.decode_timing['parent_first_byte_s']
        assert reader.decode_timing['parent_packet_complete_s'] >= reader.decode_timing['parent_first_byte_s']
        assert reply["sequence"] == 3
        np.testing.assert_array_equal(reply["state"], np.arange(20))
        a.close()
        with pytest.raises(EOFError):
            ipc.receive_packet(b, time.monotonic()+1)
    finally:
        a.close()
        b.close()


def test_receive_timeout_and_invalid_length():
    a, b = pair()
    try:
        with pytest.raises(TimeoutError):
            ipc.receive_packet(b, time.monotonic()+0.01)
        a.send(struct.pack("!I", ipc.MAX_PACKET_BYTES+1))
        with pytest.raises(ValueError, match="payload size"):
            ipc.receive_packet(b, time.monotonic()+1)
        with pytest.raises(ValueError, match="too large"):
            ipc.send_packet(a, b"x"*ipc.MAX_PACKET_BYTES, time.monotonic()+1)
    finally:
        a.close()
        b.close()


def test_backpressure_send_is_bounded():
    a, b = pair()
    try:
        a.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
        start = time.monotonic()
        with pytest.raises(ipc.PacketSendTimeout) as failure:
            ipc.send_packet(a, b"x"*100000, start+0.03)
        assert failure.value.bytes_sent > 0
        assert time.monotonic()-start < 0.3
    finally:
        a.close()
        b.close()


def test_expired_packet_send_transfers_no_bytes_and_keeps_framing():
    a, b = pair()
    try:
        with pytest.raises(ipc.PacketSendTimeout) as failure:
            ipc.send_packet(a, {'expired': True}, time.monotonic()-1)
        assert failure.value.bytes_sent == 0
        with pytest.raises(BlockingIOError):
            b.recv(1)
        ipc.send_packet(a, {'fresh': True}, time.monotonic()+1)
        assert ipc.receive_packet(b, time.monotonic()+1) == {'fresh': True}
    finally:
        a.close()
        b.close()


def test_unsent_request_deadline_preserves_worker_and_execution_memory(planners, monkeypatch):
    _, remote, root = planners
    remote._warmup_complete = True
    original = ipc.send_packet
    memory = pickle.dumps(remote.execution_memory())

    def expire_before_transfer(sock, value, deadline, **kwargs):
        monkeypatch.setattr(ipc, 'send_packet', original)
        return original(sock, value, time.monotonic()-1, **kwargs)

    monkeypatch.setattr(ipc, 'send_packet', expire_before_transfer)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == 'deadline_missed'
    assert not remote._failed and remote._pending_sequence is None
    assert not np.any(plan.command_logical_velocity)
    assert pickle.dumps(remote.execution_memory()) == memory
    failure = remote.last_transport_failure
    assert failure['bytes_sent'] == 0
    assert failure['request_send_timing']['serialization_wall_s'] >= 0
    assert failure['preparation_timings_s']['process_root_clone'] >= 0
    assert failure['preparation_timings_s']['process_memory_snapshot'] >= 0
    assert remote.plan(root, [20, 0, 2, 0, 0, 0], [.002, 0, .07]).valid


@pytest.fixture
def planners():
    bundle = load_runtime(str(MANIFEST), device="cpu")
    bundle.runtime.initialize(1_000_000_000, np.zeros(3))
    root = bundle.runtime.clone_state()
    contract = load_hardware_contract(LIMITS, "imricor_test")
    config = MppiConfig(samples=8, planning_deadline_s=1.0, capture_radius_mm=0)
    local = CatheterMppi(bundle.runtime, contract, config)
    remote = ipc.ProcessPlanner(
        bundle.runtime, contract, config, manifest=str(MANIFEST), device="cpu",
        options={}, identity=bundle.identity.manifest_sha256)
    try:
        yield local, remote, root
    finally:
        remote.close()
        assert not remote._worker.is_alive()


@pytest.mark.parametrize("objective", [{}, {"x_only": True}, {"xz_only": True}])
def test_spawned_planner_matches_persistent_local_planner(planners, objective):
    local, remote, root = planners
    for index in range(3):
        args = (root, [20, 0, 2, 0, 0, 0], [0.002, 0, 0.07])
        a, b = local.plan(*args, **objective), remote.plan(*args, **objective)
        assert b.reason == a.reason == "ok"
        trace = remote.process_trace[-1]
        assert trace['sequence'] == index+1
        assert not trace['discarded']
        assert trace['solve_wall_s'] >= trace['solve_cpu_s'] * .9
        assert trace['root_transfer_wall_s'] >= 0
        assert trace['reply_ready_to_decoded_ms'] >= 0
        if index:
            previous = trace['previous_reply_send']
            assert previous['sequence'] == index
            assert previous['send_completed_s'] <= trace['worker_received_s']
        np.testing.assert_allclose(a.command_logical_velocity, b.command_logical_velocity,
                                   atol=1e-10)
        np.testing.assert_allclose(a.command_tip_sequence_m, b.command_tip_sequence_m,
                                   atol=1e-8)
        if index == 1:
            local.reset()
            remote.reset()


def test_worker_death_never_falls_back(planners):
    _, remote, root = planners
    remote._worker.terminate()
    remote._worker.join(timeout=2)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "worker_unavailable"
    assert not np.any(plan.command_logical_velocity)


def test_timeout_discards_late_response_and_recovers(planners, monkeypatch):
    local, remote, root = planners
    remote._warmup_complete = True
    original = remote._reader.receive
    def timeout_once(deadline):
        monkeypatch.setattr(remote._reader, "receive", original)
        raise TimeoutError("synthetic reply timeout")
    monkeypatch.setattr(remote._reader, "receive", timeout_once)
    memory = remote.execution_memory()
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "deadline_missed"
    assert not np.any(plan.command_logical_velocity)
    assert not remote._failed
    assert remote._pending_sequence == 1
    assert "process_reply_wait_decode" in remote.process_timings_s
    assert pickle.dumps(remote.execution_memory()) == pickle.dumps(memory)
    args = (root, [20, 0, 2, 0, 0, 0], [0.002, 0, 0.07])
    expected, recovered = local.plan(*args), remote.plan(*args)
    assert recovered.valid and remote._pending_sequence is None
    assert "process_reply_drain" in remote.process_timings_s
    assert remote.process_trace[0]['sequence'] == 1
    assert remote.process_trace[0]['discarded']
    assert remote.process_trace[1]['sequence'] == 2
    assert not remote.process_trace[1]['discarded']
    np.testing.assert_allclose(recovered.command_logical_velocity,
                               expected.command_logical_velocity, atol=1e-10)


@pytest.mark.parametrize("split", [2, 7])
def test_partial_reply_survives_timeout(split):
    a, b = pair()
    try:
        reader = ipc.PacketReader(b)
        payload = pickle.dumps((17, "late reply"), protocol=5)
        packet = struct.pack("!I", len(payload))+payload
        a.sendall(packet[:split])
        with pytest.raises(TimeoutError):
            reader.receive(time.monotonic()+.01)
        a.sendall(packet[split:])
        assert reader.receive(time.monotonic()+1) == (17, "late reply")
        ipc.send_packet(a, (18, "fresh reply"), time.monotonic()+1)
        assert reader.receive(time.monotonic()+1) == (18, "fresh reply")
    finally:
        a.close()
        b.close()


def test_repeated_pending_reply_misses_do_not_enqueue(planners, monkeypatch):
    _, remote, root = planners
    remote._warmup_complete = True
    original = remote._reader.receive
    def timeout(*args):
        raise TimeoutError("pending reply delayed")
    monkeypatch.setattr(remote._reader, "receive", timeout)
    for _ in range(3):
        plan = remote.plan(root, np.zeros(6), np.zeros(3))
        assert plan.reason == "deadline_missed" and not plan.valid
        assert remote._sequence == remote._pending_sequence == 1
        assert not remote._failed
    monkeypatch.setattr(remote._reader, "receive", original)
    assert remote.plan(root, np.zeros(6), np.zeros(3)).valid


def test_reply_decoded_after_deadline_is_not_committed(planners, monkeypatch):
    _, remote, root = planners
    remote._warmup_complete = True
    remote.config = replace(remote.config, planning_deadline_s=.05)
    original = remote._reader.receive
    def delayed_decode(deadline):
        reply = original(time.monotonic()+1)
        time.sleep(.06)
        return reply
    monkeypatch.setattr(remote._reader, "receive", delayed_decode)
    memory = remote.execution_memory()
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "deadline_missed"
    assert remote._pending_sequence is None and not remote._failed
    assert pickle.dumps(remote.execution_memory()) == pickle.dumps(memory)

    assert remote.process_trace[-1]['discarded']
    assert remote.process_trace[-1]['reply_ready_to_decoded_ms'] >= 60


@pytest.mark.parametrize('bytes_sent', [None, 4])
def test_partial_request_send_timeout_still_fails_closed(planners, monkeypatch, bytes_sent):
    _, remote, root = planners
    def timeout(*args, **kwargs):
        if bytes_sent is not None:
            raise ipc.PacketSendTimeout(bytes_sent)
        raise TimeoutError("partial request")
    monkeypatch.setattr(ipc, "send_packet", timeout)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "worker_failure"
    assert remote._failed and not np.any(plan.command_logical_velocity)
    assert "process_request_send" in remote.process_timings_s
    assert remote.last_transport_failure['bytes_sent'] == bytes_sent


def test_new_capture_evidence_invalidates_inflight_decision(planners, monkeypatch):
    _, remote, root = planners
    original = remote._reader.receive
    def receive_and_release(*args):
        result = original(*args)
        remote.observe_capture_response([1, 0, 0], [0, 0, 0])
        return result
    monkeypatch.setattr(remote._reader, "receive", receive_and_release)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "worker_state_changed"
    assert remote.capture_diagnostics()["release_count"] == 1


def test_same_path_preview_updates_do_not_discard_worker_reply(planners, monkeypatch):
    from test_path_startup import owner, preview
    from catheter_control.node import CatheterControlNode
    _, remote, root = planners
    node, faults, _ = owner(remote)
    CatheterControlNode._path_reference_cb(node, preview())
    solved_reference = node.path_reference
    original = remote._reader.receive

    def receive_with_rolling_previews(*args):
        reply = original(*args)
        for sequence in range(2, 6):
            CatheterControlNode._path_reference_cb(node, preview(sequence))
        return reply

    monkeypatch.setattr(remote._reader, "receive", receive_with_rolling_previews)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert plan.valid and not faults
    assert not remote.process_trace[-1]["discarded"]
    assert CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", None, solved_reference, root.timestamp_ns, 100.04)


def test_final_hold_refresh_crosses_real_worker_and_commit_gate(planners, monkeypatch):
    from test_path_startup import owner, preview
    from catheter_control.node import CatheterControlNode
    from geometry_msgs.msg import Point
    _, remote, root = planners
    node, faults, _ = owner(remote)
    def final_preview(sequence):
        message = preview(sequence, final_hold=True)
        message.positions = [Point(x=.02, y=.02, z=.06) for _ in message.positions]
        return message
    CatheterControlNode._path_reference_cb(node, final_preview(1))
    solved_reference = node.path_reference
    target, _ = solved_reference.sample(
        root.timestamp_ns, remote.config.horizon_steps, remote.config.step_s, 100.)
    original = remote._reader.receive
    def receive_with_refresh(*args):
        reply = original(*args)
        CatheterControlNode._path_reference_cb(node, final_preview(2))
        return reply
    monkeypatch.setattr(remote._reader, "receive", receive_with_refresh)
    plan = remote.plan(root, np.zeros(6), target)
    assert plan.valid and not faults
    assert node.path_reference is not solved_reference
    assert CatheterControlNode._planner_reference_matches_locked(
        node, "PATH", target, solved_reference, root.timestamp_ns, 100.04)


def test_path_policy_change_discards_worker_reply(planners, monkeypatch):
    from test_path_startup import owner, preview
    from catheter_control.node import CatheterControlNode
    _, remote, root = planners
    node, _, _ = owner(remote)
    CatheterControlNode._path_reference_cb(node, preview())
    original = remote._reader.receive

    def receive_with_policy_change(*args):
        reply = original(*args)
        CatheterControlNode._path_reference_cb(node, preview(2, final_hold=True))
        return reply

    monkeypatch.setattr(remote._reader, "receive", receive_with_policy_change)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not plan.valid and plan.reason == "worker_state_changed"
    assert not np.any(plan.command_logical_velocity)


def test_consistent_capture_observation_is_not_lost(planners, monkeypatch):
    _, remote, root = planners
    original = remote._reader.receive
    def receive_and_observe(*args):
        result = original(*args)
        remote.observe_capture_response([1, 0, 0], [1, 0, 0])
        return result
    monkeypatch.setattr(remote._reader, "receive", receive_and_observe)
    plan = remote.plan(root, np.zeros(6), np.zeros(3))
    assert plan.valid
    assert remote.capture_diagnostics()["last_response_reason"] == "response_consistent"


def test_wrong_sequence_poisoned_channel(planners, monkeypatch):
    _, remote, root = planners
    original = remote._reader.receive
    def wrong_sequence(*args):
        sequence, plan, memory = original(*args)
        return sequence-1, plan, memory
    monkeypatch.setattr(remote._reader, "receive", wrong_sequence)
    result = remote.plan(root, np.zeros(6), np.zeros(3))
    assert not result.valid and result.reason == "worker_failure"
    assert remote._failed


def test_worker_initialization_checks_model_identity():
    bundle = load_runtime(str(MANIFEST), device="cpu")
    contract = load_hardware_contract(LIMITS, "imricor_test")
    with pytest.raises(RuntimeError, match="identity mismatch"):
        ipc.ProcessPlanner(bundle.runtime, contract, MppiConfig(samples=8),
            manifest=str(MANIFEST), device="cpu", options={}, identity="wrong")


def test_send_timeout_records_exact_phase_and_keeps_channel_fail_closed(planners, monkeypatch):
    _, remote, root = planners
    def timeout(*args, **kwargs):
        raise TimeoutError('injected bounded send stall')
    monkeypatch.setattr(ipc,'send_packet',timeout)
    result=remote.plan(root,np.zeros(6),np.zeros(3))
    assert not result.valid and result.reason=='worker_failure'
    assert remote._failed
    failure=json.loads(remote.worker_diagnostics()['planner_transport_failure'])
    assert failure['phase']=='process_request_send' and failure['sending']
    assert failure['error_type']=='TimeoutError'
    assert failure['error']=='injected bounded send stall'
    assert failure['failed_s']>=failure['phase_started_s']>=failure['request_started_s']
    assert remote.plan(root,np.zeros(6),np.zeros(3)).reason=='worker_unavailable'
    assert json.loads(remote.worker_diagnostics()['planner_transport_failure'])==failure
    assert all(isinstance(value,str) for value in remote.worker_diagnostics().values())
