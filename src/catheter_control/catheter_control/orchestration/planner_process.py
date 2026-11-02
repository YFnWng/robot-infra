"""Opt-in spawned MPPI compute owner. No ROS, commands, or filesystem IPC.

One private socketpair carries one bounded request at a time. Parent safety
gates and capture feedback remain authoritative. No CUDA tensors cross IPC.
The spawn context avoids inheriting ROS threads or a live CUDA context.
"""
from __future__ import annotations

import multiprocessing
import os
from pathlib import Path
import json
import pickle
import select
import socket
import struct
import threading
import time

from ..planning.mppi import CatheterMppi
from .runtime import load_runtime

MAX_PACKET_BYTES = 4 * 1024 * 1024


class PacketSendTimeout(TimeoutError):
    """A bounded send with an exact framing-progress count."""

    def __init__(self, bytes_sent):
        super().__init__("planner IPC deadline exceeded")
        self.bytes_sent = bytes_sent


def _transfer(sock, buffer, *, writing, deadline, timing=None):
    view = memoryview(buffer)
    transferred = 0
    if writing and timing is not None:
        timing['bytes_sent'] = 0
    while view:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            if writing:
                raise PacketSendTimeout(transferred)
            raise TimeoutError("planner IPC deadline exceeded")
        readable, writable, _ = select.select(
            [] if writing else [sock], [sock] if writing else [], [], remaining)
        if not readable and not writable:
            if writing:
                raise PacketSendTimeout(transferred)
            raise TimeoutError("planner IPC deadline exceeded")
        try:
            count = sock.send(view) if writing else sock.recv_into(view)
        except BlockingIOError:
            continue
        if not count:
            raise EOFError("planner worker disconnected")
        transferred += count
        if writing and timing is not None:
            timing['bytes_sent'] = transferred
        view = view[count:]


def send_packet(sock, value, deadline, *, timing=None):
    started, cpu = time.monotonic(), time.thread_time()
    payload = pickle.dumps(value, protocol=5)
    if timing is not None:
        timing.update(serialization_wall_s=time.monotonic()-started,
                      serialization_cpu_s=time.thread_time()-cpu,
                      packet_bytes=len(payload))
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("planner IPC payload too large")
    _transfer(sock, struct.pack("!I", len(payload)) + payload,
              writing=True, deadline=deadline, timing=timing)
    if timing is not None:
        timing['send_completed_s'] = time.monotonic()


def receive_packet(sock, deadline):
    return PacketReader(sock).receive(deadline)


class PacketReader:
    """Retain partial framing across deadline misses; never reuse lost bytes."""

    def __init__(self, sock):
        self.sock = sock
        self.buffer = bytearray()
        self.size = None
        self.decode_timing = {}
        self.first_byte_s = None

    def receive(self, deadline):
        while True:
            needed = 4 if self.size is None else self.size
            if len(self.buffer) == needed:
                if self.size is None:
                    self.size, = struct.unpack("!I", self.buffer)
                    self.buffer.clear()
                    if not 0 < self.size <= MAX_PACKET_BYTES:
                        raise ValueError("invalid planner IPC payload size")
                else:
                    payload = bytes(self.buffer)
                    self.buffer.clear()
                    self.size = None
                    # Private trusted child only; never deserialize external input.
                    started, cpu = time.monotonic(), time.thread_time()
                    value = pickle.loads(payload)
                    self.decode_timing = dict(decode_wall_s=time.monotonic()-started,
                                              decode_cpu_s=time.thread_time()-cpu,
                                              parent_first_byte_s=self.first_byte_s,
                                              parent_packet_complete_s=started)
                    self.first_byte_s = None
                    return value
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("planner IPC deadline exceeded")
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                raise TimeoutError("planner IPC deadline exceeded")
            try:
                chunk = self.sock.recv(needed - len(self.buffer))
            except BlockingIOError:
                continue
            if not chunk:
                raise EOFError("planner worker disconnected")
            if self.first_byte_s is None:
                self.first_byte_s = time.monotonic()
            self.buffer.extend(chunk)


def _worker(sock, manifest, device, options, identity, contract, config, threads):
    import os
    import torch
    torch.set_num_threads(threads[0])
    torch.set_num_interop_threads(threads[1])
    sock.setblocking(False)
    evidence = None
    try:
        bundle = load_runtime(manifest, device=device, options=options)
        if bundle.identity.manifest_sha256 != identity:
            raise ValueError("planner worker model identity mismatch")
        planner = CatheterMppi(bundle.runtime, contract, config)
        directory = os.environ.get('CR_MPPI_EVIDENCE_DIR', '')
        if directory:
            from .planner_evidence import PlannerEvidenceWriter
            evidence = PlannerEvidenceWriter(directory, dict(
                schema='mppi_worker_evidence_v1',manifest=manifest,
                manifest_sha256=identity,device=device,options=options,
                runtime_identity=bundle.identity,
                manifest_contents=json.loads(Path(manifest).read_text()),
                contract=contract,config=config,threads=threads,
                note='Intrusive diagnostics; input snapshots/RNG are exact worker inputs, not actuator acknowledgement.'))
        send_packet(sock, {"ready": True}, time.monotonic() + 30)
        previous_send = None
        while True:
            # An idle timeout terminates the worker; never execute old work.
            request = receive_packet(sock, time.monotonic() + 3600)
            if request is None:
                return
            sequence, root, position, target, kwargs, memory, expires = request
            recorded_root = root
            candidates = []
            received = time.monotonic()
            trace = {"sequence": sequence, "worker_received_s": received,
                     "request_started_s": kwargs["deadline_started_s"],
                     "previous_reply_send": previous_send}
            planner.restore_execution_memory(memory)
            if time.monotonic() >= expires:
                plan = planner._safe_result(kwargs["deadline_started_s"], "deadline_missed")
            else:
                wall, cpu = time.monotonic(), time.thread_time()
                importer = getattr(bundle.runtime, 'import_planning_state', None)
                root = (importer(root) if importer is not None else
                        root.clone_to(bundle.runtime.device, dtype=bundle.runtime.dtype))
                trace.update(root_transfer_wall_s=time.monotonic()-wall,
                             root_transfer_cpu_s=time.thread_time()-cpu)
                wall, cpu = time.monotonic(), time.thread_time()
                plan = planner.plan(root, position, target,
                    **({'candidate_diagnostics': candidates.append} if evidence else {}), **kwargs)
                trace.update(solve_wall_s=time.monotonic()-wall,
                             solve_cpu_s=time.thread_time()-cpu)
            # Transport cleanup is not permission to execute a late plan.
            # Parent discards expired replies and never commits their memory.
            trace["worker_reply_ready_s"] = time.monotonic()
            previous_send = dict(sequence=sequence)
            send_packet(sock, (sequence, plan, planner.execution_memory(), trace),
                        time.monotonic() + 1.0, timing=previous_send)
            if evidence is not None:
                evidence.submit(dict(sequence=sequence,received_monotonic_s=received,
                    input_root=recorded_root,position=position,target=target,
                    planner_inputs=kwargs,input_execution_memory=memory,
                    candidate_ledgers=candidates,plan=plan,worker_trace=trace,
                    reply_send=previous_send.copy()))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            send_packet(sock, {"error": f"{type(error).__name__}: {error}"},
                        time.monotonic() + 0.1)
        except Exception:
            pass
    finally:
        try:
            if evidence is not None:
                evidence.close()
        finally:
            sock.close()


class ProcessPlanner(CatheterMppi):
    """Keep lightweight planner policy local; isolate each complete solve.

    No background request queue: the node's plan lock serializes calls. A
    reply timeout retains one pending sequence, drained and discarded before
    new work within the next solve's budget. Reset/capture never wait for IPC.
    """

    def __init__(self, backend, contract, config, *, manifest, device,
                 options, identity, threads=(4, 1)):
        super().__init__(backend, contract, config)
        parent, child = socket.socketpair()
        parent.setblocking(False)
        self._socket = parent
        self._reader = PacketReader(parent)
        self._pending_sequence = None
        self._sequence = 0
        self._failed = False
        self._memory_revision = 0
        self._observation_revision = 0
        self._memory_lock = threading.RLock()
        self.process_timings_s = {}
        self.process_trace = []
        self.last_worker_error = ""
        self.last_transport_failure = None
        self._worker = multiprocessing.get_context("spawn").Process(
            target=_worker, args=(child, manifest, device, options, identity,
                                  contract, config, threads), daemon=True)
        self._worker.start()
        child.close()
        try:
            reply = receive_packet(parent, time.monotonic() + 30)
            if reply != {"ready": True}:
                raise RuntimeError(f"planner worker initialization failed: {reply}")
        except Exception:
            self.close()
            raise

    def reset(self):
        with self._memory_lock:
            super().reset()
            self._memory_revision += 1

    def reset_capture_target(self):
        with self._memory_lock:
            super().reset_capture_target()
            self._memory_revision += 1

    def observe_capture_response(self, *args, **kwargs):
        with self._memory_lock:
            result = super().observe_capture_response(*args, **kwargs)
            self._observation_revision += 1
            if result:
                self._memory_revision += 1
            return result

    def note_capture_control_executed(self, command):
        with self._memory_lock:
            super().note_capture_control_executed(command)
            self._memory_revision += 1

    def plan(self, root_state, joint_position, target_tip_base_m, **kwargs):
        started = kwargs.get("deadline_started_s", time.monotonic())
        kwargs = dict(kwargs, deadline_started_s=started)
        self.process_timings_s = {}
        self.process_trace = []
        self.process_stage_marks = []
        self._mark_process_stage('entry')
        if self._failed or not self._worker.is_alive():
            return self._safe_result(started, "worker_unavailable")
        self.last_transport_failure = None
        # The existing discarded warmup is allowed a bounded cold budget.
        cold = not getattr(self, "_warmup_complete", False)
        expires = started + (30.0 if cold else self.config.planning_deadline_s)
        phase = "process_reply_drain"
        phase_started = time.monotonic()
        sending = False
        request_send_timing = {}
        self.process_timings_s['process_before_entry'] = max(
            0.0, time.monotonic()-started)
        try:
            if self._pending_sequence is not None:
                reply = self._reader.receive(expires)
                if not isinstance(reply, tuple) or reply[0] != self._pending_sequence:
                    raise ValueError(f"invalid pending planner worker reply: {reply}")
                self._record_reply(reply, discarded=True)
                self._pending_sequence = None
                self.process_timings_s[phase] = time.monotonic() - phase_started
            import torch
            # Both reviewed runtime families expose a tensor lambda view.
            # Experimental engagement snapshots do not have motor_angle_rad.
            if root_state.lambda_value.device.type != "cpu":
                raise ValueError("isolated planner requires a CPU estimator snapshot")
            phase = "process_root_clone"
            phase_started = time.monotonic()
            cpu_started = time.thread_time()
            exporter = getattr(self.backend, 'export_planning_state', None)
            root = (exporter(root_state) if exporter is not None else
                    root_state.clone_to(torch.device("cpu")))
            self.process_timings_s[phase] = time.monotonic()-phase_started
            self.process_timings_s['process_root_clone_cpu'] = time.thread_time()-cpu_started
            self._mark_process_stage('root_copied')
            phase = "process_memory_snapshot"
            phase_started = time.monotonic()
            cpu_started = time.thread_time()
            with self._memory_lock:
                memory = self.execution_memory()
                revision = self._memory_revision
                observation_revision = self._observation_revision
            self.process_timings_s[phase] = time.monotonic()-phase_started
            self.process_timings_s['process_memory_snapshot_cpu'] = time.thread_time()-cpu_started
            self._mark_process_stage('memory_copied')
            self._sequence += 1
            if time.monotonic() >= expires:
                return self._safe_result(started, "deadline_missed")
            phase = "process_request_send"
            phase_started = time.monotonic()
            self._mark_process_stage('before_send')
            sending = True
            send_packet(self._socket, (self._sequence, root, joint_position,
                        target_tip_base_m, kwargs, memory, expires), expires,
                        timing=request_send_timing)
            sending = False
            self._mark_process_stage('sent')
            for key in ('serialization_wall_s', 'serialization_cpu_s'):
                self.process_timings_s['process_request_'+key[:-2]] = request_send_timing[key]
            self._pending_sequence = self._sequence
            self.process_timings_s[phase] = time.monotonic() - phase_started
            phase = "process_reply_wait_decode"
            phase_started = time.monotonic()
            reply = self._reader.receive(expires)
            self._mark_process_stage('reply_decoded')
            self.process_timings_s[phase] = time.monotonic() - phase_started
            if not isinstance(reply, tuple) or reply[0] != self._sequence:
                raise ValueError(f"invalid planner worker reply: {reply}")
            self._pending_sequence = None
            _, plan, memory, _ = reply
            self._record_reply(reply, discarded=not cold and time.monotonic() >= expires)
            self.process_trace[-1]['parent_request_send'] = dict(request_send_timing)
            self.process_trace[-1]['parent_preparation_timings_s'] = dict(self.process_timings_s)
            self._warmup_complete = True
            if not cold and time.monotonic() >= expires:
                self.process_trace[-1]['discarded'] = True
                return self._safe_result(started, "deadline_missed")
            self.last_worker_error = ""
            with self._memory_lock:
                if self._memory_revision != revision:
                    # New target/reset/visual feedback invalidates the solve.
                    self.process_trace[-1]['discarded'] = True
                    return self._safe_result(started, "worker_state_changed")
                self.restore_execution_memory(
                    memory, capture=observation_revision == self._observation_revision)
                if observation_revision != self._observation_revision:
                    # Consistent visual evidence changed diagnostic fields,
                    # not policy. Keep it, but accept the selected hold expiry.
                    with self._capture_lock:
                        self._capture_hold_until_s = memory["capture"]["_capture_hold_until_s"]
            self._warmup_complete = True
            return self.enforce_deadline(plan, started)
        except TimeoutError as error:
            self._mark_process_stage('timeout')
            self.process_timings_s[phase] = time.monotonic() - phase_started
            self.last_worker_error = str(error)
            self.last_transport_failure = dict(sequence=self._sequence, phase=phase,
                phase_started_s=phase_started, failed_s=time.monotonic(),
                request_started_s=started, sending=sending,
                error_type=type(error).__name__, error=str(error),
                bytes_sent=getattr(error, 'bytes_sent', None),
                preparation_timings_s=dict(self.process_timings_s),
                request_send_timing=dict(request_send_timing))
            if sending and not (
                    isinstance(error, PacketSendTimeout) and error.bytes_sent == 0):
                # A partial request cannot be safely replayed or reframed.
                self._failed = True
                self._socket.close()
                return self._safe_result(started, "worker_failure")
            return self._safe_result(started, "deadline_missed")
        except Exception as error:
            self.process_timings_s[phase] = time.monotonic()-phase_started
            self.last_worker_error = f"{type(error).__name__}: {error}"
            self.last_transport_failure = dict(sequence=self._sequence, phase=phase,
                phase_started_s=phase_started, failed_s=time.monotonic(),
                request_started_s=started, sending=sending,
                error_type=type(error).__name__, error=str(error))
            self._failed = True
            self._socket.close()
            return self._safe_result(started, "worker_failure")

    def _mark_process_stage(self, stage):
        """Bounded request-local marks; no policy or deadline changes."""
        self.process_stage_marks.append(dict(stage=stage, monotonic_s=time.monotonic(),
            thread_cpu_s=time.thread_time(), native_thread_id=threading.get_native_id()))

    def close(self):
        self._failed = True
        self._socket.close()
        if os.environ.get('CR_MPPI_EVIDENCE_DIR'):
            # EOF lets the diagnostic writer finish at shutdown, not in a
            # control callback. Termination below remains a bounded fallback.
            self._worker.join(timeout=2)
        if self._worker.is_alive():
            self._worker.terminate()
        self._worker.join(timeout=2)
        if self._worker.is_alive():
            self._worker.kill()
            self._worker.join(timeout=2)

    def _record_reply(self, reply, *, discarded):
        """Timing survives rejection; it never restores expired policy memory.

        Monotonic clocks share the host epoch. Reply-ready to decoded includes
        serialization, transport, scheduling and decode, not GPU kernel time.
        """
        trace = dict(reply[3], parent_decoded_s=time.monotonic(), discarded=discarded,
                     **self._reader.decode_timing)
        trace["request_to_worker_ms"] = 1000 * (
            trace["worker_received_s"] - trace["request_started_s"])
        trace["reply_ready_to_decoded_ms"] = 1000 * (
            trace["parent_decoded_s"] - trace["worker_reply_ready_s"])
        self.process_trace.append(trace)

    def worker_diagnostics(self):
        """Non-blocking status; never wait for a child response in diagnostics."""
        return {"planner_worker_pid": str(self._worker.pid),
                "planner_worker_alive": str(self._worker.is_alive()),
                "planner_worker_pending_sequence": str(self._pending_sequence),
                "planner_worker_error": self.last_worker_error,
                "planner_request_preparation": json.dumps(dict(
                    timings_s=self.process_timings_s)),
                "planner_transport_failure": json.dumps(self.last_transport_failure)}
