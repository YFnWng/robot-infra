"""Measured-tip AdapJ decision/update owner; no ROS or actuator interfaces."""
from collections import deque
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np

from control.adapj import AdapJController
from ..safety.hardware_contract import HardwareContract, MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
from ..safety.position_execution import measured_standstill


def initialization_shaft_step(position, axis, distance_mm, contract):
    """Isolate one physical shaft using the authoritative coupling conversion."""
    if axis not in (0, 2) or not np.isfinite(distance_mm) or distance_mm == 0:
        raise ValueError("calibration requires nonzero finite chassis/tendon travel")
    motor = HardwareContract.logical_position_to_motor_axis_position(position)
    motor[axis] += distance_mm
    target = HardwareContract.motor_axis_position_to_logical_position(motor)
    if not contract.position_is_valid(target):
        raise RuntimeError("compensated initialization reached a hard joint limit")
    return target


def initialization_response_confirmed(values, axis, direction, since_ns, now_ns, *,
                                      provisional=False, acquisition_evidence_ns=None):
    """Fresh onset, or retained ENGAGED proof from this acquisition leg.

    Retained proof is used only after the caller observed fresh directional
    onset and verified a stopped handoff. The caller must still enforce fresh
    controller/encoder/marker/manager status. A stationary producer need not
    emit a new engagement event on every status publication.
    """
    try:
        phase = json.loads(values["backlash_phase"])[axis]
        observed = json.loads(values["backlash_engaged_direction"])[axis]
        stamp = json.loads(values["backlash_last_evidence_timestamp_ns"])[axis]
        expected = int(np.sign(direction/MOTOR_AXIS_UNITS_PER_ENCODER_COUNT[axis]))
        phases = ("ENGAGED", "PROVISIONAL") if provisional else ("ENGAGED",)
        if not (phase in phases and observed == expected and since_ns < stamp <= now_ns):
            return False
        if acquisition_evidence_ns is not None:
            return (not provisional and phase == "ENGAGED"
                    and since_ns < acquisition_evidence_ns <= stamp)
        return now_ns-stamp <= 250_000_000
    except (KeyError, ValueError, TypeError, IndexError):
        return False


class TipAdapJ:
    axes = np.array([0, 2])

    def __init__(self, contract, step_s, reference_step_mm=1., *,
                 exclude_takeup_from_initialization=False, position_mode=False):
        if not np.isfinite(step_s) or step_s <= 0 or not np.isfinite(reference_step_mm) or reference_step_mm <= 0:
            raise ValueError("AdapJ steps must be finite and positive")
        self.contract = contract
        self.step_s = float(step_s)
        self.reference_step_m = reference_step_mm / 1000.
        self.exclude_takeup_from_initialization = bool(exclude_takeup_from_initialization)
        self.position_mode = bool(position_mode)
        self.controller = None
        self.capturing = False
        self.samples = deque(maxlen=3000)
        self.initialization_fit_allowed = deque(maxlen=3000)
        self.latest = None
        self.pending_command_ns = None
        self.execution_started_ns = None
        self.execution_stopped_ns = None
        self.last_reason = "initialization_required"
        self.updates = 0
        self.deferred_observation = None
        self.resume_after_ns = None
        self.position_deadline_ns = None

    @property
    def initialized(self):
        return self.controller is not None and self.controller.initialized

    def begin(self):
        self.controller = None
        self.samples.clear()
        self.initialization_fit_allowed.clear()
        self.latest = None
        self.deferred_observation = None
        self.resume_after_ns = None
        self.pending_command_ns = None
        self.execution_started_ns = None
        self.execution_stopped_ns = None
        self.capturing = True
        self.position_deadline_ns = None
        self.last_reason = "initialization_recording"

    def observe(self, timestamp_ns, tip, encoder_history, *, update_allowed,
                initialization_history=None):
        """Pair by acquisition/source time; never use receipt-time POS."""
        deferred = self.deferred_observation
        self.deferred_observation = None
        if deferred is not None and timestamp_ns > deferred[0]:
            self._observe_aligned(deferred[0], deferred[1], encoder_history,
                                  update_allowed=update_allowed and deferred[2],
                                  initialization_history=initialization_history)
        before = next(((t, q) for t, q in reversed(encoder_history) if t <= timestamp_ns), None)
        after = next(((t, q) for t, q in encoder_history if t >= timestamp_ns), None)
        if after is None and before is not None and 0 <= timestamp_ns-before[0] <= 150_000_000:
            # Zero-latency simulated capture can precede the next encoder tick.
            # Keep one bounded observation for the next callback to bracket;
            # never substitute an unaligned receipt-time position.
            self.deferred_observation = (int(timestamp_ns), np.asarray(tip).copy(), update_allowed)
            self.last_reason = "awaiting_encoder_bracket"
            return
        self._observe_aligned(timestamp_ns, tip, encoder_history, update_allowed=update_allowed,
                              initialization_history=initialization_history)

    def _initialization_interval_allowed(self, timestamp_ns, previous, history):
        """Exclude known acquisition across the complete source-time interval.

        Read the shared owner's checkpoints; do not run a second engagement
        detector. UNKNOWN is not engagement proof, but is not labeled takeup
        either (external chassis moves retain UNKNOWN in that owner).
        """
        if not self.exclude_takeup_from_initialization:
            return True
        entries = [entry for entry in (history or ()) if entry[0] <= timestamp_ns]
        start = timestamp_ns if previous is None else previous[0]
        anchor = next((i for i in range(len(entries)-1, -1, -1)
                       if entries[i][0] <= start), None)
        if anchor is None or timestamp_ns-entries[-1][0] > 150_000_000:
            return False
        window = entries[anchor:]
        return all(bool(entry[3]) == bool(window[0][3])
                   and not any(phase in ("TAKEUP", "PROVISIONAL", "FAILED")
                               for phase in entry[2]["phase"])
                   for entry in window)

    def _observe_aligned(self, timestamp_ns, tip, encoder_history, *, update_allowed,
                         initialization_history=None):
        if self.latest is not None and timestamp_ns <= self.latest[0]:
            return
        before = next(((t, q) for t, q in reversed(encoder_history) if t <= timestamp_ns), None)
        after = next(((t, q) for t, q in encoder_history if t >= timestamp_ns), None)
        if before is None or after is None or after[0]-before[0] > 150_000_000:
            self.invalidate()
            self.latest = None
            self.last_reason = "unbracketed_observation"
            return
        fraction = 0. if after[0] == before[0] else (timestamp_ns-before[0])/(after[0]-before[0])
        counts = (1-fraction)*np.asarray(before[1])+fraction*np.asarray(after[1])
        q = HardwareContract.encoder_counts_to_logical_position(counts)[self.axes]
        tip = np.asarray(tip, dtype=float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)) or not np.all(np.isfinite(q)):
            raise ValueError("invalid AdapJ observation")
        old = self.latest
        self.latest = (int(timestamp_ns), tip.copy(), q.copy())
        gap = old is not None and timestamp_ns-old[0] > 150_000_000
        if self.capturing:
            if len(self.samples) == self.samples.maxlen:
                self.capturing = False
                raise ValueError("AdapJ initialization sample budget exhausted")
            self.samples.append(self.latest)
            self.initialization_fit_allowed.append(
                self._initialization_interval_allowed(timestamp_ns, old, initialization_history))
        if not self.initialized:
            return
        s, a = self.coordinates(tip, q)
        if not update_allowed or gap:
            self.invalidate()
            self.controller.seed_history(s, a)
            self.last_reason = "update_skipped"
        elif (self.pending_command_ns is not None
              and self.execution_stopped_ns is not None
              and timestamp_ns >= self.execution_stopped_ns):
            self.controller.update(s, a)
            if not all(np.all(np.isfinite(value)) for value in self.controller.matrices.values()):
                raise ValueError("nonfinite AdapJ parameters")
            self.pending_command_ns = None
            self.position_deadline_ns = None
            self.execution_started_ns = None
            self.updates += 1
            self.last_reason = "updated"
        elif self.pending_command_ns is None:
            self.controller.seed_history(s, a)

    def coordinates(self, tip, q):
        return (np.asarray(tip)-self.tip_origin)/self.tip_scale, (np.asarray(q)-self.q_origin)/self.q_scale

    def finish(self, directory):
        self.capturing = False
        if len(self.samples) < 20:
            raise ValueError("AdapJ requires at least 20 aligned initialization samples")
        stamps = np.array([item[0] for item in self.samples], dtype=np.int64)
        tips = np.array([item[1] for item in self.samples])
        actions = np.array([item[2] for item in self.samples])
        allowed = np.asarray(self.initialization_fit_allowed, dtype=bool)
        if np.count_nonzero(allowed) < 20:
            raise ValueError("AdapJ requires at least 20 eligible initialization samples outside takeup")
        valid = ((np.diff(stamps) > 0) & (np.diff(stamps) <= 150_000_000)
                 & allowed[:-1] & allowed[1:])
        if np.count_nonzero(valid) < 19:
            raise ValueError("insufficient contiguous initialization transitions")
        first = np.flatnonzero(allowed)[0]
        self.tip_origin, self.q_origin = tips[first].copy(), actions[first].copy()
        self.tip_scale = np.maximum(np.std(tips[allowed], axis=0), .001)
        self.q_scale = np.maximum(np.std(actions[allowed], axis=0), .1)
        action_steps = np.diff(actions, axis=0)[valid]/self.q_scale
        singular = np.linalg.svd(action_steps, compute_uv=False)
        if singular[-1] < 1e-3 or singular[-1] < 1e-3*singular[0]:
            raise ValueError("initialization did not excite both axes")
        states, acts = self.coordinates(tips, actions)
        # Fit the same duration used for command pulses, not one camera tick.
        # Interpolate only within continuous, source-aligned calibration data.
        end_stamps = stamps+round(self.step_s*1e9)
        right = np.searchsorted(stamps, end_stamps)
        starts = np.array([i for i, j in enumerate(right)
                           if j < len(stamps) and np.all(valid[i:j])], dtype=int)
        if len(starts) < 19:
            raise ValueError("insufficient initialization control intervals")
        ends = right[starts]
        fractions = ((end_stamps[starts]-stamps[ends-1])
                     /(stamps[ends]-stamps[ends-1]))[:, None]
        next_states = states[ends-1]*(1-fractions)+states[ends]*fractions
        next_acts = acts[ends-1]*(1-fractions)+acts[ends]*fractions
        controller = AdapJController(n_state=3, n_joints=2, dt=self.step_s,
            joint_lower=(self.contract.position_lower[self.axes]-self.q_origin)/self.q_scale,
            joint_upper=(self.contract.position_upper[self.axes]-self.q_origin)/self.q_scale,
            action_encoding="identity", ridge=.001, rho=1., max_delta=.1)
        forward = controller.initialize_transitions(
            next_states, states[starts], acts[starts], next_acts, forward_increment=True)
        controller.seed_history(states[-1], acts[-1])
        record = dict(schema_version=1, method="AdapJController", axes=self.axes.tolist(),
            timestamp_ns=stamps.tolist(), tip_m=tips.tolist(), achieved_logical_position=actions.tolist(),
            valid_transitions=valid.tolist(), tip_origin_m=self.tip_origin.tolist(),
            initialization_excludes_takeup=self.exclude_takeup_from_initialization,
            initialization_fit_allowed=allowed.tolist(),
            tip_scale_m=self.tip_scale.tolist(), q_origin=self.q_origin.tolist(), q_scale=self.q_scale.tolist(),
            ridge=.001, rho=1., max_delta=.1, reference_step_mm=1000*self.reference_step_m,
            action_encoding="identity", action_singular_values=singular.tolist())
        record["parameters"] = {name: value.tolist() for name, value in controller.matrices.items()}
        record['initialization_method'] = 'measured_forward_increment_inverse_seed'
        record['initialization_forward_jacobian_normalized'] = forward.tolist()
        record["control_interval_s"] = self.step_s
        record["fit_interval_start_ns"] = stamps[starts].tolist()
        record["fit_interval_end_ns"] = end_stamps[starts].tolist()
        implementation = Path(inspect.getfile(AdapJController)).resolve()
        record["implementation"] = dict(path=str(implementation),
            sha256=hashlib.sha256(implementation.read_bytes()).hexdigest())
        path = Path(directory)/"initialization.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x") as stream:
            json.dump(record, stream, indent=2, allow_nan=False)
        self.controller = controller
        self.pending_command_ns = None
        self.last_reason = "initialized"
        return path

    def invalidate(self):
        self.position_deadline_ns = None
        self.pending_command_ns = None
        self.execution_started_ns = None
        self.execution_stopped_ns = None
        self.deferred_observation = None
        if self.initialized:
            self.controller.reset()

    def resume_after(self, timestamp_ns):
        """Discard take-up history; require a post-handoff source-time sample."""
        self.invalidate()
        self.resume_after_ns = int(timestamp_ns)

    def interval_active(self, timestamp_ns):
        """Keep a proposed pulse only within its bounded publication interval."""
        if self.pending_command_ns is None or self.execution_stopped_ns is not None:
            return False
        start = (self.pending_command_ns if self.execution_started_ns is None
                 else self.execution_started_ns)
        return 0 <= timestamp_ns-start < round(self.step_s*1e9)

    def execution_velocity(self, velocity, timestamp_ns):
        """Bound heartbeat publication and mark the zero barrier for learning.

        This records controller output, not a firmware execution acknowledgement.
        Safety/recovery zero before the interval ends discards the partial step.
        Take-up owns output separately after invalidating the intercepted proposal.
        """
        if self.execution_stopped_ns is not None:
            return np.zeros(6)
        if self.pending_command_ns is None:
            return np.zeros(6)
        if self.execution_started_ns is None:
            if not np.any(velocity) or not self.interval_active(timestamp_ns):
                self.invalidate()
                return np.zeros(6)
            self.execution_started_ns = int(timestamp_ns)
        if timestamp_ns-self.execution_started_ns >= round(self.step_s*1e9):
            self.execution_stopped_ns = int(timestamp_ns)
            self.last_reason = "awaiting_completed_interval_response"
            return np.zeros(6)
        if not np.any(velocity):
            self.invalidate()
        return velocity

    def note_position_execution(self, deadline_ns, timestamp_ns):
        """Record immutable authority, not requested displacement as training data."""
        if self.pending_command_ns is None:
            return
        if self.position_deadline_ns is None:
            self.position_deadline_ns = int(deadline_ns)
            self.execution_started_ns = int(timestamp_ns)
        elif self.position_deadline_ns != int(deadline_ns):
            raise ValueError("AdapJ position deadline changed")

    def confirm_position_standstill(self, encoder_history):
        """After expiry, require 100 ms of measured six-shaft standstill.

        Learning subsequently uses only a marker observation acquired after
        this barrier and its interpolated measured encoders. No commanded
        endpoint or requested velocity is substituted for achieved motion.
        """
        if self.position_deadline_ns is None or self.execution_stopped_ns is not None:
            return
        if measured_standstill(encoder_history, self.position_deadline_ns):
            self.execution_stopped_ns = int(encoder_history[-1][0])
            self.last_reason = "awaiting_position_response_after_measured_standstill"

    def command(self, target, position, command_timestamp_ns):
        if not self.initialized:
            raise ValueError("AdapJ is not initialized")
        if self.latest is None or command_timestamp_ns-self.latest[0] > 150_000_000:
            return None
        if self.resume_after_ns is not None:
            if self.latest[0] <= self.resume_after_ns:
                self.last_reason = "post_takeup_observation_hold"
                return None
            self.resume_after_ns = None
        if self.pending_command_ns is not None:
            return None
        stamp, tip, achieved = self.latest
        delta = np.asarray(target)-tip
        length = np.linalg.norm(delta)
        reference = tip+delta*min(1., self.reference_step_m/max(length, 1e-12))
        state, action = self.coordinates(tip, achieved)
        ref, _ = self.coordinates(reference, achieved)
        # A new target or disarm can reset history before another camera sample
        # arrives. Seed the preceding action from this aligned measurement,
        # never from the normalization origin or an unachieved proposal.
        self.controller.seed_history(state, action)
        proposed = self.q_origin+self.q_scale*self.controller.compute(state, ref, stamp*1e-9)
        if not np.all(np.isfinite(proposed)):
            raise ValueError("nonfinite AdapJ action")
        velocity = np.zeros(6)
        velocity[self.axes] = (proposed-np.asarray(position)[self.axes])/self.step_s
        # Preserve the joint-space direction when any joint hits its speed cap.
        ratio = np.max(np.abs(velocity)/np.maximum(self.contract.velocity_max, 1e-12))
        velocity /= max(1., ratio)
        self.pending_command_ns = int(command_timestamp_ns)
        self.execution_started_ns = None
        self.execution_stopped_ns = None
        self.last_reason = "awaiting_causal_response"
        # Native endpoint distance must not inherit a minimum-speed floor.
        return (velocity if self.position_mode else
                self.contract.project_publishable_velocity(velocity, position))
