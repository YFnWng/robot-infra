"""Plan-anchored bounded endpoints; no ROS or actuator access."""
from dataclasses import dataclass
import numpy as np
from control_interface_py.streamed_position import StreamedPositionCommand
from .hardware_contract import MOTOR_AXIS_UNITS_PER_ENCODER_COUNT


def measured_standstill(history, after_ns):
    """100 ms continuous, six-axis raw evidence; no gap above 30 ms."""
    samples = [(t, q) for t, q in history if t >= after_ns]
    if not samples:
        return False
    end = samples[-1][0]
    start = next((i for i, (t, _) in enumerate(samples)
                  if end-t <= 100_000_000), len(samples)-1)
    window = samples[max(0, start-1):]
    counts = np.asarray([q for _, q in window])
    return bool(end-window[0][0] >= 100_000_000
        and all(0 < b[0]-a[0] <= 30_000_000 for a, b in zip(window, window[1:]))
        and counts.shape[1:] == (6,) and np.all(np.isfinite(counts))
        and np.all(np.ptp(counts, axis=0) < 2))


@dataclass(frozen=True)
class PositionExecution:
    command: StreamedPositionCommand
    deadline_ros_ns: int
    velocity: tuple[float, ...]


def position_execution_in_flight(execution, now_ros_ns):
    """Do not stop/rebuild a native first-step move on every planner tick."""
    return bool(execution is not None and execution.command.active_mask
                and now_ros_ns < execution.deadline_ros_ns)


def root_encoder_counts(history, timestamp_ns):
    """Match the processed encoder root, allowing only float-clock rounding."""
    if not history:
        raise ValueError('position execution has no root encoder history')
    stamp, counts = min(history, key=lambda entry: abs(entry[0]-timestamp_ns))
    if abs(stamp-timestamp_ns) > 2000 or np.asarray(counts).shape != (6,):
        raise ValueError('position execution root has no aligned six-axis encoder sample')
    return np.asarray(counts, dtype=np.float64).copy()


def make_position_execution(contract, command_id, anchor_counts, current_counts,
                            logical_velocity, step_s, now_ros_ns, *, integer_degrees=False,
                            authority_s=None, preserve_displacement=False):
    if not np.isfinite(step_s) or not 0 < step_s <= .25:
        raise ValueError('streamed-position first step must be in (0, 250 ms]')
    authority_s = step_s if authority_s is None else authority_s
    if not np.isfinite(authority_s) or not step_s <= authority_s <= .25:
        raise ValueError('position authority must cover the step and not exceed 250 ms')
    anchor = np.asarray(anchor_counts, dtype=np.float64)
    current = np.asarray(current_counts, dtype=np.float64)
    if anchor.shape != (6,) or current.shape != (6,) or not np.all(np.isfinite([anchor, current])):
        raise ValueError('position execution requires six finite raw encoder counts')
    if np.any(anchor != np.rint(anchor)) or np.any(current != np.rint(current)):
        raise ValueError('position execution requires integer encoder counts')
    projection = contract.project_velocity(logical_velocity,
                                           contract.encoder_counts_to_logical_position(anchor))
    motor_velocity = (contract.logical_position_to_motor_axis_position(logical_velocity)
                      if preserve_displacement else projection.realized_motor_axis_velocity)
    delta = motor_velocity*step_s/MOTOR_AXIS_UNITS_PER_ENCODER_COUNT
    if integer_degrees:
        # Installed driver A parser behaves as integer degrees. Explicitly
        # quantize before limit checks, rather than silently losing fractions.
        delta = np.rint(delta*.045)/.045
    target = np.rint(anchor+delta)
    if preserve_displacement:
        # Match the native backend's three-degree endpoint tolerance. Do not
        # start acquisition for an endpoint already inside that tolerance.
        target[np.abs(target-anchor) <= 67] = anchor[np.abs(target-anchor) <= 67]
    # If preceding motion already passed a sampled endpoint, do not generate
    # a reverse correction that the planner never sampled.
    moving = (target-current)*delta > 0
    target[~moving] = current[~moving]
    if preserve_displacement and contract.model_encoder_count_lower is not None:
        lower = contract.model_encoder_count_lower+contract.model_encoder_count_margin
        upper = contract.model_encoder_count_upper-contract.model_encoder_count_margin
        outward = ((target[:3] < lower) & (target[:3] < current[:3])
                   | (target[:3] > upper) & (target[:3] > current[:3]))
        if np.any(outward):
            raise ValueError('position endpoint crosses operational encoder reserve')
    endpoint = contract.encoder_counts_to_logical_position(target)
    if (np.any(endpoint < contract.control_position_lower)
            or np.any(endpoint > contract.control_position_upper)
            or not contract.position_is_valid(endpoint)
            or not contract.model_encoder_counts_are_valid(target)):
        raise ValueError('position execution endpoint violates calibrated limits')
    mask = sum(1 << a for a in range(6) if moving[a])
    if preserve_displacement:
        # Speed is independent of distance. A physical-shaft speed floor does
        # not change its endpoint or activate another coupled shaft.
        from .hardware_contract import MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM
        requested_rpm = np.abs(motor_velocity/MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM)
        floor = np.ceil(contract.velocity_min/np.abs(MOTOR_AXIS_UNITS_PER_SECOND_PER_RPM))
        speed = np.minimum(250, np.maximum(floor, np.ceil(requested_rpm)))
    else:
        speed = projection.motor_rpm
    rpm = tuple(int(speed[a]) if moving[a] else 0 for a in range(6))
    duration = int(round(authority_s*1e6))
    command = StreamedPositionCommand(command_id, duration, mask,
                                     tuple(int(x) for x in target), rpm)
    return PositionExecution(command, int(now_ros_ns)+duration*1000,
                             tuple(float(x) for x in logical_velocity))
