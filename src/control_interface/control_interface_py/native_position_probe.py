"""Bounded qualification of the existing driver-native POS transaction path.

Tekceleo Protocol Guide p10 defines A as a relative angular move; p8 defines
O0 stop. Legacy replacement is explicit stop/standstill then a fresh POS move.
The separate default-off live-update diagnostic observes one A-only update
under an existing guarded stream lease. No raw UART/device-service bypass.
"""
import time


def classify_bracketed_update(anchor, before, after, final, control_endpoint=None):
    """Unknown driver receipt time is an interval, not the old cached encoder."""
    low, high = sorted((before, after))
    hypotheses = {'replace_original_reference': [anchor+100]*2,
                  'replace_current_reference': [low+100, high+100],
                  'queued_increment': [anchor+500]*2}
    if control_endpoint is not None:
        hypotheses['ignored_matched_control'] = [control_endpoint]*2
    matches = [name for name, (lo, hi) in hypotheses.items() if lo-15 <= final <= hi+15]
    return (matches[0] if len(matches) == 1 and control_endpoint is not None else 'ambiguous_or_other'), hypotheses


def run_live_update_probe(node, state, report, anchor, goal, fresh, spin, goal_pub, live_pub,
                          control=False, control_report=None):
    from control_interface.msg import DeviceStream
    from control_interface_py.streamed_position import LIVE_UPDATE
    sent = False
    update_wait_started = None
    report['scope'] = 'matched no-A control' if control else 'one live A4.5; original lease unchanged'
    report['live_bracket_schema'] = 1
    while node.get_clock().now().nanoseconds < goal.deadline_ros_ns:
        fresh()
        if not -2 <= state['enc'][0]-anchor[0] <= 667 or any(
                abs(state['enc'][a]-anchor[a]) > 2 for a in range(1, 6)):
            raise RuntimeError('live probe travel envelope exceeded')
        selected = [e for e in state['events'] if len(e['data']) >= 18
                    and int(e['data'][0]) == goal.session_id and int(e['data'][1]) == goal.command_id]
        if not sent and state['enc'][0]-anchor[0] >= 5:
            if not any(e['text'] == 'POSITION_STREAM:NATIVE_STARTED' for e in selected):
                raise RuntimeError('motion without ACK-qualified native start evidence')
            if state['enc'][0]-anchor[0] >= 300:
                raise RuntimeError('first move too far advanced; scenario not exercised')
            update = DeviceStream()
            update.header.frame_id = 'autonomy'
            update.header.stamp = node.get_clock().now().to_msg()
            update.predicate, update.data = LIVE_UPDATE, [float(goal.session_id), float(goal.command_id), 0. if control else 45.]
            report['live_update_requested_counts'] = list(state['enc'])
            report['live_update_request_ns'] = node.get_clock().now().nanoseconds
            live_pub.publish(update) # one shot; no retries or lease renewal
            sent = True
            update_wait_started = time.monotonic()
        if sent and update_wait_started is not None:
            # X and Y are separate DDS topics with one shared source-stamp
            # gate. A newer Y must not overtake the one-shot X at either relay.
            tx = [t for t in state['tx'] if t['predicate'] == LIVE_UPDATE
                  and len(t.get('data', ())) == 3
                  and int(t['data'][0]) == goal.session_id and int(t['data'][1]) == goal.command_id]
            completed = any(e['text'] == 'POSITION_STREAM:LIVE_UPDATED' for e in selected)
            if tx and completed:
                report['live_update_ordering_wait_ms'] = 1000*(time.monotonic()-update_wait_started)
                update_wait_started = None
            elif time.monotonic()-update_wait_started >= .075:
                raise RuntimeError('live update dispatch/completion timeout (75 ms); no retry or lease renewal')
            else:
                spin(.005)
                continue
        goal.header.stamp = node.get_clock().now().to_msg()
        goal_pub.publish(goal) # immutable original heartbeat only
        spin(.005)
    if update_wait_started is not None:
        raise RuntimeError('original lease expired while waiting for live update evidence; no retry')
    spin(.05)
    fresh()
    stopped = tuple(state['enc'])
    spin(.2)
    fresh()
    final = tuple(state['enc'])
    updates = [e for e in state['events'] if e['text'] == 'POSITION_STREAM:LIVE_UPDATED'
               and len(e['data']) >= 18 and int(e['data'][0]) == goal.session_id
               and int(e['data'][1]) == goal.command_id]
    tx = [t for t in state['tx'] if t['predicate'] == LIVE_UPDATE]
    if not sent or len(updates) != 1 or len(tx) != 1:
        raise RuntimeError('live update lacked unique manager TX and driver ACK evidence; do not infer semantics')
    brackets = [[e for e in state['events'] if e['text'] == label and len(e['data']) >= 18
                 and int(e['data'][0]) == goal.session_id and int(e['data'][1]) == goal.command_id]
                for label in ('POSITION_STREAM:LIVE_BEFORE_WRITE', 'POSITION_STREAM:LIVE_AFTER_ACK')]
    if any(len(events) != 1 for events in brackets):
        raise RuntimeError('missing unique fresh encoder write/ACK bracket; firmware reflash required')
    before, after = (int(events[0]['data'][6]) for events in brackets)
    # Before/after are fresh hardware samples, not cached counts in an ACK event.
    ack_counts = int(updates[0]['data'][6])
    control_endpoint = None
    if control_report is not None:
        control_endpoint = anchor[0]+control_report['final_counts'][0]-control_report['anchor_counts'][0]
    classification, hypotheses = classify_bracketed_update(anchor[0], before, after, final[0], control_endpoint)
    if control:
        classification = 'matched_no_update_control'
    stable = all(abs(x-y) <= 2 for x, y in zip(stopped, final))
    bounded = -2 <= final[0]-anchor[0] <= 667 and all(abs(final[a]-anchor[a]) <= 2 for a in range(1, 6))
    expired = any(e['text'] == 'POSITION_STREAM:EXPIRED' and len(e['data']) >= 18
                  and int(e['data'][0]) == goal.session_id and int(e['data'][1]) == goal.command_id
                  for e in state['events'])
    report.update(final_counts=list(final), post_deadline_counts=list(stopped),
                  live_update_ack_counts=ack_counts, endpoint_hypotheses=hypotheses,
                  live_semantics=classification, live_update_encoder_bracket=[before, after],
                  live_update_device_time_bracket_us=[events[0]['data'][2] for events in brackets],
                  matched_control_endpoint=control_endpoint,
                  live_update_encoder_degrees_bracket=[x*DEGREES_PER_COUNT for x in (before, after)],
                  endpoint_delta_from_control_counts=None if control_endpoint is None else final[0]-control_endpoint)
    report['passed'] = bool(stable and bounded and expired and classification != 'ambiguous_or_other')
    report['message'] = 'bounded live semantics observation: '+classification+'; not qualification for continuous retargeting'

from control_interface_py.encoder_coordinates import (
    DEGREES_PER_COUNT, UNITS_PER_SECOND_PER_RPM, counts_to_logical)


def native_offsets(mode):
    if mode == 'endpoint' or mode == 'cancel':
        return (400,)
    if mode == 'replacement':
        return (400, 600)
    if mode == 'reversal':
        return (400, -200)
    raise ValueError('native position supports endpoint/cancel/replacement/reversal, not expiry')


def native_target(anchor, offset):
    counts = [int(x) for x in anchor]
    counts[0] += offset
    if not all(-2147483648 <= x <= 2147483647 for x in counts):
        raise ValueError('native target outside int32 encoder range')
    return counts, [float(x) for x in counts_to_logical(counts)]


def relative_angle_counts(current, target):
    """Document legacy firmware's integer-degree roundf quantization."""
    degrees = abs(target-current)*DEGREES_PER_COUNT
    # C++ roundf is half-away-from-zero; magnitude is nonnegative.
    integer_degrees = int(degrees+.5)
    return integer_degrees, integer_degrees/DEGREES_PER_COUNT


def native_score(mode, anchor, target, stop_counts, final, cancelled):
    still = (all(abs(x-y) <= 2 for x, y in zip(stop_counts, final))
             and all(abs(final[a]-anchor[a]) <= 2 for a in range(1, 6))
             and abs(final[0]-anchor[0]) <= 667)
    if mode == 'cancel':
        return bool(still and cancelled and abs(final[0]-target[0]) > 67
                    and final[0]-anchor[0] >= 5)
    return bool(still and abs(final[0]-target[0]) <= 67
                and (mode == 'endpoint' or cancelled))


def run_native_probe(node, args, state, report, anchor, fresh, spin, event,
                     goal_pub):
    # Imports remain at the ROS composition boundary; pure calculations above
    # are importable by offline tests.
    from control_interface.msg import ControlStream, ManagerEvent
    offsets = native_offsets(args.mode)
    report['scope'] = 'driver-native POS with existing firmware supervisor; no live A replacement'
    report['planned_offsets_counts'] = list(offsets)
    report['native_commands'] = []
    report['cancelled_before_target'] = False
    cancelled = False

    def checked():
        fresh()
        current = state['enc']
        if abs(current[0]-anchor[0]) > 667 or any(
                abs(current[a]-anchor[a]) > 2 for a in range(1, 6)):
            raise RuntimeError('native probe travel envelope exceeded')
        if any(e['text'] in ('POSITION_REJECTED', 'POSITION_TIMED_OUT')
               for e in state['events']):
            raise RuntimeError('native firmware rejected/timed out transaction')

    def stopped():
        # Caller stops target publication before sending STOP. Never resend the
        # cancelled target as a heartbeat during stationary verification.
        event(ManagerEvent.STOP_MOTOR)
        report.setdefault('stop_requests', []).append({
            'ros_ns': node.get_clock().now().nanoseconds, 'counts': list(state['enc'])})
        end = time.monotonic()+.35
        stable_since, candidate = None, None
        while time.monotonic() < end:
            spin(.01)
            checked()
            current = state['enc']
            if candidate is None or any(abs(x-y) > 2 for x, y in zip(candidate, current)):
                candidate, stable_since = current, time.monotonic()
            if time.monotonic()-stable_since >= .1:
                return tuple(current)
        raise RuntimeError('stop failed to establish 100 ms encoder standstill')

    for index, offset in enumerate(offsets):
        checked()
        before = tuple(state['enc'])
        target, logical = native_target(anchor, offset)
        deg, quantized = relative_angle_counts(before[0], target[0])
        entry = {'index': index+1, 'root_counts': list(before), 'target_counts': target,
                 'logical_target': logical, 'predicted_driver_degrees': deg,
                 'predicted_quantized_travel_counts': quantized,
                 'start_ros_ns': node.get_clock().now().nanoseconds}
        report['native_commands'].append(entry)
        event(ManagerEvent.MODE, chr(ManagerEvent.JOINT_POS))
        spin(.03)
        checked()
        if any(abs(x-y) > 2 for x, y in zip(before, state['enc'])):
            raise RuntimeError('encoder anchor moved before native command')
        command = ControlStream()
        command.header.frame_id = 'autonomy'
        command.joint_pos = logical
        command.joint_vel = [float(args.rpm*UNITS_PER_SECOND_PER_RPM[0]), 0., 0., 0., 0., 0.]
        # Do not allow the existing 500 ms correction/retry policy to determine
        # the outcome. The driver's first native move must settle in this window.
        deadline = time.monotonic()+.35
        intentional_cancel = index == 0 and args.mode != 'endpoint'
        event_start = len(state['events'])
        tx_start = len(state['tx'])
        exercised = False
        while time.monotonic() < deadline:
            checked()
            current = state['enc']
            if intentional_cancel and current[0]-before[0] >= 5:
                if abs(current[0]-target[0]) <= 67 or any(
                        e['text'] == 'POSITION_COMPLETE' for e in state['events'][event_start:]):
                    raise RuntimeError('native move finished before cancellation; scenario not exercised')
                report['cancel_counts'] = list(current)
                exercised = True
                break
            if not intentional_cancel and any(
                    e['text'] == 'POSITION_COMPLETE' for e in state['events'][event_start:]):
                exercised = True
                break
            command.header.stamp = node.get_clock().now().to_msg()
            report['executed'] = True
            goal_pub.publish(command)
            spin(.01)
        traces = [t for t in state['tx'][tx_start:] if t['predicate'] == ord('P')]
        expected = logical+[float(x) for x in command.joint_vel]
        if not traces or any(len(t['data']) != 12 or any(
                abs(t['data'][a]-expected[a]) > 1e-5 for a in range(12)) for t in traces):
            raise RuntimeError('no faithful native POS transmission evidence (missing/clipped target)')
        if not exercised:
            raise RuntimeError('native scenario did not complete in bounded first-move window')
        stop_counts = stopped()
        entry['stopped_counts'] = list(stop_counts)
        entry['stop_confirmed_ros_ns'] = node.get_clock().now().nanoseconds
        if intentional_cancel:
            if abs(stop_counts[0]-target[0]) <= 67:
                raise RuntimeError('cancellation did not stop short of original target')
            cancelled = True
            report['cancelled_before_target'] = True
        elif abs(stop_counts[0]-target[0]) > 67:
            raise RuntimeError('driver-native endpoint outside tolerance')
        if index+1 < len(offsets):
            # The next target is anchored to freshly observed stopped counts.
            # Retained report target offsets are absolute relative to the
            # original probe anchor, never accumulated requested increments.
            report['replacement_stationary_counts'] = list(stop_counts)
    spin(.15)
    checked()
    final = tuple(state['enc'])
    report.update(final_counts=list(final), post_stop_counts=list(stop_counts),
                  final_target_counts=target)
    report['passed'] = native_score(args.mode, anchor, target, stop_counts, final, cancelled)
    report['message'] = ('native '+args.mode+' check' if report['passed']
                         else 'native endpoint/cancellation/standstill check failed')
