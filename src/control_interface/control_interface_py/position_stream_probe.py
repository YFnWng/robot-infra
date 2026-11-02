#!/usr/bin/env python3
"""Explicitly authorized, bounded axis-0 endpoint probe through the manager.

No serial access, power qualification, encoder reset, or automatic retry.
Run without --execute to inspect a stationary encoder anchor only.
"""
import argparse
import json
import math
import time
from pathlib import Path
from control_interface_py.encoder_coordinates import UNITS_PER_SECOND_PER_RPM

# Reviewed imricor_test axis-0 floor: 2 mm/s, rounded UP to physical RPM.
# This first hardware qualification helper deliberately supports that speed only.
PROBE_RPM = math.ceil(2.0 / float(UNITS_PER_SECOND_PER_RPM[0]))


def command_trace_subscription_qos():
    """Match the bridge's non-blocking BEST_EFFORT command evidence channel."""
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def validate_probe(delta, rpm, duration_us):
    if not 68 <= abs(delta) <= 200:
        raise ValueError('axis-0 delta must be 68..200 counts (above 67-count tolerance)')
    if rpm != PROBE_RPM or not 10000 <= duration_us <= 250000:
        raise ValueError(f'probe RPM must be {PROBE_RPM}; lifetime must be 10000..250000 us')


def encoder_counts(data):
    if len(data) != 6 or any(not math.isfinite(x) or int(x) != x
                            or not -2147483648 <= x <= 2147483647 for x in data):
        raise ValueError('expected six exact int32 encoder counts')
    return tuple(int(x) for x in data)


def probe_offsets(mode, delta):
    if mode in ('replacement', 'reversal'):
        if abs(delta) != 200:
            raise ValueError('two-command probes require initial offset of +/-200 counts')
        # Both targets remain inside +/-200 counts of the original anchor.
        return (delta, delta-(20 if delta > 0 else -20) if mode == 'replacement' else -delta//2)
    return (delta,)


def score_probe(mode, anchor, targets, stopped, final, events, session, command_id):
    selected = [e for e in events if len(e['data']) >= 18
                and int(e['data'][0]) == session]
    last = [e for e in selected if int(e['data'][1]) == command_id]
    failure = any(e['text'] in ('POSITION_STREAM:REJECTED', 'POSITION_STREAM:INHIBITED',
                               'POSITION_STREAM:NO_PROGRESS', 'POSITION_STREAM:INVALID_COMMAND',
                               'POSITION_STREAM:INVALID_FEEDBACK') for e in selected)
    residual = abs(final[0]-targets[0])
    stopped_ok = (all(abs(x-y) <= 2 for x, y in zip(final, stopped))
                  and abs(final[0]-anchor[0]) <= 267
                  and all(abs(final[a]-anchor[a]) <= 2 for a in range(1, 6))
                  and bool(last) and all(rpm == 0 for rpm in last[-1]['data'][12:18]))
    if mode == 'expiry':
        outcome = (any(e['text'] == 'POSITION_STREAM:EXPIRED' for e in last)
                   and residual > 67 and abs(final[0]-anchor[0]) > 2
                   and (final[0]-anchor[0])*(targets[0]-anchor[0]) > 0
                   and not any(e['text'] == 'POSITION_STREAM:COMPLETE' for e in last))
    else:
        outcome = any(e['text'] == 'POSITION_STREAM:COMPLETE' for e in last) and residual <= 67
    if mode in ('replacement', 'reversal'):
        outcome = outcome and any(e['text'] == 'POSITION_STREAM:SUPERSEDED'
                                  and int(e['data'][1]) == 1 for e in selected)
    return bool(stopped_ok and outcome and not failure)


def gate_diagnostics(state, monotonic_now, ros_now_ns):
    """Describe independent gates without conflating missing and stale data."""
    def receipt_age(key):
        return None if state[key] is None else (monotonic_now-state[key])*1000

    enc_age = receipt_age('enc_time')
    safety_age = receipt_age('safety_time')
    stamp = state.get('enc_stamp_ns')
    source_age = None if stamp is None else (ros_now_ns-stamp)/1000000
    reasons = []
    if state['enc'] is None:
        reasons.append('ENC_MISSING_OR_REJECTED')
    if enc_age is None or enc_age > 100:
        reasons.append('ENC_RECEIPT_STALE')
    if source_age is None or not 0 <= source_age <= 100:
        reasons.append('ENC_SOURCE_STALE_OR_FUTURE')
    if safety_age is None or safety_age > 250:
        reasons.append('SAFETY_RECEIPT_STALE')
    if state['safety'] != 'MANAGER_READY':
        reasons.append('MANAGER_NOT_READY')
    last_stamp = state.get('last_enc_stamp_ns')
    return {'ready': not reasons, 'reasons': reasons,
            'encoder_receipt_age_ms': enc_age, 'encoder_source_age_ms': source_age,
            'last_encoder_source_age_ms': None if last_stamp is None else
            (ros_now_ns-last_stamp)/1000000,
            'safety_receipt_age_ms': safety_age, 'safety_text': state['safety'],
            'encoder_frames_seen': state.get('enc_seen', 0),
            'encoder_frames_accepted': state.get('enc_accepted', 0),
            'last_encoder_rejection': state.get('enc_rejection')}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('encoder_stream', 'driver_position'), default='encoder_stream')
    parser.add_argument('--mode', choices=('endpoint', 'expiry', 'replacement', 'reversal', 'cancel', 'live-update', 'live-control'), default='endpoint')
    parser.add_argument('--compare-control', type=Path, help='matched live-control JSON, required to resolve ignored-vs-update evidence')
    parser.add_argument('--driver-state', action='store_true', help='read axis-0 DA/DS while stopped; no motion/session claim')
    parser.add_argument('--delta-counts', type=int)
    parser.add_argument('--rpm', type=int)
    parser.add_argument('--duration-us', type=int)
    parser.add_argument('--execute', action='store_true', help='authorize one physical endpoint')
    parser.add_argument('--output', type=Path, required=True, help='new JSON report file')
    parser.add_argument('--startup-timeout-s', type=float, default=10.,
                        help='non-actuating discovery/input wait, 1..30 seconds')
    args = parser.parse_args(argv)
    native = args.backend == 'driver_position'
    if args.driver_state and (args.execute or args.mode != 'endpoint' or native or args.compare_control):
        parser.error('--driver-state is a separate stopped-state query, without --execute or scenario options')
    live = args.mode in ('live-update', 'live-control')
    control_report = None
    if args.compare_control:
        if args.mode != 'live-update':
            parser.error('--compare-control requires live-update')
        control_report = json.loads(args.compare_control.read_text())
        if (not control_report.get('passed') or control_report.get('mode') != 'live-control'
                or control_report.get('rpm') != 36 or control_report.get('delta_counts') != 400
                or control_report.get('duration_us') != 250000 or control_report.get('live_bracket_schema') != 1):
            parser.error('comparison requires a passed matched 36-RPM live-control report')
        encoder_counts(control_report.get('anchor_counts', []))
        encoder_counts(control_report.get('final_counts', []))
    if live and (native or args.delta_counts is not None or args.duration_us is not None):
        parser.error('live-update requires encoder_stream and its fixed 400-count/250-ms scenario')
    if native and (args.duration_us is not None or args.delta_counts not in (None, 400)):
        parser.error('driver_position uses bounded fixed offsets and no streamed lease; omit delta/duration')
    if not native and args.mode == 'cancel':
        parser.error('cancel scenario requires --backend driver_position')
    args.delta_counts = args.delta_counts if args.delta_counts is not None else (400 if native or live else 100 if args.mode == 'endpoint' else 200)
    args.rpm = args.rpm if args.rpm is not None else PROBE_RPM
    args.duration_us = args.duration_us if args.duration_us is not None else (20000 if args.mode == 'expiry' else 250000)
    if native:
        from control_interface_py.native_position_probe import native_offsets
        try:
            offsets = native_offsets(args.mode)
        except ValueError as error:
            parser.error(str(error))
        if args.rpm != PROBE_RPM:
            parser.error(f'driver_position probe requires {PROBE_RPM} RPM')
    elif live:
        if args.rpm != PROBE_RPM:
            parser.error('live-update requires 36 RPM')
        offsets = (400,)
    else:
        validate_probe(args.delta_counts, args.rpm, args.duration_us)
        if args.mode != 'expiry' and args.duration_us < 100000:
            parser.error('short leases are reserved for the explicit expiry scenario')
        offsets = probe_offsets(args.mode, args.delta_counts)
    if not math.isfinite(args.startup_timeout_s) or not 1 <= args.startup_timeout_s <= 30:
        parser.error('--startup-timeout-s must be 1..30 seconds')
    # Fail before ROS/motion if the destination exists or cannot be created.
    report_file = args.output.open('x')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from control_interface.msg import (ControlStream, DeviceStream, DeviceEvent,
                                       ManagerEvent, StreamedPositionControl)
    from control_interface.srv import OpenPositionStream
    from control_interface.srv import DeviceCmd
    rclpy.init(args=[])
    node = Node('position_stream_probe')
    state = {'enc': None, 'enc_time': None, 'enc_stamp_ns': None,
             'last_enc_stamp_ns': None, 'enc_seen': 0, 'enc_accepted': 0,
             'enc_rejection': None, 'safety': '', 'safety_time': None, 'events': [], 'samples': [], 'tx': []}
    claimed = False
    report = {'executed': False, 'passed': False, 'backend': args.backend,
              'phase': 'execution' if args.execute else 'preview',
              'mode': args.mode, 'delta_counts': args.delta_counts,
              'rpm': args.rpm, 'duration_us': None if native else args.duration_us}
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def enc_callback(msg):
        if msg.predicate == DeviceStream.ENC:
            state['enc_seen'] += 1
            stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            state['last_enc_stamp_ns'] = stamp_ns
            age_ns = node.get_clock().now().nanoseconds-stamp_ns
            if not 0 <= age_ns <= 100000000:
                state['enc_rejection'] = 'source timestamp outside 0..100 ms'
                return
            try:
                state['enc'] = encoder_counts(msg.data)
            except ValueError as error:
                state['enc_rejection'] = str(error)
                return
            state['enc_time'] = time.monotonic()
            state['enc_stamp_ns'] = stamp_ns
            state['enc_accepted'] += 1
            state['enc_rejection'] = None
            if report['executed']:
                state['samples'].append({'stamp_ns': stamp_ns, 'counts': list(state['enc'])})

    def safety_callback(msg):
        state['safety'], state['safety_time'] = msg.text, time.monotonic()

    def event_callback(msg):
        state['events'].append({'receipt_ns': time.time_ns(), 'predicate': int(msg.predicate),
                                'text': msg.text, 'data': [float(x) for x in msg.data]})

    def tx_callback(msg):
        if report['executed']:
            state['tx'].append({'stamp_ns': msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                               'predicate': int(msg.predicate), 'data': [float(x) for x in msg.data]})

    subscriptions = [node.create_subscription(DeviceStream, '/device/state', enc_callback, 10),
                     node.create_subscription(ManagerEvent, '/manager/safety_status', safety_callback, qos),
                     node.create_subscription(DeviceEvent, '/device/event', event_callback, 100),
                     node.create_subscription(DeviceStream, '/device/command_tx', tx_callback,
                                              command_trace_subscription_qos())]
    event_pub = node.create_publisher(ManagerEvent, '/teleop/event', 10)
    zero_pub = node.create_publisher(ControlStream, '/teleop/control', 10)
    goal_pub = node.create_publisher(StreamedPositionControl, '/teleop/position_control', 10)
    live_pub = node.create_publisher(DeviceStream, '/teleop/native_live_update', 10)
    client = node.create_client(OpenPositionStream, '/manager/open_position_stream')
    state_client = node.create_client(DeviceCmd, '/manager/read_driver_state')

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., end-time.monotonic())))

    def diagnostics():
        return gate_diagnostics(state, time.monotonic(), node.get_clock().now().nanoseconds)

    def fresh():
        gates = diagnostics()
        report['gates'] = gates
        if not gates['ready']:
            raise RuntimeError('input gate failed: '+json.dumps(gates))

    def event(predicate, text=''):
        msg = ManagerEvent()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id, msg.predicate, msg.text = 'autonomy', predicate, text
        event_pub.publish(msg)

    try:
        startup_end = time.monotonic()+args.startup_timeout_s
        while time.monotonic() < startup_end:
            spin(.01)
            if diagnostics()['ready']:
                break
        fresh()
        if (node.count_publishers('/teleop/control') != 1
                or node.count_publishers('/teleop/position_control') != 1
                or (live and node.count_publishers('/teleop/native_live_update') != 1)
                or any(name in ('catheter_mppi', 'catheter_adapj')
                       for name, _ in node.get_node_names_and_namespaces())):
            raise RuntimeError('stop other command publishers/controllers before probing')
        anchor = state['enc']
        for _ in range(50):
            spin(.01)
            fresh()
            if any(abs(x-y) > 2 for x, y in zip(state['enc'], anchor)):
                raise RuntimeError('encoder standstill check failed')
        report['anchor_counts'] = list(anchor)
        if control_report is not None and any(abs(x-y) > 2 for x, y in zip(anchor, control_report['anchor_counts'])):
            raise RuntimeError('comparison requires the same six-axis encoder anchor; home first')
        targets = list(anchor)
        targets[0] += args.delta_counts
        encoder_counts(targets)
        report['target_counts'] = targets
        report['planned_offsets_counts'] = list(offsets)
        if args.driver_state:
            report.update(backend='driver_state', mode='stopped_readback', rpm=None, duration_us=None)
            report.pop('target_counts', None)
            report.pop('planned_offsets_counts', None)
            from control_interface_py.driver_evidence import parse_state
            if not state_client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('guarded read_driver_state service unavailable')
            request = DeviceCmd.Request()
            request.predicate, request.cmd = ManagerEvent.DRIVER_DIAGNOSTIC, '0'
            future = state_client.call_async(request)
            deadline = time.monotonic()+2.
            while not future.done() and time.monotonic() < deadline:
                spin(.01); fresh()
            if not future.done() or not future.result().success:
                raise RuntimeError('stopped-state query failed: '+str(future.result() if future.done() else 'timeout'))
            report['driver_state'] = parse_state(future.result().response)
            report['phase'] = 'readback'
            report['passed'] = True
            report['message'] = 'stopped-state DA/DS readback; no enable, mode claim or motion sent'
            spin(.15) # collect deferred, bounded UART evidence
        elif not args.execute:
            report['passed'] = True
            report['message'] = 'preview readiness passed; no session claim or motion sent'
        elif native:
            from control_interface_py.native_position_probe import run_native_probe
            claimed = True  # Ensure STOP on any exception after attempting mode claim.
            run_native_probe(node, args, state, report, anchor, fresh, spin, event, zero_pub)
        else:
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('stream handshake service unavailable')
            request = OpenPositionStream.Request()
            request.source = 'autonomy:live_probe' if live else 'autonomy'
            future = client.call_async(request)
            end = time.monotonic()+3.
            while not future.done() and time.monotonic() < end:
                spin(.01)
                fresh()
            if not future.done() or not future.result().success:
                raise RuntimeError('stream handshake failed: '+str(future.result() if future.done() else 'timeout'))
            session = future.result().session_id
            report['session_id'] = session
            event(ManagerEvent.MODE, chr(ManagerEvent.JOINT_VEL))
            claimed = True
            spin(.05)
            fresh()
            if any(abs(x-y) > 2 for x, y in zip(state['enc'], anchor)):
                raise RuntimeError('anchor moved during handshake')
            def make_goal(command_id, offset):
                msg = StreamedPositionControl()
                msg.header.frame_id = 'autonomy'
                msg.session_id, msg.command_id = session, command_id
                msg.duration_us, msg.active_mask = args.duration_us, 1
                endpoint = list(anchor)
                endpoint[0] += offset
                encoder_counts(endpoint)
                msg.target_counts, msg.speed_rpm = endpoint, [args.rpm, 0, 0, 0, 0, 0]
                msg.deadline_ros_ns = node.get_clock().now().nanoseconds+args.duration_us*1000
                report.setdefault('commands', []).append({'id': command_id, 'target_counts': endpoint,
                                                          'deadline_ros_ns': msg.deadline_ros_ns})
                return msg

            goal = make_goal(1, offsets[0])
            report['executed'] = True
            if live:
                from control_interface_py.native_position_probe import run_live_update_probe
                run_live_update_probe(node, state, report, anchor, goal, fresh, spin, goal_pub, live_pub,
                                      control=args.mode == 'live-control', control_report=control_report)
            while not live and node.get_clock().now().nanoseconds < goal.deadline_ros_ns:
                fresh()
                if abs(state['enc'][0]-anchor[0]) > 267 or any(
                        abs(state['enc'][a]-anchor[a]) > 2 for a in range(1, 6)):
                    raise RuntimeError('probe travel envelope exceeded; stopping')
                if goal.command_id == 1 and len(offsets) == 2:
                    first_events = [e for e in state['events'] if len(e['data']) >= 18
                                    and int(e['data'][0]) == session and int(e['data'][1]) == 1]
                    if first_events and first_events[-1]['text'] in ('POSITION_STREAM:COMPLETE', 'POSITION_STREAM:EXPIRED'):
                        raise RuntimeError('first goal ended before moving replacement; scenario not exercised')
                    if ((state['enc'][0]-anchor[0])*(1 if offsets[0] > 0 else -1) >= 5
                            and first_events and first_events[-1]['text'] == 'POSITION_STREAM:RUNNING'):
                        report['replacement_counts'] = list(state['enc'])
                        goal = make_goal(2, offsets[1])
                goal.header.stamp = node.get_clock().now().to_msg()
                goal_pub.publish(goal)  # same ID, endpoint and deadline; no renewal
                spin(.005)
            if len(offsets) == 2 and goal.command_id != 2:
                raise RuntimeError('no moving replacement occurred within first lease')
            # Do not cancel immediately: observe independent firmware expiry/endpoint stop.
            spin(.05)
            fresh()
            stopped = state['enc']
            spin(.2)
            fresh()
            final = state['enc']
            report.update(final_counts=list(final), post_deadline_counts=list(stopped),
                          final_target_counts=None if live else list(encoder_counts(goal.target_counts)))
            if not live:
                report['passed'] = score_probe(args.mode, anchor, goal.target_counts, stopped, final,
                                              state['events'], session, goal.command_id)
                report['message'] = args.mode+' and stop check' if report['passed'] else 'failed scenario/stop/telemetry check; do not escalate to MPPI'
    except (Exception, KeyboardInterrupt) as error:
        report['message'] = str(error)
    finally:
        report['final_gates'] = diagnostics()
        report['events'] = state['events']
        report['encoder_samples'] = state['samples']
        report['command_tx'] = state['tx']
        report['driver_uart_evidence'] = [e for e in state['events'] if e['predicate'] == ord('W')]
        try:
            if claimed:
                if native:
                    event(ManagerEvent.STOP_MOTOR)
                else:
                    zero = ControlStream()
                    zero.header.frame_id = 'autonomy'
                    zero.header.stamp = node.get_clock().now().to_msg()
                    zero.joint_vel = [0.]*6
                    zero_pub.publish(zero)
                event(ManagerEvent.MODE, chr(ManagerEvent.NONE))
                spin(.1)
        except (Exception, KeyboardInterrupt) as error:
            report['passed'] = False
            report['cleanup_error'] = str(error)
        try:
            report_file.write(json.dumps(report, indent=2)+'\n')
        finally:
            report_file.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(json.dumps({'report': str(args.output), 'passed': report['passed'],
                      'phase': report['phase'], 'executed': report['executed'],
                      'message': report['message']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
