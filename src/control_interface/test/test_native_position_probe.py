import json
import sys
import types
from types import SimpleNamespace

import pytest

from control_interface_py import native_position_probe as probe
from control_interface_py.encoder_coordinates import UNITS_PER_COUNT


def test_live_semantics_classification_and_ambiguous_reference():
    for final, label in ((100, 'replace_original_reference'), (180, 'replace_current_reference'),
                         (400, 'ignored_matched_control'), (500, 'queued_increment')):
        assert probe.classify_bracketed_update(0, 80, 80, final, 400)[0] == label
    assert probe.classify_bracketed_update(0, 300, 310, 400, 400)[0] == 'ambiguous_or_other'


def test_live_cli_rejects_unbounded_or_legacy_scenarios(tmp_path):
    from control_interface_py.position_stream_probe import main
    for options in (['--backend', 'driver_position'], ['--delta-counts', '500'],
                    ['--duration-us', '100000'], ['--rpm', '1']):
        output = tmp_path/'not_created.json'
        with pytest.raises(SystemExit):
            main(['--mode', 'live-update', '--output', str(output)]+options)
        assert not output.exists()


@pytest.mark.parametrize('ack', [True, False])
@pytest.mark.parametrize('control', [True, False])
def test_live_probe_records_semantics_only_with_unique_ack_and_tx(ack, control, monkeypatch):
    from control_interface.msg import StreamedPositionControl
    from control_interface_py.streamed_position import LIVE_UPDATE
    now = [0.]
    monkeypatch.setattr(probe.time, 'monotonic', lambda: now[0])
    pending = []
    publications = []
    state = {'enc': (0,)*6, 'events': [], 'tx': []}
    report = {}
    goal = StreamedPositionControl()
    goal.header.frame_id = 'autonomy'
    goal.session_id, goal.command_id, goal.deadline_ros_ns = 42, 1, 250000000
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(now[0]*1e9), to_msg=lambda: goal.header.stamp)))

    def event(text, status, flags=0):
        state['events'].append({'text': text, 'data': [42, 1, 0, 0, status, flags]
                                +list(state['enc'])+[0]*6})

    def publish_goal(msg):
        publications.append(('Y', now[0]))
        assert not pending, 'newer Y must not overtake pending X'
        if not state['events']:
            state['enc'] = (80, 0, 0, 0, 0, 0)
            event('POSITION_STREAM:NATIVE_STARTED', 12)

    def publish_live(msg):
        assert msg.predicate == LIVE_UPDATE and list(msg.data) == [42., 1., 0. if control else 45.]
        publications.append(('X', now[0]))
        pending.append((now[0]+.02, list(msg.data)))

    def deliver(data):
        state['tx'].append({'predicate': LIVE_UPDATE, 'data': data})
        if ack:
            event('POSITION_STREAM:LIVE_BEFORE_WRITE', 14, 45)
            state['enc'] = (80 if control else 100, 0, 0, 0, 0, 0)
            event('POSITION_STREAM:LIVE_AFTER_ACK', 15, 45)
            event('POSITION_STREAM:LIVE_UPDATED', 13, 45)
        state['enc'] = (400 if control else 200, 0, 0, 0, 0, 0)

    def spin(seconds):
        now[0] += seconds
        if pending and now[0] >= pending[0][0]:
            _, data = pending.pop(0)
            deliver(data)
        if now[0] >= .25 and not any(e['text'].endswith('EXPIRED') for e in state['events']):
            event('POSITION_STREAM:EXPIRED', 3)

    args = (node, state, report, (0,)*6, goal, lambda: None, spin,
            SimpleNamespace(publish=publish_goal), SimpleNamespace(publish=publish_live))
    if ack:
        probe.run_live_update_probe(*args, control=control,
                                   control_report={'anchor_counts': [0]*6, 'final_counts': [400]+[0]*5})
        assert report['passed'] and report['live_semantics'] == (
            'matched_no_update_control' if control else 'replace_current_reference')
        assert goal.deadline_ros_ns == 250000000
        assert report['live_update_ordering_wait_ms'] >= 20
        assert sum(kind == 'X' for kind, _ in publications) == 1
    else:
        with pytest.raises(RuntimeError, match='dispatch/completion timeout'):
            probe.run_live_update_probe(*args, control=control)
        assert sum(kind == 'X' for kind, _ in publications) == 1
        x_time = next(t for kind, t in publications if kind == 'X')
        assert not any(kind == 'Y' and t >= x_time for kind, t in publications)
        assert goal.deadline_ros_ns == 250000000


def test_bracketed_classifier_needs_control_and_keeps_wide_interval_ambiguous():
    assert probe.classify_bracketed_update(0, 80, 100, 200, 465)[0] == 'replace_current_reference'
    assert probe.classify_bracketed_update(0, 80, 100, 465, 465)[0] == 'ignored_matched_control'
    assert probe.classify_bracketed_update(0, 80, 100, 200)[0] == 'ambiguous_or_other'
    assert probe.classify_bracketed_update(0, 200, 400, 465, 465)[0] == 'ambiguous_or_other'


def test_native_offsets_and_quantization():
    assert probe.native_offsets('replacement') == (400, 600)
    assert probe.native_offsets('reversal') == (400, -200)
    with pytest.raises(ValueError):
        probe.native_offsets('expiry')
    target, logical = probe.native_target((0,)*6, 400)
    assert target == [400, 0, 0, 0, 0, 0]
    assert logical[0] == pytest.approx(400*UNITS_PER_COUNT[0])
    assert probe.relative_angle_counts(0, 400) == (18, 400.)
    assert probe.relative_angle_counts(100, 600)[0] == 23
    json.dumps(dict(target=target, logical=logical))


def test_cancel_score_does_not_accept_completed_endpoint_or_drift():
    anchor, target, stopped = (0,)*6, [400, 0, 0, 0, 0, 0], [100, 0, 0, 0, 0, 0]
    assert probe.native_score('cancel', anchor, target, stopped, stopped, True)
    assert not probe.native_score('cancel', anchor, target, target, target, True)
    assert not probe.native_score('cancel', anchor, target, stopped, [110, 0, 0, 0, 0, 0], True)
    assert not probe.native_score('replacement', anchor, target, target, target, False)


def test_native_cli_rejects_stream_lease_without_opening_report(tmp_path):
    from control_interface_py.position_stream_probe import main
    report = tmp_path/'must_not_exist.json'
    for options in (['--mode', 'expiry'], ['--duration-us', '20000'], ['--rpm', '12']):
        with pytest.raises(SystemExit):
            main(['--backend', 'driver_position', '--output', str(report)]+options)
        assert not report.exists()


@pytest.mark.parametrize('mode', ['endpoint', 'cancel', 'replacement', 'reversal'])
def test_native_transactions_stop_before_replacement(monkeypatch, mode):
    # Replay composition with deterministic fake motion; no ROS graph or serial.
    class Message:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id='', stamp=None)

    Event = types.SimpleNamespace(MODE=77, JOINT_POS=52, STOP_MOTOR=83)
    monkeypatch.setitem(sys.modules, 'control_interface.msg',
                        types.SimpleNamespace(ControlStream=Message, ManagerEvent=Event))
    clock = [0.]
    monkeypatch.setattr(probe.time, 'monotonic', lambda: clock[0])
    state = dict(enc=(0,)*6, events=[], tx=[])
    report = dict(executed=False)
    motion = dict(target=None)
    actions = []

    def spin(seconds):
        clock[0] += seconds
        if motion['target'] is not None:
            current, target = state['enc'][0], motion['target']
            step = max(-100, min(100, target-current))
            state['enc'] = (current+step, 0, 0, 0, 0, 0)
            if state['enc'][0] == target:
                state['events'].append({'text': 'POSITION_COMPLETE'})
                motion['target'] = None

    def event(predicate, text=''):
        actions.append(('event', predicate, state['enc'][0]))
        if predicate == 83:
            motion['target'] = None

    class Publisher:
        def publish(self, msg):
            target = round(msg.joint_pos[0]/UNITS_PER_COUNT[0])
            actions.append(('POS', target, state['enc'][0]))
            motion['target'] = target
            state['tx'].append({'predicate': ord('P'), 'data': msg.joint_pos+msg.joint_vel})

    node = types.SimpleNamespace(get_clock=lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=int(clock[0]*1e9), to_msg=lambda: None)))
    probe.run_native_probe(node, types.SimpleNamespace(mode=mode, rpm=36), state,
                           report, (0,)*6, lambda: None, spin, event, Publisher())
    assert report['passed']
    if mode in ('replacement', 'reversal'):
        commands = report['native_commands']
        assert commands[1]['root_counts'] == commands[0]['stopped_counts']
        assert commands[1]['start_ros_ns'] >= commands[0]['stop_confirmed_ros_ns']
        expected = 600 if mode == 'replacement' else -200
        assert state['enc'][0] == expected
        first_stop = next(i for i, a in enumerate(actions) if a[:2] == ('event', 83))
        assert all(a[1] != 400 for a in actions[first_stop+1:] if a[0] == 'POS')
    assert all(a[0] in ('POS', 'event') for a in actions)
