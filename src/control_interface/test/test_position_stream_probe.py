import pytest
import json
import numpy as np
from control_interface_py.position_stream_probe import (
    validate_probe, encoder_counts, gate_diagnostics, probe_offsets, score_probe)


def test_probe_bounds():
    validate_probe(100, 36, 250000)
    validate_probe(-100, 36, 20000)
    for values in ((0, 36, 250000), (201, 36, 250000), (100, 12, 250000),
                   (100, 1, 250000), (100, 3, 250000), (100, 35, 250000),
                   (100, 37, 250000), (100, 36, 250001), (67, 36, 250000),
                   (200, 36, 9999)):
        with pytest.raises(ValueError):
            validate_probe(*values)


def test_exact_encoder_counts():
    assert encoder_counts([1., 2., 3., 4., 5., 6.]) == (1, 2, 3, 4, 5, 6)
    for data in ([1]*5, [float('nan')]*6, [.5]*6, [2147483648]*6):
        with pytest.raises(ValueError):
            encoder_counts(data)


def test_ros_fixed_array_encoder_report_is_json_serializable():
    # Generated ROS fixed int32 fields expose NumPy scalars, not Python ints.
    ros_targets = np.asarray([200, 0, 0, 0, 0, 0], dtype=np.int32)
    report = {'final_target_counts': list(encoder_counts(ros_targets))}
    assert json.loads(json.dumps(report))['final_target_counts'] == [200, 0, 0, 0, 0, 0]


def test_gate_missing_reports_independent_causes():
    state = dict(enc=None, enc_time=None, safety='', safety_time=None)
    gates = gate_diagnostics(state, 1., 1000000000)
    assert not gates['ready']
    assert 'ENC_MISSING_OR_REJECTED' in gates['reasons']
    assert 'SAFETY_RECEIPT_STALE' in gates['reasons']
    assert gates['encoder_receipt_age_ms'] is None


def test_gate_ready_and_stale_or_future_source():
    state = dict(enc=(0,)*6, enc_time=1., enc_stamp_ns=1000000000,
                 safety='MANAGER_READY', safety_time=1.)
    assert gate_diagnostics(state, 1.01, 1010000000)['ready']
    # A recently delivered but old source frame must not pass.
    assert 'ENC_SOURCE_STALE_OR_FUTURE' in gate_diagnostics(state, 1.01, 1200000000)['reasons']
    assert 'ENC_SOURCE_STALE_OR_FUTURE' in gate_diagnostics(state, 1.01, 999000000)['reasons']
    state['safety_time'] = .7
    assert 'SAFETY_RECEIPT_STALE' in gate_diagnostics(state, 1.01, 1010000000)['reasons']


def test_compound_offsets_are_bounded_and_reverse():
    assert probe_offsets('replacement', 200) == (200, 180)
    assert probe_offsets('replacement', -200) == (-200, -180)
    assert probe_offsets('reversal', -200) == (-200, 100)
    assert probe_offsets('reversal', 200) == (200, -100)
    with pytest.raises(ValueError):
        probe_offsets('replacement', 100)


def telemetry(text, command=1, rpm=0):
    return {'text': 'POSITION_STREAM:'+text,
            'data': [42, command, 0, 0, 0, 0]+[0]*6+[rpm]+[0]*5}


def test_expiry_requires_expired_incomplete_motion_and_zero_rpm():
    origin, target, final = [0]*6, [200]+[0]*5, [30]+[0]*5
    assert score_probe('expiry', origin, target, final, final, [telemetry('EXPIRED')], 42, 1)
    assert not score_probe('expiry', origin, target, final, final, [telemetry('COMPLETE')], 42, 1)
    assert not score_probe('expiry', origin, target, final, final, [telemetry('EXPIRED', rpm=1)], 42, 1)
    assert not score_probe('expiry', origin, target, origin, origin, [telemetry('EXPIRED')], 42, 1)
    backwards = [-30]+[0]*5
    assert not score_probe('expiry', origin, target, backwards, backwards, [telemetry('EXPIRED')], 42, 1)


def test_replacement_requires_superseded_and_second_complete():
    origin, target, final = [0]*6, [200]+[0]*5, [220]+[0]*5
    events = [telemetry('SUPERSEDED'), telemetry('COMPLETE', command=2)]
    assert score_probe('replacement', origin, target, final, final, events, 42, 2)
    assert not score_probe('replacement', origin, target, final, final, events[1:], 42, 2)
    assert not score_probe('replacement', origin, target, final, final,
                           events+[telemetry('NO_PROGRESS', command=2)], 42, 2)
    drift = [230]+[0]*5
    assert not score_probe('replacement', origin, target, final, drift, events, 42, 2)
