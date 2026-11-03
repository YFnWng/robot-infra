"""Observer contracts: non-actuating by construction, ordered inbox, measured conversion. No ROS graph."""
import ast
from pathlib import Path

import numpy as np
import pytest

from runtime_supervision import reduced_filter_replay as replay
from runtime_supervision.reduced_filter_shadow import ShadowInbox, encoder_sample, marker_quality_summary
from catheter_control.safety.hardware_contract import ENCODER_RADIANS_PER_COUNT
from control_interface_py.encoder_coordinates import counts_to_logical

PACKAGE = Path(__file__).resolve().parents[1]/'runtime_supervision'


def calls(path, name):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return [node for node in ast.walk(tree) if isinstance(node, ast.Call)
            and getattr(node.func, 'attr', getattr(node.func, 'id', None)) == name]


def imported(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return {alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
            and node.module == 'control_interface.msg' for alias in node.names}


def test_observer_publishes_only_private_string_status_and_has_no_clients():
    path = PACKAGE/'reduced_filter_shadow.py'
    publishers = calls(path, 'create_publisher')
    assert [(p.args[0].id, p.args[1].value) for p in publishers] == [('String', 'status')]
    for name in ('create_client', 'ActionClient', 'create_service'):
        assert not calls(path, name)
    assert imported(path) == {'DeviceStream'}


def test_replay_publishes_only_prefixed_state_and_has_no_clients():
    path = PACKAGE/'reduced_filter_replay.py'
    assert {p.args[0].id for p in calls(path, 'create_publisher')} == {'DeviceStream', 'PointCloud'}
    for name in ('create_client', 'ActionClient', 'create_service'):
        assert not calls(path, name)
    assert imported(path) == {'DeviceStream'}


@pytest.mark.parametrize('name', ['reduced_filter_shadow.py', 'reduced_filter_replay.py'])
def test_no_zero_or_command_vocabulary(name):
    source = (PACKAGE/name).read_text(encoding='utf-8')
    for forbidden in ('SET_ZERO', 'ControlStream', '/teleop', '/manager', 'command_output_enabled'):
        assert forbidden not in source


@pytest.mark.parametrize('domain,prefix', [(None, replay.REPLAY_PREFIX), ('0', replay.REPLAY_PREFIX),
    ('42', replay.REPLAY_PREFIX), ('214', ''), ('214', '/other')])
def test_replay_refuses_without_isolation(domain, prefix):
    with pytest.raises(RuntimeError):
        replay.require_isolation(domain, prefix)


def test_replay_accepts_isolated_domain_and_prefix():
    replay.require_isolation('214', replay.REPLAY_PREFIX)


def test_inbox_keeps_order_and_latches_overflow():
    inbox = ShadowInbox(maximum_events=2)
    assert inbox.put('a') and inbox.put('b') and not inbox.put('c')
    assert inbox.drain() == (['a', 'b'], True)
    assert inbox.drain() == ([], True)


def test_encoder_sample_uses_existing_conversion_owners():
    counts = np.array([26850., 0., -68642., 0., 0., 0.])
    shaft, tendon = encoder_sample(counts)
    np.testing.assert_array_equal(shaft, counts[:3]*ENCODER_RADIANS_PER_COUNT)
    assert tendon == counts_to_logical(counts[None])[0, 2]


@pytest.mark.parametrize('counts', [np.zeros(5), np.full(6, np.nan)])
def test_encoder_sample_rejects_malformed(counts):
    with pytest.raises(ValueError):
        encoder_sample(counts)


def test_replay_events_are_arrival_ordered_and_keep_the_selected_images():
    sensors = dict(marker_ns=np.array([10, 20, 30], np.int64), marker_receipt_ns=np.array([15, 25, 35], np.int64),
        encoder_ns=np.array([5, 12, 22, 32, 42], np.int64), encoder_receipt_ns=np.array([6, 13, 23, 33, 43], np.int64))
    events = replay.replay_events(sensors, np.array([10, 30], np.int64), start=0, stop=1, encoder_lead_ns=5)
    assert [e[0] for e in events] == sorted(e[0] for e in events)
    assert [e[2] for e in events if e[1]] == [0, 2] and 5 not in [e[2] for e in events if not e[1]]


def test_marker_quality_is_summarized_for_the_report_only():
    quality = dict(confidence=np.array([.9, .8, .7, .6]), reprojection_error_px=np.array([.1, .2, .3, .4]),
                   source_rig_count=np.array([2., 2., 2., 1.]))
    assert marker_quality_summary(quality) == dict(confidence=[.9, .8, .7, .6],
        reprojection_error_px=[.1, .2, .3, .4], source_rig_count=[2., 2., 2., 1.])
    tree = ast.parse((PACKAGE/'reduced_filter_shadow.py').read_text(encoding='utf-8'))
    added = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and getattr(node.func, 'attr', None) == 'add_image']
    assert [ast.unparse(node.args[1]) for node in added] == ['points']  # quality never reaches the filter


def test_observer_selects_every_third_30fps_frame_by_default():
    import inspect
    from runtime_supervision import reduced_filter_shadow
    assert "default=90." in inspect.getsource(reduced_filter_shadow.main)
