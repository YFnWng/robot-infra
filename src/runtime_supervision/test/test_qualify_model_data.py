import numpy as np
from pathlib import Path

from runtime_supervision.compute_profile import ReplayEvent
from runtime_supervision.qualify_model_data import extract


def test_extraction_uses_measured_encoder_calibration_not_commands():
    events = []
    for i in range(4):
        t = 1_000_000_000+i*20_000_000
        events += [ReplayEvent(t+1, t, 'encoder', np.array([100+i,0,-100-i,0,0,0])),
            ReplayEvent(t+2, t, 'marker', np.c_[np.zeros(4),np.zeros(4),np.arange(4)*.02],
                dict(confidence=np.ones(4), source_rig_count=np.ones(4), reprojection_error_px=np.zeros(4)))]
    events += [ReplayEvent(2_000_000_000, 2_000_000_000, 'plan', [1000]*6)]
    arrays, issues = extract(events)
    assert not issues
    np.testing.assert_allclose(arrays['shaft_rad'][0], np.array([100,0,-100])*np.pi/4000)
    assert arrays['logical_position'][0, 0] > arrays['logical_position'][0, 2]
    assert len(arrays['encoder_ns']) == 4


def test_discontinuity_and_nonadvancing_samples_are_recorded_not_rezeroed():
    events = []
    for t, count in [(1_000_000_000,100), (1_020_000_000,101),
                     (1_020_000_000,101), (1_040_000_000,10000)]:
        events += [ReplayEvent(t+1,t,'encoder',np.array([count,0,0,0,0,0])),
            ReplayEvent(t+2,t,'marker',np.c_[np.zeros(4),np.zeros(4),np.arange(4)*.02],
                dict(confidence=np.ones(4),source_rig_count=np.ones(4),reprojection_error_px=np.zeros(4)))]
    arrays, issues = extract(events)
    assert arrays['encoder_counts'][-1,0] == 10000
    assert len(arrays['encoder_ns']) == 3
    assert any(r['reason'] == 'encoder_rate_discontinuity' for r in issues)
    assert len(arrays['interruptions_ns']) == 2


def test_canonical_reader_sensor_mode_preserves_invalid_evidence(tmp_path):
    import pytest
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud, ChannelFloat32
    from geometry_msgs.msg import Point32
    from builtin_interfaces.msg import Time
    from control_interface.msg import DeviceStream
    from runtime_supervision.compute_profile import read_events

    session = tmp_path/'fixture'; session.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(session/'robot_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr','cdr'))
    for name, kind in [('/device/state','control_interface/msg/DeviceStream'),
                       ('/shape_tracking/markers','sensor_msgs/msg/PointCloud')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
    bad = PointCloud(); bad.header.frame_id = 'robot_base'
    writer.write('/shape_tracking/markers', serialize_message(bad), 1_000_000_001)
    bad.header.stamp = Time(sec=1, nanosec=10_000_000)
    writer.write('/shape_tracking/markers', serialize_message(bad), 1_010_000_001)
    good = PointCloud(); good.header.frame_id = 'robot_base'
    good.header.stamp = Time(sec=1, nanosec=20_000_000)
    good.points = [Point32(x=0.,y=0.,z=.02*i) for i in range(4)]
    good.channels = [ChannelFloat32(name=name, values=values) for name,values in
        [('marker_id',[0.,1.,2.,3.]), ('confidence',[1.]*4),
         ('source_rig_count',[1.]*4), ('reprojection_error_px',[0.]*4)]]
    writer.write('/shape_tracking/markers', serialize_message(good), 1_020_000_001)
    encoder = DeviceStream(); encoder.header.stamp = Time(sec=1)
    encoder.predicate = ord('E'); encoder.data = [100.]*6
    writer.write('/device/state', serialize_message(encoder), 1_000_000_001)
    del writer
    with pytest.raises(ValueError, match='invalid source timestamp'):
        read_events(session, require_plan=False, sensors_only=True)
    invalid = []
    events = read_events(session, require_plan=False, sensors_only=True, invalid_samples=invalid)
    assert {e.kind for e in events} == {'encoder','marker'}
    assert {e['reason'] for e in invalid} == {'invalid_source_timestamp','marker_count'}


def test_cli_records_configuration_and_exact_sources(tmp_path, monkeypatch):
    import hashlib
    import json
    from runtime_supervision import qualify_model_data as owner
    monkeypatch.setattr(owner, 'run_session', lambda *_args, **_kwargs:
                        dict(audit=dict(supported_windows=4,test_windows=2)))
    output = tmp_path/'report'
    assert owner.main(['--session',str(tmp_path/'session'), '--output',str(output),
                       '--stride-s','.3']) == 0
    result = json.loads((output/'report.json').read_text())
    assert result['window_contract']['stride_s'] == .3
    for path, digest in result['source_identity'].items():
        saved = output/'source_snapshot'/(digest[:12]+'_'+Path(path).name)
        assert hashlib.sha256(saved.read_bytes()).hexdigest() == digest


def test_model_failure_is_not_disguised_as_bad_recording(tmp_path, monkeypatch):
    import pytest
    from runtime_supervision import qualify_model_data as owner
    def fail(*_args, **_kwargs):
        raise ValueError('nonfinite prediction')
    monkeypatch.setattr(owner, 'run_session', fail)
    with pytest.raises(ValueError, match='nonfinite prediction'):
        owner.main(['--session',str(tmp_path/'session'), '--output',str(tmp_path/'report')])
