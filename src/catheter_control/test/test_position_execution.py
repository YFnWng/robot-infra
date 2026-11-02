from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.position_execution import make_position_execution, root_encoder_counts, position_execution_in_flight
from catheter_control.node import CatheterControlNode


@pytest.fixture
def contract():
    path = Path(__file__).resolve().parents[2]/'control_interface/config/catheter_limits.yaml'
    return load_hardware_contract(path, 'imricor_test')


def test_endpoint_is_anchored_once(contract):
    anchor = np.array([30000, 0, -10000, 10000, 0, 0])
    execution = make_position_execution(contract, 1, anchor, anchor, [5., 0, 2., 0, 0, 0], 1/15, 1000000000)
    assert execution.deadline_ros_ns == 1066667000
    assert execution.command.active_mask == 5
    assert execution.command.target_counts[0] > anchor[0]
    assert execution.command.target_counts[2] < anchor[2]


def test_native_distance_is_not_raised_to_minimum_speed(contract):
    anchor = np.array([47537, 0, -10000, 0, 0, 0])
    velocity = [9., 0, .05, 0, 0, 0]
    execution = make_position_execution(contract, 1, anchor, anchor,
        velocity, 1/15, 1, integer_degrees=True, authority_s=.25,
        preserve_displacement=True)
    assert execution.command.active_mask == 1
    assert execution.command.target_counts[2] == anchor[2]
    expected_chassis = (9.-.05)/15/.00042074900625
    assert execution.command.target_counts[0]-anchor[0] == pytest.approx(expected_chassis, abs=12)


def test_native_coupling_holds_chassis_without_activating_it(contract):
    anchor = np.array([47537, 0, -10000, 0, 0, 0])
    execution = make_position_execution(contract, 1, anchor, anchor,
        [.5, 0, .5, 0, 0, 0], 1/15, 1, integer_degrees=True,
        authority_s=.25, preserve_displacement=True)
    assert execution.command.active_mask == 4
    assert execution.command.target_counts[0] == anchor[0]
    assert execution.command.speed_rpm[2] >= 101
    assert abs(execution.command.target_counts[2]-anchor[2]) < 250


def test_native_endpoint_preserves_operational_encoder_reserve(contract):
    anchor = np.array([108990, 0, -10000, 0, 0, 0])
    with pytest.raises(ValueError, match='operational encoder reserve'):
        make_position_execution(contract, 1, anchor, anchor,
            [2.,0,0,0,0,0], 1/15, 1, integer_degrees=True,
            authority_s=.25, preserve_displacement=True)


def test_repeat05_small_tendon_seed_cannot_start_native_tendon_acquisition(contract):
    velocity = np.array([19.83220692, 0, .09592443, 0, 0, 0])
    velocity /= max(1., np.max(np.abs(velocity)/contract.velocity_max))
    anchor = np.array([47537, 0, -1, 0, 0, 0])
    execution = make_position_execution(contract, 1, anchor, anchor,
        velocity, 1/15, 1, integer_degrees=True, authority_s=.25,
        preserve_displacement=True)
    assert execution.command.active_mask == 1
    assert execution.command.target_counts[2] == -1


def test_adapj_native_takeup_has_no_nonzero_velocity_fallback(contract):
    packets, velocities, events = [], [], []
    now = [1_000_000_000]
    velocity = np.array([2., 0, 2., 0, 0, 0])
    fake = SimpleNamespace(command_output_enabled=True, source='test',
        mppi_execution_mode='position_stream', takeup_transaction_enabled=True,
        takeup_arbiter=SimpleNamespace(state='TAKEUP_ACTIVE',
            note_command_publication=lambda *a: events.append(a)),
        adapj=SimpleNamespace(step_s=1/15), contract=contract,
        position_execution=None, position_stream_session=12,
        position_stream_command_id=0,
        position_stream_encoder_counts=np.array([47537, 0, -10000, 0, 0, 0]),
        _adapj_encoder_history=[], _fault_locked=lambda e: pytest.fail(e),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=now[0], to_msg=lambda: Time(sec=now[0]//10**9,
                                                   nanosec=now[0]%10**9))),
        position_control_pub=SimpleNamespace(publish=packets.append),
        control_pub=SimpleNamespace(publish=velocities.append),
        tip_forecast_monitor=SimpleNamespace(note_command=lambda *a: None))
    for _ in range(2):
        CatheterControlNode._publish_velocity_locked(fake, velocity)
    assert [p.command_id for p in packets] == [1, 1]
    assert packets[0].active_mask == 4 and not velocities
    now[0] = 1_250_000_000
    CatheterControlNode._publish_velocity_locked(fake, velocity)
    assert not any(velocities[-1].joint_vel)
    fake._adapj_encoder_history = [(t, fake.position_stream_encoder_counts)
        for t in range(1_250_000_000, 1_360_000_001, 10_000_000)]
    now[0] = 1_360_000_000
    CatheterControlNode._publish_velocity_locked(fake, velocity)
    assert packets[-1].command_id == 2
    # Confirming a shaft changes the requested set: cancel, never live retarget.
    now[0] += 10_000_000
    CatheterControlNode._publish_velocity_locked(fake, np.array([4., 0, 2., 0, 0, 0]))
    assert len(packets) == 3 and not any(velocities[-1].joint_vel)
    assert all(not any(m.joint_vel) for m in velocities)


def test_encoder_anchor_is_root_aligned_not_newest_received_sample():
    history = [(1000000000, [1]*6), (1020000000, [2]*6)]
    assert list(root_encoder_counts(history, 1000000100)) == [1]*6
    with pytest.raises(ValueError):
        root_encoder_counts(history, 1010000000)
    with pytest.raises(ValueError):
        root_encoder_counts([(1000000000, [1]*3)], 1000000000)


def test_already_passed_endpoint_cannot_generate_unsampled_reversal(contract):
    anchor = np.array([30000, 0, -10000, 10000, 0, 0])
    current = anchor.copy(); current[0] += 5000
    execution = make_position_execution(contract, 1, anchor, current, [5., 0, 0, 0, 0, 0], 1/15, 1)
    assert execution.command.active_mask == 0
    assert execution.command.target_counts[0] == current[0]


def test_stream_heartbeat_repeats_id_and_expiry_stops(contract):
    anchor = np.array([30000, 0, -10000, 10000, 0, 0])
    execution = make_position_execution(contract, 7, anchor, anchor, [5., 0, 0, 0, 0, 0], 1/15, 1000000000)
    packets, velocities = [], []
    now = [1001000000]
    fake = SimpleNamespace(command_output_enabled=True, source='catheter_mppi',
        mppi_execution_mode='position_stream', takeup_transaction_enabled=False,
        position_execution=execution, position_stream_session=12,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=now[0], to_msg=lambda: Time(sec=1))),
        position_control_pub=SimpleNamespace(publish=packets.append),
        control_pub=SimpleNamespace(publish=velocities.append),
        tip_forecast_monitor=SimpleNamespace(note_command=lambda *args: None))
    for _ in range(2):
        CatheterControlNode._publish_velocity_locked(fake, np.array(execution.velocity))
    assert [p.command_id for p in packets] == [7, 7]
    assert np.array_equal(packets[0].target_counts, packets[1].target_counts)
    now[0] = execution.deadline_ros_ns
    CatheterControlNode._publish_velocity_locked(fake, np.array(execution.velocity))
    assert list(velocities[-1].joint_vel) == [0.]*6
    assert fake.position_execution is None


@pytest.mark.parametrize('step', [0, -.1, .251, float('nan')])
def test_first_step_is_bounded(contract, step):
    with pytest.raises(ValueError):
        make_position_execution(contract, 1, [0]*6, [0]*6, [0]*6, step, 1)


def test_native_execution_window_never_extends_lease(contract):
    execution = make_position_execution(contract, 1, [0]*6, [0]*6, [2., 0, 0, 0, 0, 0], .2, 1000000000)
    assert position_execution_in_flight(execution, 1100000000)
    assert not position_execution_in_flight(execution, execution.deadline_ros_ns)
    assert not position_execution_in_flight(None, 1)


def test_adapj_integer_degree_projection_is_explicit(contract):
    anchor = np.array([30000, 0, -10000, 10000, 0, 0])
    execution = make_position_execution(contract, 1, anchor, anchor,
        [5., 0, 2., 0, 0, 0], .2, 1, integer_degrees=True)
    displacement = np.array(execution.command.target_counts)-anchor
    # Raw encoder units round to within half a count of integer motor degrees.
    np.testing.assert_allclose(displacement*.045,
        np.rint(displacement*.045), atol=.0225)


def test_execution_authority_does_not_increase_sampled_displacement(contract):
    args = (contract, 1, [0]*6, [0]*6, [2., 0, 0, 0, 0, 0], 1/15, 1_000_000_000)
    short = make_position_execution(*args)
    native = make_position_execution(*args, authority_s=.25)
    assert native.command.target_counts == short.command.target_counts
    assert native.command.duration_us == 250000
    assert native.deadline_ros_ns == 1_250_000_000
    with pytest.raises(ValueError, match='authority'):
        make_position_execution(*args, authority_s=.3)
    with pytest.raises(ValueError, match='authority'):
        make_position_execution(*args, authority_s=.01)
