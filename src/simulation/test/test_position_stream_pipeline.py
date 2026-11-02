"""Isolated firmware-owner/actuator execution replay, no ROS entities."""
from pathlib import Path
import numpy as np
import pytest
from ament_index_python.packages import get_package_prefix
from control_interface_py.streamed_position import PositionEnvelope
from control_interface_py.streamed_position_native import Config, NativePositionSession
from catheter_control.safety.hardware_contract import load_hardware_contract
from catheter_control.safety.position_execution import make_position_execution
from simulation.sim_plant import SimulatedActuatorPlant
from simulation.sim_device import SimulatedDeviceNode
from types import SimpleNamespace


@pytest.fixture
def pipeline():
    root = Path(__file__).resolve().parents[2]
    contract = load_hardware_contract(root/'control_interface/config/catheter_limits.yaml', 'imricor_test')
    config = Config()
    config.native_position = 1
    for axis in range(6):
        config.lower[axis], config.upper[axis] = -2147483648, 2147483647
        config.step[axis], config.tolerance[axis] = 8334, 67
        config.minimum_rpm[axis], config.maximum_rpm[axis] = 1, 250
    library = Path(get_package_prefix('control_interface'))/'lib/libcatheter_position_stream.so'
    assert library.is_file(), 'build the M2 control_interface overlay before this gate'
    native = NativePositionSession(library, config)
    token = native.open(True, 123456)
    assert token == 123456
    plant = SimulatedActuatorPlant(contract, timestamp_ns=1000000000,
        initial_encoder_counts=[30000, 0, -10000, 10000, 0, 0], step_s=.01)
    yield contract, native, token, plant
    native.close()


def test_plan_endpoint_through_native_firmware_and_encoder_plant(pipeline):
    contract, native, token, plant = pipeline
    anchor = plant.encoder_counts.copy()
    execution = make_position_execution(contract, 1, anchor, anchor,
        [5., 0, 2., 0, 0, 0], .2, plant.timestamp_ns)
    assert native.submit(PositionEnvelope(token, 200000, execution.command),
        plant.timestamp_ns//1000, [int(x) for x in anchor]) == 0
    applied = [0]*6
    device = SimpleNamespace(plant=plant, position_stream=native,
                             _stream_native_targets=anchor.copy(), _stream_applied_rpm=applied)
    for _ in range(22):
        update = native.update(plant.timestamp_ns//1000,
            [int(x) for x in plant.encoder_counts], applied)
        SimulatedDeviceNode._apply_native_stream_update(device, update)
        applied = device._stream_applied_rpm
        plant.step()
    residual = np.asarray(execution.command.target_counts)-plant.encoder_counts
    assert np.all(np.abs(residual) <= 67)
    assert applied == [0]*6
    stopped_counts = plant.encoder_counts.copy()
    for _ in range(5):
        plant.step()
    assert np.array_equal(plant.encoder_counts, stopped_counts)


def test_cancel_then_legacy_velocity_cannot_replay_completed_goal(pipeline):
    contract, native, token, plant = pipeline
    anchor = plant.encoder_counts.copy()
    execution = make_position_execution(contract, 1, anchor, anchor,
        [5., 0, 0, 0, 0, 0], .2, plant.timestamp_ns)
    packet = PositionEnvelope(token, 200000, execution.command)
    native.submit(packet, plant.timestamp_ns//1000, [int(x) for x in anchor])
    native.cancel()
    plant.set_command([2., 0, 0, 0, 0, 0])
    plant.step()
    assert native.submit(packet, plant.timestamp_ns//1000, [int(x) for x in plant.encoder_counts]) == 1
    update = native.update(plant.timestamp_ns//1000, [int(x) for x in plant.encoder_counts], [36, 0, 0, 0, 0, 0])
    assert update.status == 4 and list(update.rpm) == [0]*6
