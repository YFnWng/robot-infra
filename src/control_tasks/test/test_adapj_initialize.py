import numpy as np
import json
from types import SimpleNamespace
import pytest
import yaml
from pathlib import Path

from control_tasks import adapj_initialize as module
from control_tasks.adapj_initialize import initialization_positions, main
from catheter_control.planning.adapj import initialization_shaft_step, initialization_response_confirmed
from catheter_control.safety.hardware_contract import HardwareContract
from control_interface.msg import ManagerEvent
from control_tasks.sparse_point_experiment import load_sparse_point_experiment


def test_initialization_sequence_is_bounded_returns_home_and_locks_other_axes():
    home = np.array([20., 0., 0., 0., 0., 0.])
    sequence = np.array(initialization_positions(home))
    np.testing.assert_array_equal(sequence[-1], home)
    assert np.all(sequence[:, [1, 3, 4, 5]] == 0)
    assert np.all((sequence[:, 0] >= 20) & (sequence[:, 0] <= 24))
    assert np.all((sequence[:, 2] >= 0) & (sequence[:, 2] <= 4))


def test_initialization_requires_explicit_execute_before_ros_start():
    with pytest.raises(SystemExit):
        main(["unused.yaml"])


def test_initialization_accepts_ros_service_remaps_before_execute_guard(capsys):
    with pytest.raises(SystemExit):
        main(["unused.yaml", "--ros-args", "-r",
              "/catheter_mppi/adapj_initialization:=/sim/catheter_mppi/adapj_initialization"])
    assert "--execute is required" in capsys.readouterr().err


def test_simulation_task_uses_only_sim_endpoints_and_preserves_history():
    config = Path(__file__).resolve().parents[2]/"catheter_control/config/experiments/sparse_points"
    spec = load_sparse_point_experiment(config/"adapj_plain_twoaxis_sim.yaml")
    for field in ("action_name", "marker_topic", "position_topic", "status_topic",
                  "safety_topic", "control_topic", "event_topic",
                  "planned_control_topic", "manager_control_topic"):
        assert getattr(spec, field).startswith("/sim/")
    assert spec.plant_reset_mode == "history_preserving"


def test_point_profile_keeps_original_four_targets():
    config = Path(__file__).resolve().parents[2]/"catheter_control/config/experiments/sparse_points"
    spec = load_sparse_point_experiment(config/"adapj_plain_twoaxis.yaml")
    baseline = load_sparse_point_experiment(config/"farther_tendon_12_plain.yaml")
    assert spec.required_controller_status["control_method"] == "adapj"
    np.testing.assert_array_equal(spec.generator["targets_m"], baseline.generator["targets_m"])


@pytest.mark.parametrize("suffix", ["", "_sim"])
def test_compensated_task_changes_only_compensation_requirements(suffix):
    root = Path(__file__).resolve().parents[2]/"catheter_control/config/experiments/sparse_points"
    plain = yaml.safe_load((root/f"adapj_plain_twoaxis{suffix}.yaml").read_text())
    compensated = yaml.safe_load((root/f"adapj_takeup_twoaxis{suffix}.yaml").read_text())
    for name in ("backlash_compensation_enabled", "takeup_transaction_enabled"):
        plain["required_controller_status"][name] = True
    assert plain == compensated


def bounded_contract():
    return HardwareContract(np.zeros(6), np.full(6, 100.),
                            np.zeros(6), np.full(6, 10.))


def test_compensated_steps_isolate_physical_shafts_and_keep_rotation_locked():
    position = np.array([20., 0., 5., 0., 0., 0.])
    for axis in (0, 2):
        target = initialization_shaft_step(position, axis, .5, bounded_contract())
        delta = (HardwareContract.logical_position_to_motor_axis_position(target)
                 - HardwareContract.logical_position_to_motor_axis_position(position))
        np.testing.assert_allclose(delta, np.eye(6)[axis]*.5)
        np.testing.assert_array_equal(target[[1, 3, 4, 5]], position[[1, 3, 4, 5]])
    with pytest.raises(RuntimeError, match="joint limit"):
        initialization_shaft_step(np.zeros(6), 2, -.5, bounded_contract())


@pytest.mark.parametrize("phase,direction,stamp,expected", [
    ("ENGAGED", -1, 1_100_000_000, True),
    ("PROVISIONAL", -1, 1_100_000_000, False),
    ("ENGAGED", 1, 1_100_000_000, False),
    ("ENGAGED", -1, 900_000_000, False),
    ("ENGAGED", -1, 1_300_000_000, False)])
def test_confirmation_requires_fresh_correct_sign_source_evidence(phase, direction, stamp, expected):
    values = dict(backlash_phase=json.dumps(["UNKNOWN", "UNKNOWN", phase]),
                  backlash_engaged_direction=json.dumps([0, 0, direction]),
                  backlash_last_evidence_timestamp_ns=json.dumps([0, 0, stamp]))
    assert initialization_response_confirmed(values, 2, 1, 1_000_000_000, 1_200_000_000) is expected
    if phase == "PROVISIONAL":
        assert initialization_response_confirmed(values, 2, 1, 1_000_000_000, 1_200_000_000, provisional=True)
    assert not initialization_response_confirmed({}, 2, 1, 0, 1)


def fake_calibration_node(monkeypatch, *, response_after_mm=5.):
    root = Path(__file__).resolve().parents[2]/"catheter_control/config/experiments/sparse_points"
    spec = load_sparse_point_experiment(root/"adapj_takeup_twoaxis.yaml")
    now = [0.]
    def clock():
        now[0] += .005
        return now[0]
    monkeypatch.setattr(module.time, "monotonic", clock)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    stages, modes, captures = [], [], []
    phases, signs, stamps = ["UNKNOWN"]*3, [0]*3, [0]*3
    node = SimpleNamespace(spec=spec, position=spec.home_position.copy(),
        manager_ready=True, controller_armed=False, controller_state="DISARMED",
        controller_values={}, _controller_requirements_met=lambda: True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000+round(now[0]*1e9))),
        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
        _publish_mode=modes.append, _publish_home=lambda *a: None)
    def fresh():
        for field in ("position_time", "tip_time", "controller_time", "safety_time"):
            setattr(node, field, now[0])
        node.controller_values = dict(backlash_phase=json.dumps(phases),
            backlash_engaged_direction=json.dumps(signs),
            backlash_last_evidence_timestamp_ns=json.dumps(stamps))
    def spin(*a, **k):
        now[0] += .02
        fresh()
    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    previous_leg = [None]
    travel = [0.]
    active_axis = [None]
    def stage(target, axes, deadline, label, *, stop_when):
        motor_delta = (HardwareContract.logical_position_to_motor_axis_position(target)
                       - HardwareContract.logical_position_to_motor_axis_position(node.position))
        axis = int(np.argmax(np.abs(motor_delta)))
        direction = int(np.sign(motor_delta[axis]))
        if "acquisition" in label:
            if label != previous_leg[0]:
                travel[0] = 0.
                previous_leg[0] = label
            travel[0] += abs(motor_delta[axis])
            active_axis[0] = axis
            phases[axis] = "TAKEUP"
            node.position = target.copy()
            if travel[0] >= response_after_mm:
                phases[axis] = "PROVISIONAL"
                signs[axis] = direction if axis == 0 else -direction
                stamps[axis] = node.get_clock().now().nanoseconds
            fresh()
            stages.append((label, node.position.copy(), travel[0]))
            return stop_when()
        assert phases[axis] == "ENGAGED"
        assert not stop_when()
        node.position = target.copy()
        fresh()
        stages.append((label, node.position.copy(), None))
    node._run_home_stage = stage
    def stopped(*a):
        axis = active_axis[0]
        phases[axis] = "ENGAGED"
        stamps[axis] = node.get_clock().now().nanoseconds
        fresh()
    node._stop_before_home_retry = stopped
    def capture(*a):
        captures.append("fit")
        return "initialization.json"
    monkeypatch.setattr(module, "capture", capture)
    node.home = lambda index: captures.append("final_home")
    fresh()
    return node, spec, stages, modes, captures


def test_compensated_initialization_acquires_beyond_old_four_mm_then_calibrates(monkeypatch):
    node, spec, stages, modes, captures = fake_calibration_node(monkeypatch)
    result = module.run_compensated_initialization(node, spec, bounded_contract())
    assert result == "initialization.json"
    assert sum("useful calibration" in label for label, _, _ in stages) == 4
    assert all(travel <= 10 for _, _, travel in stages if travel is not None)
    assert captures == ["fit", "final_home"]
    assert node.spec is spec and modes[-1] == ManagerEvent.NONE


def test_compensated_initialization_retains_confirmation_through_slow_stop(monkeypatch):
    node, spec, stages, modes, captures = fake_calibration_node(monkeypatch)
    stopped = node._stop_before_home_retry
    def delayed_stop(*args):
        stopped(*args)
        # Fresh status/feedback continues, but the engagement event stays fixed.
        for _ in range(20):
            module.rclpy.spin_once(node, timeout_sec=.02)
    node._stop_before_home_retry = delayed_stop
    assert module.run_compensated_initialization(node, spec, bounded_contract()) == 'initialization.json'
    assert sum('useful calibration' in label for label, _, _ in stages) == 4
    assert captures == ['fit', 'final_home']


@pytest.mark.parametrize('phase,sign,stamp,anchor,expected', [
    ('ENGAGED', -1, 1_100_000_000, 1_050_000_000, True),
    ('PROVISIONAL', -1, 1_100_000_000, 1_050_000_000, False),
    ('UNKNOWN', -1, 1_100_000_000, 1_050_000_000, False),
    ('FAILED', -1, 1_100_000_000, 1_050_000_000, False),
    ('ENGAGED', 1, 1_100_000_000, 1_050_000_000, False),
    ('ENGAGED', -1, 1_100_000_000, 1_150_000_000, False),
    ('ENGAGED', -1, 900_000_000, 1_050_000_000, False),
    ('ENGAGED', -1, 2_100_000_000, 1_050_000_000, False),
    ('ENGAGED', -1, 1_100_000_000, 900_000_000, False),
])
def test_retained_confirmation_is_bound_to_current_leg(phase, sign, stamp, anchor, expected):
    values = dict(backlash_phase=json.dumps(['UNKNOWN', 'UNKNOWN', phase]),
        backlash_engaged_direction=json.dumps([0, 0, sign]),
        backlash_last_evidence_timestamp_ns=json.dumps([0, 0, stamp]))
    assert not initialization_response_confirmed(values, 2, 1, 1_000_000_000, 2_000_000_000)
    assert initialization_response_confirmed(values, 2, 1, 1_000_000_000, 2_000_000_000,
        acquisition_evidence_ns=anchor) is expected


def test_compensated_initialization_unconfirmed_response_exhausts_budget_without_fit(monkeypatch):
    node, spec, stages, modes, captures = fake_calibration_node(monkeypatch, response_after_mm=100.)
    with pytest.raises(RuntimeError, match="travel exhausted"):
        module.run_compensated_initialization(node, spec, bounded_contract())
    assert captures == [] and not any("useful calibration" in label for label, _, _ in stages)
    assert modes[-1] == ManagerEvent.NONE and node.spec is spec


def test_compensated_initialization_stale_inputs_stop_before_any_probe(monkeypatch):
    node, spec, stages, modes, captures = fake_calibration_node(monkeypatch)
    node.tip_time = -1.
    with pytest.raises(RuntimeError, match="fresh-input"):
        module.run_compensated_initialization(node, spec, bounded_contract())
    assert stages == captures == [] and modes[-1] == ManagerEvent.NONE


@pytest.mark.parametrize("phase,reason", [("PROVISIONAL", "confirmation lost"),
                                          ("FAILED", "confirmation failure")])
def test_compensated_initialization_lost_confirmation_aborts_without_fit(monkeypatch, phase, reason):
    node, spec, stages, modes, captures = fake_calibration_node(monkeypatch)
    original = node._run_home_stage
    def lose_confirmation(target, axes, deadline, label, *, stop_when):
        if "useful calibration" in label:
            node.controller_values["backlash_phase"] = json.dumps([phase]*3)
            return stop_when()
        return original(target, axes, deadline, label, stop_when=stop_when)
    node._run_home_stage = lose_confirmation
    with pytest.raises(RuntimeError, match=reason):
        module.run_compensated_initialization(node, spec, bounded_contract())
    assert captures == [] and node.spec is spec and modes[-1] == ManagerEvent.NONE


@pytest.mark.parametrize("ready", [True, False])
def test_position_stage_early_stop_preserves_manager_gate(monkeypatch, ready):
    monkeypatch.setattr(module.time, "monotonic", lambda: 0.)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    modes = []
    node = SimpleNamespace(manager_ready=ready, safety_time=0., position=np.zeros(6),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        _publish_mode=modes.append,
        _publish_home=lambda *a: pytest.fail("early stop must not publish a new target"))
    if ready:
        assert module.SparsePointExperiment._run_home_stage(
            node, np.ones(6), range(6), 1., "test", stop_when=lambda: True)
        assert modes == [ManagerEvent.NONE]
    else:
        with pytest.raises(RuntimeError, match="inhibited"):
            module.SparsePointExperiment._run_home_stage(
                node, np.ones(6), range(6), 1., "test", stop_when=lambda: True)
