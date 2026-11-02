from pathlib import Path

import numpy as np
import pytest
import yaml

from control_tasks.sparse_point_experiment import (
    SparsePointExperiment, accepted_home_target, decoupled_tendon_prehome_target, load_sparse_point_experiment,
    sparse_circle_targets)
from control_interface.msg import ManagerEvent
from types import SimpleNamespace


@pytest.mark.parametrize("failure", [None, RuntimeError("home failed"), KeyboardInterrupt()])
def test_home_only_never_generates_or_reaches(tmp_path, monkeypatch, failure):
    from control_tasks import sparse_point_experiment as module
    events = []
    class Node:
        tip = np.array([.01, .02, .03])
        def wait_for_inputs(self, timeout):
            events.append("inputs")
        def home(self, index):
            events.append("home")
            if failure is not None:
                raise failure
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))
        def cancel_active(self):
            events.append("cancel")
        def _publish_mode(self, mode):
            assert mode == ManagerEvent.NONE
        def get_logger(self):
            return SimpleNamespace(warn=lambda _: None)
        def destroy_node(self):
            events.append("destroy")
    monkeypatch.setattr(module, "SparsePointExperiment", lambda _: Node())
    monkeypatch.setattr(module.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: events.append("shutdown"))
    journal = tmp_path / "home.jsonl"
    args = ["task", str(CONFIG_ROOT / "experiments/sparse_points/farther_tendon_12_plain.yaml"),
            "--home-only", "--trial-records", str(journal)]
    if failure is None:
        module.main(args)
        assert '"home_only": true' in journal.read_text()
        assert '"event": "home_completed"' in journal.read_text()
    else:
        with pytest.raises(type(failure)):
            module.main(args)
        assert '"event": "home_completed"' not in journal.read_text()
    assert events[:2] == ["inputs", "home"]
    assert events[-2:] == ["destroy", "shutdown"]


@pytest.mark.parametrize("failure", [None, "reach", "final_home"])
def test_task_homes_after_normal_results_not_fault_cleanup(tmp_path, monkeypatch, failure):
    from control_tasks import sparse_point_experiment as module
    events = []
    class Node:
        tip = np.array([.01, .02, .03])
        def wait_for_inputs(self, timeout): pass
        def home(self, index):
            events.append(("home", index))
            if failure == "final_home" and index == 1:
                raise RuntimeError("home failed")
        def generate_targets(self, tip): return np.array([[.02, .02, .03]])
        def run_target(self, index, count, target):
            events.append(("reach", index))
            if failure == "reach": raise RuntimeError("controller faulted")
            return SimpleNamespace(reached_waypoints=0, timed_out_waypoints=1, final_error_mm=3.)
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))
        def cancel_active(self): events.append(("cancel", None))
        def _publish_mode(self, mode): assert mode == ManagerEvent.NONE
        def get_logger(self): return SimpleNamespace(info=lambda _: None)
        def destroy_node(self): pass
    monkeypatch.setattr(module, "SparsePointExperiment", lambda _: Node())
    monkeypatch.setattr(module.rclpy, "init", lambda **_: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: None)
    journal = tmp_path / "task.jsonl"
    args = ["task", str(CONFIG_ROOT / "experiments/sparse_points/farther_tendon_12_plain.yaml"),
            "--trial-records", str(journal)]
    if failure:
        with pytest.raises(RuntimeError): module.main(args)
        assert '"event": "task_completed"' not in journal.read_text()
    else:
        module.main(args)
        assert events == [("home", 0), ("reach", 0), ("home", 1)]
        assert '"event": "final_home_completed"' in journal.read_text()
        assert '"event": "task_completed"' in journal.read_text()
    if failure == "reach":
        assert ("home", 1) not in events


def test_frozen_preview_preserves_task_guards_and_never_overwrites(tmp_path):
    from control_tasks.sparse_point_experiment import save_frozen_targets
    source=tmp_path/'preview.yaml'
    config={'required_controller_status':{'engagement_control_enabled':True},
            'generator':{'type':'controller_model_joint_displacements'}}
    source.write_text(yaml.safe_dump(config))
    output=tmp_path/'frozen.yaml'
    save_frozen_targets(source,output,[[.01,.02,.03]])
    frozen=yaml.safe_load(output.read_text())
    assert frozen['required_controller_status']==config['required_controller_status']
    assert frozen['generator']=={'type':'absolute_targets_m','targets_m':[[.01,.02,.03]]}
    assert frozen['target_preview']['semantics']=='hypothetical_post_engagement_shape_displacement'
    assert frozen['target_preview']['takeup_execution_feasibility_certified'] is False
    with pytest.raises(FileExistsError):save_frozen_targets(source,output,[[0.,0.,0.]])


def test_g5_baseline_targets_survive_freeze_load_round_trip(tmp_path):
    from control_tasks.sparse_point_experiment import save_frozen_targets
    configs = Path(__file__).resolve().parents[2]/'catheter_control/config/experiments/sparse_points'
    source = configs/'engagement_twoaxis.yaml'
    spec = load_sparse_point_experiment(source)
    baseline = load_sparse_point_experiment(configs/'farther_tendon_12_grouped.yaml')
    targets = sparse_circle_targets(spec, np.ones(3))
    np.testing.assert_array_equal(targets, sparse_circle_targets(baseline, np.zeros(3)))
    output = tmp_path/'frozen.yaml'
    save_frozen_targets(source, output, targets)
    restored = load_sparse_point_experiment(output)
    np.testing.assert_array_equal(sparse_circle_targets(restored, np.zeros(3)), targets)
    assert restored.required_controller_status['engagement_control_enabled'] is True
    assert 'target_preview' not in yaml.safe_load(output.read_text())


@pytest.mark.parametrize("status", [ManagerEvent.POSITION_TIMED_OUT,
                                  ManagerEvent.POSITION_REJECTED])
def test_home_terminal_event_is_reported_without_retry(status):
    node = SimpleNamespace(home_stage_start_ns=2_000_000_000,
                           home_transaction_failure=None)
    event = ManagerEvent()
    event.header.stamp.sec = 1
    event.predicate = ManagerEvent.POSITION_STATUS
    event.state = [1, status, 4]
    event.text = "POSITION_TIMED_OUT"
    event.data = [1., float(status), 4., 0., 0., -6.4018, 0., 0., 0.]
    SparsePointExperiment._home_event_cb(node, event)
    assert node.home_transaction_failure is None  # Previous transaction.
    event.header.stamp.sec = 3
    SparsePointExperiment._home_event_cb(node, event)
    assert "mask=0x04" in node.home_transaction_failure
    assert "-6.4018" in node.home_transaction_failure
    node.home_stage_start_ns = None
    node.home_transaction_failure = None
    SparsePointExperiment._home_event_cb(node, event)
    assert node.home_transaction_failure is None


CONFIG_ROOT = (Path(__file__).resolve().parents[2]
               / "catheter_control" / "config")


CONFIG = (CONFIG_ROOT / "sparse_circle_points_sim.yaml")
HARDWARE_CONFIG = (CONFIG_ROOT / "sparse_circle_points_hardware.yaml")
HISTORY_SIM_CONFIG = (CONFIG_ROOT
                      / "sparse_circle_points_sim_history_preserving.yaml")
AXIAL_HARDWARE_CONFIG = (CONFIG_ROOT / "axial_points_v174_hardware.yaml")
V175_AXIAL_HARDWARE_CONFIG = (
    CONFIG_ROOT
    / "axial_points_v175_hardware_no_rotation.yaml")
V175_TWO_AXIS_HARDWARE_CONFIG = (
    CONFIG_ROOT
    / "two_axis_points_v175_hardware_no_rotation.yaml")
V175_TWO_AXIS_SIM_CONFIG = (
    CONFIG_ROOT
    / "two_axis_points_v175_sim_no_rotation.yaml")
V175_FAR_TWO_AXIS_HARDWARE_CONFIG = (
    CONFIG_ROOT
    / "far_two_axis_points_v175_hardware_no_rotation.yaml")
V175_FAR_TWO_AXIS_SIM_CONFIG = (
    CONFIG_ROOT
    / "far_two_axis_points_v175_sim_no_rotation.yaml")
V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG = (
    CONFIG_ROOT
    / "farther_two_axis_points_v175_hardware_no_rotation.yaml")
V175_TENDON_12_TWO_AXIS_HARDWARE_CONFIG = (
    CONFIG_ROOT
    / "farther_tendon_12_two_axis_points_v175_hardware_no_rotation.yaml")
V175_PLAIN_TAKEUP_TARGET_CONFIG = (
    CONFIG_ROOT
    / "farther_tendon_12_two_axis_points_v175_plain_takeup_hardware_no_rotation.yaml")
V171_PLAIN_TARGET_CONFIG = (
    CONFIG_ROOT
    / "farther_tendon_12_two_axis_points_v171_plain_hardware_no_rotation.yaml")
V175_PLAIN_TAKEUP_PROFILE = (
    CONFIG_ROOT
    / "v175_plain_takeup_hardware_no_rotation.yaml")
V171_PLAIN_PROFILE = (
    CONFIG_ROOT
    / "v171_plain_hardware_no_rotation.yaml")
V174_PROFILE = (CONFIG_ROOT / "legacy/v174_fixed_hardware_no_rotation.yaml")
CAUSAL_V2_PROFILE = (CONFIG_ROOT / "legacy/causal_v2_fixed_hardware.yaml")
V175_NO_ROTATION_PROFILE = (
    CONFIG_ROOT
    / "v175_grouped_hardware_no_rotation.yaml")


def test_sparse_circle_uses_eight_unique_absolute_targets():
    spec = load_sparse_point_experiment(CONFIG)
    home_tip = np.array([.020, .017, .060])
    targets = sparse_circle_targets(spec, home_tip)

    assert targets.shape == (8, 3)
    assert len(np.unique(np.round(targets, decimals=12), axis=0)) == 8
    assert targets[:, 0] == pytest.approx(np.full(8, .035))
    center_yz = np.array([0.0, .060])
    assert np.linalg.norm(
        targets[:, 1:]-center_yz, axis=1) == pytest.approx(
            np.full(8, .010))


def test_sparse_experiment_has_reviewed_home_and_independent_action():
    spec = load_sparse_point_experiment(CONFIG)

    assert spec.home_position == pytest.approx([20, 0, 0, 0, 0, 0])
    assert np.all(spec.home_speed > 0.0)
    assert np.all(spec.home_tolerance > 0.0)
    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.home_mode_settle_s == pytest.approx(0.15)
    assert spec.action_name == "/sim/catheter_mppi/track_tip_trajectory"
    assert spec.generator["close_circle"] is False
    assert spec.plant_reset_mode == "full_simulation"
    assert spec.plant_reset_services == (
        "/sim/catheter_sim/reset_plant",
        "/sim/catheter_sim/reset_perception",
        "/sim/catheter_mppi/reset_simulation_state")


def test_hardware_sparse_experiment_preserves_physical_history():
    spec = load_sparse_point_experiment(HARDWARE_CONFIG)

    assert spec.plant_reset_mode == "history_preserving"
    assert spec.plant_reset_services == ()


def test_axial_hardware_targets_are_frozen_relative_to_home_tip():
    spec = load_sparse_point_experiment(AXIAL_HARDWARE_CONFIG)
    home_tip = np.array([.021, .017, .071])
    targets = sparse_circle_targets(spec, home_tip)

    assert spec.plant_reset_mode == "history_preserving"
    assert spec.home_position == pytest.approx([20, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(
        targets,
        np.asarray([
            [.021, .017, .076],
            [.021, .017, .066],
        ]),
        rtol=0.0,
        atol=1.0e-12,
    )


def test_v175_axial_hardware_test_is_fail_closed_no_rotation():
    spec = load_sparse_point_experiment(V175_AXIAL_HARDWARE_CONFIG)

    assert spec.rotation_guard_enabled is True
    assert spec.rotation_guard_axis == 1
    assert spec.rotation_guard_tolerance == pytest.approx(1.0e-9)
    assert spec.required_controller_status == {
        "command_output_enabled": True,
        "marker_estimator": "ukf",
        "model_adaptation_enabled": False,
        "engaged_gain_enabled": True,
        "mppi_engaged_gain_scenarios": True,
        "mppi_samples": 512,
        "mppi_point_rollout_step_s": 0.2,
        "mppi_point_rollout_coarse_steps": True,
        "mppi_point_prediction_tail_steps": 0,
        "controller_velocity_max": [10.0, 0.0, 4.5, 4.0, 25.0, 25.0],
    }


def test_v175_two_axis_targets_are_model_generated_and_reserve_qualified():
    spec = load_sparse_point_experiment(V175_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.generator["type"] == (
        "controller_model_joint_displacements")
    assert spec.generator["logical_displacements"] == [
        [5.0, 0.0, 0.0],
        [0.0, 0.0, 3.0],
        [7.0, 0.0, 3.0],
        [-4.0, 0.0, 3.0],
    ]
    assert spec.generator["minimum_endpoint_reserve"] == [
        2.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True

    with pytest.raises(ValueError, match="preview service"):
        sparse_circle_targets(spec, np.array([.02, .01, .07]))


def test_v175_two_axis_sim_uses_full_reset_and_simulated_preview_service():
    spec = load_sparse_point_experiment(V175_TWO_AXIS_SIM_CONFIG)

    assert spec.home_tolerance == pytest.approx(
        [0.1, 0.5, 0.1, 0.1, 0.5, 0.5])
    assert spec.plant_reset_mode == "full_simulation"
    assert spec.plant_reset_services == (
        "/sim/catheter_sim/reset_plant",
        "/sim/catheter_sim/reset_perception",
        "/sim/catheter_mppi/reset_simulation_state",
    )
    assert spec.generator["service"] == (
        "/sim/catheter_mppi/generate_sparse_targets")
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.required_controller_status["controller_velocity_max"][1] == 0.0


@pytest.mark.parametrize("path", [
    V175_FAR_TWO_AXIS_HARDWARE_CONFIG,
    V175_FAR_TWO_AXIS_SIM_CONFIG,
])
def test_v175_far_targets_are_per_home_model_generated(path):
    spec = load_sparse_point_experiment(path)

    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [8.0, 0.0, 0.0],
        [0.0, 0.0, 4.5],
        [-8.0, 0.0, 3.0],
        [-6.0, 0.0, 4.5],
    ]
    assert spec.generator["minimum_tip_displacement_mm"] == 3.0
    assert spec.generator["maximum_tip_displacement_mm"] == 20.0
    assert spec.rotation_guard_enabled is True


def test_v175_farther_hardware_targets_extend_qualified_block():
    spec = load_sparse_point_experiment(
        V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.target_timeout_s == pytest.approx(30.0)
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [10.0, 0.0, 0.0],
        [0.0, 0.0, 6.0],
        [-10.0, 0.0, 4.5],
        [-8.0, 0.0, 6.0],
    ]
    assert spec.generator["minimum_endpoint_reserve"] == [
        2.0, 0.0, 0.5, 0.0, 0.0, 0.0]
    assert spec.generator["minimum_tip_displacement_mm"] == 4.0
    assert spec.generator["maximum_tip_displacement_mm"] == 25.0
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True


def test_v175_tendon_12_targets_preserve_insertion_range():
    baseline = load_sparse_point_experiment(
        V175_FARTHER_TWO_AXIS_HARDWARE_CONFIG)
    spec = load_sparse_point_experiment(
        V175_TENDON_12_TWO_AXIS_HARDWARE_CONFIG)

    assert spec.target_timeout_s == pytest.approx(30.0)
    assert spec.generator["regenerate_after_each_home"] is True
    assert spec.generator["logical_displacements"] == [
        [10.0, 0.0, 0.0],
        [0.0, 0.0, 12.0],
        [-10.0, 0.0, 9.0],
        [-8.0, 0.0, 12.0],
    ]
    assert [row[0] for row in spec.generator["logical_displacements"]] == [
        row[0] for row in baseline.generator["logical_displacements"]]
    assert spec.generator["minimum_endpoint_reserve"] == [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert spec.generator["minimum_tip_displacement_mm"] == 4.0
    assert spec.generator["maximum_tip_displacement_mm"] == 35.0
    assert spec.decoupled_tendon_prehome is True
    assert spec.rotation_guard_enabled is True


@pytest.mark.parametrize("path, expected", [
    (
        CONFIG_ROOT / "experiments/sparse_points/farther_tendon_12_plain_with_takeup_fixed_gain.yaml",
        {
            "backlash_compensation_enabled": True,
            "takeup_transaction_enabled": True,
            "backlash_model_encoder_input": "estimated_transmitted",
            "engaged_gain_enabled": False,
            "mppi_engaged_gain_scenarios": False,
        },
    ),
    (
        V175_PLAIN_TAKEUP_TARGET_CONFIG,
        {
            "backlash_compensation_enabled": True,
            "takeup_transaction_enabled": True,
            "backlash_model_encoder_input": "estimated_transmitted",
            "engaged_gain_enabled": True,
            "mppi_engaged_gain_scenarios": True,
        },
    ),
    (
        V171_PLAIN_TARGET_CONFIG,
        {
            "backlash_compensation_enabled": False,
            "takeup_transaction_enabled": False,
            "backlash_model_encoder_input": "raw_shaft",
            "engaged_gain_enabled": False,
            "mppi_engaged_gain_scenarios": False,
        },
    ),
])
def test_plain_baseline_targets_require_exact_runtime_identity(path, expected):
    spec = load_sparse_point_experiment(path)

    assert spec.generator["type"] == "absolute_targets_m"
    np.testing.assert_allclose(spec.generator["targets_m"], [
        [0.02608313597738743, 0.022739075124263763, 0.08448133617639542],
        [0.03320372849702835, 0.02986534871160984, 0.06324673444032669],
        [0.031191052868962288, 0.02978038229048252, 0.0510878749191761],
        [0.031725503504276276, 0.029091984033584595, 0.050746046006679535],
    ], rtol=0.0, atol=1.0e-12)
    np.testing.assert_allclose(
        sparse_circle_targets(spec, np.zeros(3)),
        spec.generator["targets_m"], rtol=0.0, atol=0.0)
    common = {
        "command_output_enabled": True,
        "marker_estimator": "ukf",
        "model_adaptation_enabled": False,
        "mppi_grouped_mode_sampling": False,
        "mppi_best_candidate_guard": False,
        "mppi_takeup_risk_cost_weight": 0,
        "reversal_mode_selector": "plain_mppi",
        "mppi_samples": 512,
        "mppi_point_rollout_step_s": 0.2,
        "mppi_point_rollout_coarse_steps": True,
        "mppi_point_prediction_tail_steps": 0,
        "controller_velocity_max": [10.0, 0.0, 4.5, 4.0, 25.0, 25.0],
    }
    assert spec.required_controller_status == {**common, **expected}


def test_plain_hardware_profiles_are_controlled_ablations():
    grouped = yaml.safe_load(
        V175_NO_ROTATION_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]
    compensated = yaml.safe_load(
        V175_PLAIN_TAKEUP_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]
    uncompensated = yaml.safe_load(
        V171_PLAIN_PROFILE.read_text(encoding="utf-8"))[
            "catheter_mppi"]["ros__parameters"]

    assert set(compensated) == set(grouped)
    assert {
        key for key in grouped if compensated[key] != grouped[key]
    } == {
        "mppi_grouped_mode_sampling",
        "mppi_best_candidate_guard",
        "mppi_takeup_risk_weight",
        "reversal_scheduler_enabled",
    }
    assert compensated["backlash_compensation_enabled"] is True
    assert compensated["takeup_transaction_enabled"] is True

    assert set(uncompensated) == set(grouped)
    assert {
        key for key in grouped if uncompensated[key] != grouped[key]
    } == {
        "backlash_compensation_enabled",
        "takeup_transaction_enabled",
        "engaged_gain_enabled",
        "mppi_grouped_mode_sampling",
        "mppi_engaged_gain_scenarios",
        "mppi_best_candidate_guard",
        "mppi_takeup_risk_weight",
        "reversal_scheduler_enabled",
    }
    # Planner/compensator ablations share the launch-selected model bundle.


def test_decoupled_tendon_prehome_holds_physical_chassis_axis():
    position = np.asarray([16.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    home = np.asarray([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    target = decoupled_tendon_prehome_target(position, home)

    assert target == pytest.approx([13.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert target[0]-target[2] == pytest.approx(position[0]-position[2])


def test_limit_projected_tendon_prehome_endpoint_and_exact_final_home():
    requested = [-12.5139, 0., 0., 0., 0., 0.]
    approved = [-10., 0., 0., 0., 0., 0.]
    assert accepted_home_target(requested, approved, True) == pytest.approx(approved)
    with pytest.raises(ValueError):
        accepted_home_target(requested, approved)
    with pytest.raises(ValueError):
        accepted_home_target(requested, [-10., 0., .1, 0., 0., 0.], True)
    with pytest.raises(ValueError):
        accepted_home_target(requested, [float("nan")] * 6, True)


def test_manager_acknowledgement_is_source_and_stamp_matched():
    from types import SimpleNamespace
    from control_interface.msg import DeviceStream
    from control_tasks.sparse_point_experiment import SparsePointExperiment
    node = SimpleNamespace(spec=SimpleNamespace(source_name="autonomy"),
                           home_request_stamp=123, home_accepted_target=None,
                           monitor_rotation=False)
    message = DeviceStream(predicate=DeviceStream.POS, data=[0.] * 12)
    message.header.stamp.nanosec = 122
    message.header.frame_id = "autonomy"
    SparsePointExperiment._manager_control_cb(node, message)
    assert node.home_accepted_target is None
    message.header.stamp.nanosec = 123
    message.header.frame_id = "other"
    SparsePointExperiment._manager_control_cb(node, message)
    assert node.home_accepted_target is None
    message.header.frame_id = "autonomy"
    SparsePointExperiment._manager_control_cb(node, message)
    assert node.home_accepted_target == pytest.approx([0.] * 6)


def test_prehome_settles_at_projected_endpoint_then_requires_exact_home(monkeypatch):
    from types import SimpleNamespace
    from control_tasks import sparse_point_experiment as module
    now = [0.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    warnings = []
    node = SimpleNamespace(manager_ready=True, safety_time=0., position_time=0.,
                           position=np.asarray([-9.9985, 0., .0001, 0., 0., 0.]),
                           spec=SimpleNamespace(home_tolerance=np.asarray([.1] * 6),
                                                home_settle_s=.1))
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warn=warnings.append)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(now[0]*1e9)))
    def publish(target):
        node.home_accepted_target = np.asarray(target).copy()
        node.home_accepted_target[0] = max(-10., target[0])
    node._publish_home = publish
    def spin(*args, **kwargs):
        now[0] += .02
        node.safety_time = node.position_time = now[0]
    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    module.SparsePointExperiment._run_home_stage(
        node, [-12.5139, 0., 0., 0., 0., 0.], [0, 2], 1., "prehome", True)
    assert len(warnings) == 1
    # Final home must not falsely succeed just because tendon is already zero.
    with pytest.raises(RuntimeError, match="home timed out"):
        module.SparsePointExperiment._run_home_stage(
            node, [20., 0., 0., 0., 0., 0.], range(6), now[0] + .2, "home")


@pytest.mark.parametrize("status,progress,second_failure,allowed", [
    ("timeout", True, False, True),
    ("timeout", False, False, False),
    ("rejected", True, False, False),
    ("timeout", True, True, False),
])
def test_home_retry_is_bounded_and_requires_progress(monkeypatch, status, progress,
                                                   second_failure, allowed):
    from types import SimpleNamespace
    from control_tasks import sparse_point_experiment as module
    now = [0.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    node = SimpleNamespace(manager_ready=True, safety_time=0., position_time=0.,
                           position=np.asarray([10., 0., 10., 0., 0., 0.]),
                           spec=SimpleNamespace(home_tolerance=np.asarray([.1]*6),
                                                home_settle_s=.1))
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(now[0]*1e9)))
    commands, stops = [], []
    def publish(target):
        commands.append(np.asarray(target).copy())
        node.home_accepted_target = np.asarray(target).copy()
    node._publish_home = publish
    def stop(*args):
        stops.append(now[0])
    monkeypatch.setattr(module.SparsePointExperiment, "_stop_before_home_retry", stop)
    failures = [0]
    def spin(*args, **kwargs):
        now[0] += .02
        node.safety_time = node.position_time = now[0]
        if (failures[0] == 0 and now[0] >= .04) or (second_failure and failures[0] == 1 and stops):
            node.position[0] = node.position[2] = 5. if progress else 10.
            node.home_transaction_failure = "test failure"
            node.home_failure_status = (module.ManagerEvent.POSITION_TIMED_OUT
                                       if status == "timeout" else module.ManagerEvent.POSITION_REJECTED)
            node.home_failure_mask = 4
            node.home_failure_time = now[0]
            failures[0] += 1
        elif stops:
            node.position[:] = 0.
    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    if allowed:
        module.SparsePointExperiment._run_home_stage(node, np.zeros(6), [0, 2], 1., "home")
        assert len(stops) == 1
        assert all(np.array_equal(command, np.zeros(6)) for command in commands)
    else:
        with pytest.raises(RuntimeError, match="retry not eligible"):
            module.SparsePointExperiment._run_home_stage(node, np.zeros(6), [0, 2], 1., "home")
        assert len(stops) == int(second_failure)


@pytest.mark.parametrize("moving,ready,fresh", [(False, True, True),
                                               (True, True, True),
                                               (False, False, True),
                                               (False, True, False)])
def test_home_retry_stop_requires_fresh_stationary_ready_feedback(monkeypatch, moving, ready, fresh):
    from types import SimpleNamespace
    from control_tasks import sparse_point_experiment as module
    now = [0.]
    monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    modes = []
    node = SimpleNamespace(manager_ready=ready, safety_time=0., position_time=0.,
                           position=np.zeros(6), _publish_mode=modes.append,
                           spec=SimpleNamespace(home_tolerance=np.asarray([.1]*6),
                                                home_mode_settle_s=.04))
    def spin(*args, **kwargs):
        now[0] += .02
        node.safety_time = now[0]
        if fresh:
            node.position_time = now[0]
        if moving:
            node.position[2] += .1
    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    if ready and fresh and not moving:
        module.SparsePointExperiment._stop_before_home_retry(node, 1., 0.)
        assert modes[-1] == module.ManagerEvent.JOINT_POS
    else:
        with pytest.raises(RuntimeError):
            module.SparsePointExperiment._stop_before_home_retry(node, 1., 0.)
        assert module.ManagerEvent.JOINT_POS not in modes


def test_v175_no_rotation_profile_only_disables_rotation_velocity():
    baseline = yaml.safe_load(
        (CONFIG_ROOT
         / "v175_grouped_hardware.yaml").read_text(encoding="utf-8"))
    isolated = yaml.safe_load(
        V175_NO_ROTATION_PROFILE.read_text(encoding="utf-8"))
    baseline_parameters = baseline["catheter_mppi"]["ros__parameters"]
    isolated_parameters = isolated["catheter_mppi"]["ros__parameters"]

    assert set(baseline_parameters) == set(isolated_parameters)
    changed = {
        key for key in baseline_parameters
        if baseline_parameters[key] != isolated_parameters[key]
    }
    assert changed == {"controller_velocity_max"}
    assert isolated_parameters["controller_velocity_max"] == pytest.approx(
        [10.0, 0.0, 4.5, 4.0, 25.0, 25.0])


def test_v174_no_rotation_profile_only_changes_jacobian_artifact():
    causal = yaml.safe_load(CAUSAL_V2_PROFILE.read_text(encoding="utf-8"))
    v174 = yaml.safe_load(V174_PROFILE.read_text(encoding="utf-8"))
    causal_parameters = causal["catheter_mppi"]["ros__parameters"]
    v174_parameters = v174["catheter_mppi"]["ros__parameters"]

    assert set(causal_parameters) == set(v174_parameters)
    changed = {
        key for key in causal_parameters
        if causal_parameters[key] != v174_parameters[key]
    }
    assert changed == {"jacobian_initialization_json"}
    assert v174_parameters["jacobian_initialization_json"].endswith(
        "/real_joint_local_distal_v174.json")
    assert v174_parameters["adaptation_enabled"] is False
    assert v174_parameters["controller_velocity_max"][1] == 0.0


@pytest.mark.parametrize("offsets", [
    [[0.0, 0.0, 0.0]],
    [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
])
def test_relative_sparse_targets_reject_zero_or_duplicate_offsets(
        tmp_path, offsets):
    content = AXIAL_HARDWARE_CONFIG.read_text(encoding="utf-8")
    document = yaml.safe_load(content)
    document["generator"]["offsets_mm"] = offsets
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError):
        load_sparse_point_experiment(path)


def test_history_preserving_simulation_is_an_explicit_comparison_mode():
    spec = load_sparse_point_experiment(HISTORY_SIM_CONFIG)

    assert spec.action_name.startswith("/sim/")
    assert spec.plant_reset_mode == "history_preserving"
    assert spec.plant_reset_services == ()
