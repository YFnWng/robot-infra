from pathlib import Path

from bringup.recording import RECORD_TOPICS


LAUNCH = (Path(__file__).resolve().parents[1]
          / "launch" / "simulation.launch.py")
CONTROL_LAUNCH = (Path(__file__).resolve().parents[1]
                  / "launch" / "control.launch.py")
CATHETER_PACKAGE = (Path(__file__).resolve().parents[2]
                    / "catheter_control" / "catheter_control")
SIMULATION_PACKAGE = (Path(__file__).resolve().parents[2]
                      / "simulation" / "simulation")
GPU_PROFILE = (Path(__file__).resolve().parents[2]
               / "catheter_control" / "config"
               / "gpu_mppi_1024_shadow.yaml")


def test_simulation_launch_has_no_serial_bridge():
    source = LAUNCH.read_text(encoding="utf-8")
    forbidden = ("device_serial_com.py", "/dev/tty", "serial_port")
    assert all(token not in source for token in forbidden)


def test_all_safety_critical_endpoints_are_explicitly_remapped():
    source = LAUNCH.read_text(encoding="utf-8")
    required = (
        '("/teleop/control", "/sim/teleop/control")',
        '("/teleop/event", "/sim/teleop/event")',
        '("/manager/control", "/sim/manager/control")',
        '("/manager/safety_status", "/sim/manager/safety_status")',
        '("/device/state", "/sim/device/state")',
        '("/device/command", "/sim/device/command")',
        '("/catheter_mppi/set_armed", "/sim/catheter_mppi/set_armed")',
        '("/catheter_mppi/track_tip_trajectory",',
    )
    assert all(endpoint in source for endpoint in required)


def test_simulation_launch_enables_output_only_behind_remaps():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'remappings=CONTROLLER_REMAPS' in source
    assert '"command_output_enabled": True' in source


def test_trajectory_action_is_launched_and_recorded_in_both_stacks():
    simulation = LAUNCH.read_text(encoding="utf-8")
    hardware = CONTROL_LAUNCH.read_text(encoding="utf-8")
    for source in (simulation, hardware):
        assert 'executable="catheter_tip_trajectory"' in source
    for suffix in ("feedback", "status"):
        assert f"track_tip_trajectory/_action/{suffix}" in simulation
        assert f"/catheter_mppi/track_tip_trajectory/_action/{suffix}" in RECORD_TOPICS
    assert "from bringup.recording import RECORD_TOPICS" in hardware
    assert "remappings=TRAJECTORY_REMAPS" in simulation
    assert ('"action_name": (\n'
            '                    "/sim/catheter_mppi/track_tip_trajectory")'
            in simulation)
    assert '"--include-hidden-topics"' in simulation
    assert '"--include-hidden-topics"' in hardware


def test_simulation_launch_requires_domain_isolation_and_single_instance():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"require_nondefault_domain", default_value="true"' in source
    assert "fcntl.LOCK_EX | fcntl.LOCK_NB" in source


def test_simulation_launch_defines_visualization_frame():
    source = LAUNCH.read_text(encoding="utf-8")
    assert ('package="tf2_ros", executable="static_transform_publisher"'
            in source)
    assert '"--child-frame-id", value("frame_id")' in source


def test_robustness_parameters_are_plant_side_and_truth_is_recorded():
    source = LAUNCH.read_text(encoding="utf-8")
    required = (
        '"marker_noise_std_mm"',
        '"marker_latency_ms"',
        '"actuator_gain"',
        '"plant_jacobian_angular_column_gain"',
        '"plant_jacobian_linear_column_gain"',
        '"truth_initial_interface_pose"',
        '"/sim/catheter_sim/ground_truth_markers"',
        '"/sim/catheter_sim/transmitted_state"',
        '"/sim/catheter_sim/visualization"',
        '"/tf_static"',
        '"/sim/catheter_sim/projected_control"',
        '"/sim/catheter_mppi/response_trace"',
        '"/sim/catheter_mppi/control_cycle_timing"',
    )
    assert all(item in source for item in required)


def test_simulation_defaults_to_manifest_and_exposes_mppi_exploration():
    source = LAUNCH.read_text(encoding="utf-8")
    assert "20260929_175554_grouped_no_rotation_v2.json" in source
    for parameter in (
            '"mppi_noise_std"', '"mppi_noise_correlation"',
            '"mppi_exploration_fraction"',
            '"mppi_reversal_backlash_rad"'):
        assert parameter in source
    assert 'max(0.15, 1.5/plan_rate_hz)' in source


def test_simulation_exposes_reproducible_grouped_plain_ab_profile():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"mppi_variant", default_value="grouped"' in source
    assert 'variant not in ("grouped", "plain", "custom")' in source
    assert 'grouped_mode_sampling = False' in source
    assert 'takeup_risk_weight = 0.0' in source
    assert '"mppi_takeup_risk_weight": takeup_risk_weight' in source
    assert '"mppi_takeup_confirmation_time_s"' in source
    assert '"mppi_grouped_mode_sampling": grouped_mode_sampling' in source
    assert '"controller_label": visualizer_label' in source


def test_path_preview_covers_effective_controller_horizon():
    simulation = LAUNCH.read_text(encoding="utf-8")
    hardware = CONTROL_LAUNCH.read_text(encoding="utf-8")

    assert "horizon_steps*rollout_step_s+0.20" in simulation
    assert '"preview_duration_s": path_preview_duration_s' in simulation
    assert "effective_horizon_steps*effective_rollout_step_s+0.20" in (
        hardware)
    assert '"preview_duration_s": path_preview_duration_s' in hardware


def test_simulated_truth_compute_is_isolated_from_controller_device():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"truth_model_device", default_value="cpu"' in source
    assert '"device": value("truth_model_device")' in source
    assert '"device": value("device")' in source


def test_model_manifest_routes_to_truth_and_controller():
    launch = LAUNCH.read_text(encoding="utf-8")
    perception = (SIMULATION_PACKAGE/"sim_perception.py"
                  ).read_text(encoding="utf-8")
    controller = (CATHETER_PACKAGE/"node.py"
                  ).read_text(encoding="utf-8")
    assert launch.count('"model_manifest"') >= 4
    assert '"model_manifest": effective_model_manifest' in launch
    assert "load_runtime_bundle" in perception
    assert 'get_parameter("model_manifest")' in perception
    assert "self.runtime_bundle, self.planner_runtime_bundle = load_runtime_pair" in controller
    assert 'get_parameter("model_manifest")' in controller


def test_legacy_model_launch_aliases_are_empty_and_fail_closed():
    launch = LAUNCH.read_text(encoding="utf-8")
    control_launch = CONTROL_LAUNCH.read_text(encoding="utf-8")
    for source in (launch, control_launch):
        assert '"cr_meta_lnn_root", default_value=""' in source
        assert '"cr_common_root", default_value=""' in source
        assert '"v171_distal_checkpoint", default_value=""' in source
        assert "resolve_model_selection" in source

def test_hardware_launch_exposes_robust_adaptation_but_defaults_fixed():
    source = CONTROL_LAUNCH.read_text(encoding="utf-8")
    assert '"adaptation_enabled", default_value="false"' in source
    assert '"model_manifest"' in source
    required = (
        '"adaptation_minimum_observations"',
        '"adaptation_minimum_normalized_action"',
        '"adaptation_minimum_rotation_deg"',
        '"adaptation_minimum_translation_mm"',
        '"adaptation_minimum_response_snr"',
        '"adaptation_reversal_holdoff_normalized_action"',
        '"adaptation_reversal_holdoff_normalized_action_shaft_0"',
        '"adaptation_reversal_holdoff_normalized_action_shaft_1"',
        '"adaptation_reversal_holdoff_normalized_action_shaft_2"',
        '"adaptation_confirmation_windows"',
        '"adaptation_minimum_column_gain"',
        '"adaptation_maximum_column_gain"',
    )
    assert all(item in source for item in required)


def test_gpu_profile_is_a_separate_non_actuating_performance_overlay():
    launch = CONTROL_LAUNCH.read_text(encoding="utf-8")
    profile = GPU_PROFILE.read_text(encoding="utf-8")

    assert '"performance_config", default_value=""' in launch
    assert "node_parameters.append" in launch
    assert "device: cuda" in profile
    assert "samples: 1024" in profile
    assert "horizon_steps: 4" in profile
    assert "command_output_enabled:" not in profile
    assert "_reject_profile_output_interlock" in launch


def test_hardware_output_interlock_is_final_and_shadow_is_fail_closed():
    source = CONTROL_LAUNCH.read_text(encoding="utf-8")

    performance_append = source.index(
        'node_parameters.append(_reject_profile_output_interlock(\n'
        '            performance_config, "performance_config"))')
    final_interlock = source.index(
        'node_parameters.append(Parameter(\n'
        '        "command_output_enabled", command_output_enabled,')
    assert final_interlock > performance_append
    assert 'from launch_ros.parameter_descriptions import Parameter' in source
    assert 'value_type=bool' in source
    assert 'must not declare command_output_enabled' in source
    assert (
        'command_output_enabled=true cannot be combined with a '
        in source)
    assert '"performance_config", default_value=""' in source


def test_hardware_launch_records_complete_response_path_by_default():
    source = CONTROL_LAUNCH.read_text(encoding="utf-8")
    required_topics = (
        '"/teleop/control"',
        '"/manager/control"',
        '"/manager/state"',
        '"/device/command_tx"',
        '"/device/state"',
        '"/device/transport_status"',
        '"/shape_tracking/markers"',
        '"/catheter_mppi/response_trace"',
        '"/catheter_mppi/control_cycle_timing"',
        '"/catheter_mppi/status"',
        '"/rosout"',
    )
    assert all(topic.strip('"') in RECORD_TOPICS for topic in required_topics)
    assert "from bringup.recording import RECORD_TOPICS" in source
    assert '"record", default_value="true"' in source
    assert 'output + "_manifest.json"' in source


def test_hardware_controller_requires_all_commanded_axes_engaged():
    controller = (CATHETER_PACKAGE/"node.py"
                  ).read_text(encoding="utf-8")
    assert "self.raw_response_during_interface_takeup = np.zeros(3, dtype=bool)" in controller
    assert "self.takeup_response_free_mask = np.ones(3, dtype=bool)" in controller
    assert "distal_confirmation_enabled=True" in controller


def test_semantic_stacks_are_resolved_and_recorded_by_both_launches():
    simulation = LAUNCH.read_text(encoding="utf-8")
    hardware = CONTROL_LAUNCH.read_text(encoding="utf-8")
    for source in (simulation, hardware):
        assert '"stack_config", default_value=""' in source
        assert "resolve_stack(" in source
        assert "resolved_configuration.as_manifest()" in source
    assert "controller_profile_parameters" in simulation
    assert "stack_config cannot be combined with legacy" in hardware
