from catheter_control.orchestration.configuration import (
    ACTIVE_CONTROLLER_PARAMETERS,
)
from catheter_control.orchestration.parameters import declare_parameters


class RecordingNode:
    def __init__(self):
        self.declarations = []

    def declare_parameter(self, name, default):
        assert name not in {item[0] for item in self.declarations}
        self.declarations.append((name, default))


def test_controller_parameter_surface_is_complete_and_stable():
    node = RecordingNode()

    declare_parameters(node, source_name="catheter_mppi")

    names = [name for name, _ in node.declarations]
    values = dict(node.declarations)
    assert len(names) == 148
    assert values["estimator_backend"] == "v171"
    assert values["reduced_filter_image_period_ms"] == 90.0
    assert values["reduced_filter_startup_pending_images"] == 8
    assert values["reduced_filter_tendon_upper_mm"] == 10.8
    assert values["synchronized_sheath_rotation"] is False
    assert values["backlash_tendon_takeup_joint_range_enabled"] is False
    assert values["backlash_minimum_distal_marker_deformation_mm"] == 0.3
    assert values["backlash_distal_confirmation_release_ratio"] == 0.75
    assert names[:10] == [
        "source_name",
        "command_output_enabled",
        "simulation_state_reset_enabled",
        "task_gc_enabled",
        "task_gc_maximum_s",
        "task_gc_growth_mb",
        "frame_id",
        "model_manifest",
        "engagement_control_enabled",
        "adaptation_enabled",
    ]
    assert names[-2:] == ["marker_topic", "marker_diagnostic_topic"]
    assert set(ACTIVE_CONTROLLER_PARAMETERS) <= set(names)
    assert values["source_name"] == "catheter_mppi"
    assert values["command_output_enabled"] is False
    assert values["task_gc_enabled"] is True
    assert values["task_gc_maximum_s"] == 300.0
    assert values["task_gc_growth_mb"] == 256.0
    assert values["model_manifest"] == ""
    assert values["engagement_control_enabled"] is False
    assert values["estimator_device"] == ""
    assert values["planner_process_enabled"] is False
    assert values["estimator_dtype"] == ""
    assert values["marker_estimator"] == "gauss_newton"
    assert values["backlash_response_window_max_s"] == 1.0
    assert values["samples"] == 32
    assert values["mppi_temperature"] == 1.0
    assert values["horizon_steps"] == 4
    assert values["maximum_planner_deadline_misses"] == 3
    assert values["marker_topic"] == "/shape_tracking/markers"
    assert values["marker_diagnostic_topic"] == (
        "/shape_tracking/marker_status")
