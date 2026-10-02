"""Canonical topics for hardware controller recording."""

RECORD_TOPICS = [
    "/teleop/control", "/teleop/event", "/manager/control", "/manager/event",
    "/manager/safety_status", "/manager/state", "/device/state", "/device/event",
    "/device/command_tx", "/device/transport_status", "/shape_tracking/markers",
    "/shape_tracking/marker_status", "/collection/events",
    "/catheter_mppi/target_tip", "/catheter_mppi/reference_horizon",
    "/catheter_mppi/reference_path", "/catheter_mppi/path_reference_point",
    "/catheter_mppi/path_tracking_trace", "/catheter_mppi/planned_control",
    "/catheter_mppi/predicted_tip", "/catheter_mppi/response_trace",
    "/catheter_mppi/estimator_trace", "/catheter_mppi/control_cycle_timing",
    "/catheter_mppi/status", "/catheter_mppi/shadow_request",
    "/catheter_mppi/shadow_decision", "/catheter_mppi/shadow_timing",
    "/catheter_mppi/shadow_status", "/catheter_mppi/shadow_worker_status",
    "/catheter_mppi/track_tip_trajectory/_action/feedback",
    "/catheter_mppi/track_tip_trajectory/_action/status",
    "/catheter_mppi/track_tip_path/_action/feedback",
    "/catheter_mppi/track_tip_path/_action/status", "/parameter_events", "/rosout",
]
