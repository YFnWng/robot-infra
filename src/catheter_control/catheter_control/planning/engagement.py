"""Unit conversion and snapshot views; shared backlash owner confirms motion."""
import copy
from dataclasses import replace
import numpy as np


def tendon_confirmation(runtime, snapshot):
    """Convert the baseline owner's shaft-radian direction to motor-mm."""
    from cr_meta_lnn.deployment.experimental.joint_engagement_filter import TendonConfirmation
    return TendonConfirmation(snapshot.phase[2],
        int(snapshot.motion_direction[2]*np.sign(runtime.tendon_mm_per_shaft_rad)),
        int(snapshot.confirmation_count[2]))


def confirmation_from_checkpoint(runtime, checkpoint):
    """Read a shared owner checkpoint, without restoring/mutating that owner."""
    from cr_meta_lnn.deployment.experimental.joint_engagement_filter import TendonConfirmation
    return TendonConfirmation(checkpoint['phase'][2],
        int(checkpoint['motion_direction'][2]*np.sign(runtime.tendon_mm_per_shaft_rad)),
        int(checkpoint['confirmation_count'][2]))


def confirm_at_source(runtime, owner, history, timestamp_ns, estimator, points, *, control_owned):
    """Compose one existing policy correction with the model's replay suffix."""
    model = runtime.clone_state(include_history=False).adaptive_jacobian
    snapshot, history = owner.observe_at_timestamp(history, timestamp_ns,
        estimator.poses(estimator.mean[None])[0], model, control_owned=control_owned,
        distal_strain=estimator.mean[:24],
        distal_bending_mode=runtime.mechanics.mode.detach().cpu().numpy(),
        observed_interface_position_m=points[0], observed_marker_points_m=points)
    decisions = []
    for stamp in [estimator.timestamp_s]+[event['time'] for event in estimator.events
                                          if event['time'] > estimator.timestamp_s]:
        checkpoint = next(entry[2] for entry in reversed(history) if entry[0]*1e-9 <= stamp)
        decisions.append((stamp, confirmation_from_checkpoint(runtime, checkpoint)))
    return snapshot, history, decisions


def reconcile_engagement(runtime, interface_snapshot, *, backlash_config):
    """Preserve shared phases, widths, anchors, evidence and failure bounds.

    Effective-coordinate uncertainty is the sole model-specific derived view.
    Do not replace the baseline policy with the standalone research detector.
    """
    state = runtime.estimator
    if state is None:
        raise ValueError('joint runtime must be initialized')
    if not state.external_confirmation:
        raise ValueError('control requires shared tendon confirmation')
    result = copy.deepcopy(interface_snapshot)
    gain = runtime.profile.gain(state.mean[30], state.direction or 1)
    conversion = abs(runtime.tendon_mm_per_shaft_rad)
    result.effective_motor_uncertainty_rad[2] = np.sqrt(state.covariance[30,30])/max(gain*conversion,1e-6)
    return replace(result, engaged_gain=None)
