# Causally reconstructed v171 state at one accepted camera timestamp.
# The header stamp is the observation/source timestamp, not callback time.

std_msgs/Header header
int64 observation_timestamp_ns
int64 state_timestamp_ns

# Upstream physical shaft counts and the effective transmitted angle supplied
# to the learned interface Jacobian. The v171 transmission/history receives
# the raw calibrated angle derived from raw_encoder_counts; with backlash
# compensation enabled these coordinates intentionally differ.
float64[3] raw_encoder_counts
float64[3] motor_angle_rad
float64[3] downstream
float64[16] interface_pose
float64[] distal_strain
float64[3] estimated_tip_m
float64[3] observed_tip_m
float64[] estimator_covariance_diagonal
float64 estimator_covariance_trace

uint64 accepted_observations
uint32 consecutive_rejections
string estimator_health
string marker_update_reason
float64 marker_rms_before_mm
float64 marker_rms_after_mm
float64 innovation_nis
float64 innovation_nis_per_dof
uint32 observable_rank

float64[18] interface_jacobian
float64[3] backlash_remaining_rad
float64[3] backlash_remaining_lower_rad
float64[3] backlash_remaining_upper_rad
float64[3] backlash_width_positive_lower_rad
float64[3] backlash_width_positive_upper_rad
float64[3] backlash_width_negative_lower_rad
float64[3] backlash_width_negative_upper_rad
float64[3] backlash_reversal_start_motor_rad
float64[3] backlash_engagement_anchor_motor_rad
float64[3] backlash_accumulated_takeup_rad
float64[3] backlash_effective_motor_rad
float64[3] backlash_effective_motor_uncertainty_rad
int64[3] backlash_last_evidence_timestamp_ns
int32[3] backlash_motion_direction
int32[3] backlash_engaged_direction
float64[3] backlash_confidence
int32[3] backlash_confirmation_count
string[] backlash_phase
string[] backlash_response_classification
float64[3] backlash_inferred_transmitted_increment_rad
float64[3] backlash_response_evidence
float64 backlash_joint_response_residual
float64 backlash_distal_bending_increment
float64 backlash_tendon_distal_response_evidence
bool backlash_tendon_distal_response_confirmed
bool adaptation_enabled
int32 adaptation_axis
string adaptation_reason
float64 adaptation_weight
float64 adaptation_update_norm
uint32 adaptation_window_frames

float64 rewind_ms
float64 correction_ms
float64 replay_ms
float64 total_update_ms
