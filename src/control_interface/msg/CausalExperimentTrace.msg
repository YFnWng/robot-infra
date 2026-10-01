# Per-command causal-identification trace. Joint vectors use the firmware
# logical order and units: [catheter_lin_mm, catheter_rot_deg,
# catheter_bend_mm, sheath_lin_mm, sheath_rot_deg, sheath_bend_deg].

std_msgs/Header header

uint32 episode_index
string episode_name
string excitation_basis
string speed_tier
uint32 repetition
float64 experiment_elapsed_s
float64 episode_elapsed_s
int64 intended_command_timestamp_ns
int64 episode_start_timestamp_ns
bool[6] command_axis_mask
float64[6] episode_start_logical_position
float64[6] episode_start_raw_encoder_counts

float64[6] requested_logical_velocity
float64[6] commanded_logical_velocity
float64[6] requested_motor_axis_velocity
int32[6] predicted_motor_rpm
float64[6] predicted_motor_radians_per_second
float64[6] predicted_realized_logical_velocity
float64[6] reference_logical_position
float64[6] measured_logical_position
int64 measured_position_receipt_timestamp_ns

float64[6] raw_encoder_counts
int64 raw_encoder_receipt_timestamp_ns
