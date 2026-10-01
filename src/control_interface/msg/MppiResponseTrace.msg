# One causal comparison between the first executable MPPI action and the
# first accepted camera observation at or after that action's prediction time.
# Length units are metres unless the field name explicitly ends in _mm.

std_msgs/Header header

int64 forecast_start_timestamp_ns
int64 start_observation_timestamp_ns
int64 forecast_due_timestamp_ns
int64 observation_timestamp_ns

float64[6] command_logical_velocity
float64[6] motor_radians_per_second
float64[6] joint_position
float64[3] model_motor_angle_rad

# Row-major 4x4 base-to-interface material pose and row-major 6x3 body
# interface Jacobian used by the rollout that produced this forecast.
float64[16] interface_pose
float64[18] interface_jacobian

float64[3] target_tip_m
float64[3] start_tip_m
float64[3] predicted_tip_m
float64[3] observed_tip_m

float64[3] predicted_delta_mm
float64[3] measured_delta_mm
float64[3] endpoint_error_mm
float64 endpoint_error_norm_mm
bool direction_cosine_valid
float64 direction_cosine
