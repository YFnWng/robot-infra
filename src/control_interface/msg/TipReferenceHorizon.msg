# Timestamped Cartesian reference preview consumed by catheter MPPI.
# positions[0] is represented by header.stamp and subsequent samples are
# separated by sample_period_s. All positions/tangents are in header.frame_id.
std_msgs/Header header
string path_id
uint64 sequence
float64 sample_period_s
geometry_msgs/Point[] positions
geometry_msgs/Vector3[] tangents
float64 progress_m
float64 total_length_m
float64 nominal_speed_m_s
bool final_hold
bool progress_paused
float64 expiry_s
