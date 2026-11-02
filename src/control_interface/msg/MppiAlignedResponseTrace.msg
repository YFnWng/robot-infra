# Explicit aligned-response metadata. The nested legacy layout is unchanged
# so historical MppiResponseTrace bags remain readable.
std_msgs/Header header
control_interface/MppiResponseTrace comparison
int64 source_observation_timestamp_ns
int64 source_start_observation_timestamp_ns
bool comparison_valid
string comparison_reason
# Validity covers published intent, not motor-applied acknowledgement.
float64[3] model_start_tip_m
float64[3] root_readout_error_mm
# Nested predicted delta excludes root offset; endpoint error includes it.
