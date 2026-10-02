# Versioned result returned to the non-commanding C++ shell.
# All steady timestamps use CLOCK_MONOTONIC on the same host.
uint32 CURRENT_SCHEMA_VERSION = 1

std_msgs/Header header
uint32 schema_version
uint64 shell_epoch
uint64 request_sequence
uint64 device_sequence
uint64 marker_sequence
uint64 manager_sequence
uint64 target_revision
int64 estimator_state_timestamp_ns
int64 computation_start_steady_ns
int64 computation_end_steady_ns
bool valid
string controller_state
string reason
float64[6] logical_velocity
bool projection_applied
string estimator_health
string planner_reason
