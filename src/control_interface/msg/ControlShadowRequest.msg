# Versioned immutable watermark emitted by the non-commanding C++ shell.
# Header stamp is ROS publication time; request_steady_ns is CLOCK_MONOTONIC.
uint32 CURRENT_SCHEMA_VERSION = 1

std_msgs/Header header
uint32 schema_version
uint64 shell_epoch
uint64 request_sequence
int64 request_steady_ns
uint64 device_sequence
uint64 marker_sequence
uint64 manager_sequence
uint64 target_revision
int64 device_source_stamp_ns
int64 marker_source_stamp_ns
int64 manager_source_stamp_ns
bool device_present
bool marker_present
bool manager_present
bool target_present
string controller_mode
string configuration_identity
