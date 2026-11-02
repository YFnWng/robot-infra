# Raw physical encoder endpoints; never a request to change encoder zero.
std_msgs/Header header
uint32 session_id
uint32 command_id
uint32 duration_us
# Absolute ROS deadline assigned once at plan commitment, unchanged by heartbeats.
int64 deadline_ros_ns
uint8 active_mask
int32[6] target_counts
uint16[6] speed_rpm
