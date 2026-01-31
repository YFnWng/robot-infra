# ============================================================
# TeleopIntent
#
# High-level operator intent for teleoperation.
# Exactly ONE of the intent sections below should be populated
# per message. The teleop manager is responsible for enforcing
# exclusivity and safety.
# ============================================================

std_msgs/Header header

# ------------------------------------------------------------------
# Source metadata
# ------------------------------------------------------------------
string source           # "keyboard", "slicer", "autonomy"
# string operator_id      # optional (empty if not applicable)

# ------------------------------------------------------------------
# Operator state (safety-critical)
# ------------------------------------------------------------------
# bool enable             # dead-man / allow motion
# bool clutch             # temporarily decouple motion
# bool emergency_stop     # latched upstream, immediate stop request

# ------------------------------------------------------------------
# CARTESIAN INTENT (used when control_mode == CARTESIAN_*)
# ------------------------------------------------------------------

# Reference frame for Cartesian commands
# Must be resolvable via TF (e.g. "world", "image", "tool")
string cartesian_frame

# Incremental motion (delta) in cartesian_frame
geometry_msgs/Twist cartesian_delta
# - linear: meters
# - angular: radians

# Absolute target pose in cartesian_frame
geometry_msgs/Pose cartesian_target

# ------------------------------------------------------------------
# JOINT / ACTUATOR INTENT (used when control_mode == JOINT_*)
# ------------------------------------------------------------------

# joint names
# string[] joint_name

# joint velocity commands
float64[] joint_vel
# Units defined by robot (rad, m, or normalized)

# Absolute joint targets
float64[] joint_pos

# ------------------------------------------------------------------
# AUTONOMY INTENT (used when control_mode == AUTO)
# ------------------------------------------------------------------

# TODO

# ------------------------------------------------------------------
# Optional scaling hints (non-authoritative)
# Teleop manager may ignore or clamp these
# ------------------------------------------------------------------
# float64 velocity_scale     # [0.0, 1.0]
# float64 acceleration_scale # [0.0, 1.0]
