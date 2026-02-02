uint8 CONNECTION        = 67 # "C"
uint8 MODE              = 77 # "M"
uint8 NONE              = 0
uint8 CARTESIAN_TWIST   = 1
uint8 CARTESIAN_POS     = 2
uint8 JOINT_VEL         = 3
uint8 JOINT_POS         = 4
uint8 DEBUG             = 68 # "D"
uint8 START_MOTOR       = 73 # "I"
uint8 STOP_MOTOR        = 83 # "S"
uint8 SET_ZERO          = 90 # "Z"
uint8 SET_TARGET_VEL    = 86 # "V"
uint8 LIMIT             = 76 # "L"
uint8 LIMIT_P           = 97 # "a"
uint8 LIMIT_M           = 98 # "b"
uint8 LIMIT_N           = 99 # "c"

std_msgs/Header header

uint8 predicate

# optional subject, at most field should be none-empty
string text
uint8[] state
float64[] data