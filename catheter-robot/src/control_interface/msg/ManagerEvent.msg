uint8 CONNECTION        = 67 # "C"
uint8 MODE              = 77 # "M"
uint8 NONE              = 0
uint8 CARTESIAN_TWIST   = 1
uint8 CARTESIAN_POS     = 2
uint8 JOINT_VEL         = 3
uint8 JOINT_POS         = 4
uint8 DEBUG             = 5
uint8 START_MOTOR       = 73 # "I"
uint8 STOP_MOTOR        = 83 # "S"
uint8 SET_ZERO          = 90 # "Z"
uint8 SET_TARGET_VEL    = 86 # "V"
uint8 LIMIT             = 76 # "L"

std_msgs/Header header

uint8 predicate
uint8[] state