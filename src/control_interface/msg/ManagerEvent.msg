uint8 CONNECTION        = 67 # "C"
uint8 MODE              = 77 # "M"
uint8 NONE              = 48 # "0"
uint8 CARTESIAN_TWIST   = 49 # "1"
uint8 CARTESIAN_POS     = 50 # "2"
uint8 JOINT_VEL         = 51 # "3"
uint8 JOINT_POS         = 52 # "4"
uint8 DEBUG             = 68 # "D"
uint8 START_MOTOR       = 73 # "I"
uint8 STOP_MOTOR        = 83 # "S"
uint8 RESET_FAULT       = 82 # "R"
uint8 FAULT_STATUS      = 81 # "Q"
uint8 SET_ZERO          = 90 # "Z"
uint8 SET_TARGET_VEL    = 70 # "F"
uint8 LIMIT             = 76 # "L"
uint8 STALL             = 84 # "T"
uint8 TRIGGER_M           = 97 # "a"
uint8 TRIGGER_C           = 98 # "b"
uint8 TRIGGER_P           = 99 # "c"
uint8 TRIGGER_N           = 110 # "n"

uint8 POSITION_STATUS     = 71 # position transaction event

uint8 MOTION_SUSPECTED    = 1
uint8 MOTION_CONFIRMED    = 2
uint8 MOTION_RECOVERED    = 3
uint8 MOTION_RESET        = 4
uint8 MOTION_UNOBSERVABLE = 5
uint8 MOTION_STATUS        = 6

uint8 POSITION_COMPLETE    = 1
uint8 POSITION_TIMED_OUT   = 2
uint8 POSITION_REJECTED    = 3

uint8 FAULT_STALL           = 1
uint8 FAULT_OVERSPEED       = 2
uint8 FAULT_WRONG_DIRECTION = 3
uint8 FAULT_FEEDBACK        = 4
uint8 FAULT_UNOBSERVABLE    = 5
uint8 FAULT_DRIVER_COMMUNICATION = 6
uint8 FAULT_ENCODER_INTEGRITY = 7


std_msgs/Header header

uint8 predicate

# optional subject, at most field should be none-empty
string text
uint8[] state
float64[] data
