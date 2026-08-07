uint8 MODE_DISABLED=1
uint8 MODE_ACTIVE=2
uint8 MODE_PASSIVE=3
uint8 MODE_GRAVITY_COMP=4
uint8 MODE_FAULT=100
uint8 MODE_CALIBRATING=101

builtin_interfaces/Time stamp
uint8 mode
bool session_owned
bool profile_valid
bool calibrated
bool all_motors_online
bool feedback_fresh
bool fault_latched
uint32 fault_code
string fault_reason
float32 command_age_s
float32 feedback_age_s
MotorIdentity[] motors

