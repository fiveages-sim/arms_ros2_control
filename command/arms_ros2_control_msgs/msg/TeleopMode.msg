# control mode
uint8 CONTROL_ARM=0
uint8 CONTROL_CHASSIS=1

builtin_interfaces/Time stamp

# true when teleop commands are active (joystick must enable; keyboard always true)
bool active
uint8 control_mode
bool mirror_movement
int32 speed_level
float32 speed_scale
