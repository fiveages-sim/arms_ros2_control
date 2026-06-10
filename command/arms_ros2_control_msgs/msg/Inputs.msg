# control input message

# position
float32 x
float32 y
float32 z

# rotation
float32 roll
float32 pitch
float32 yaw

# command target, left(1) or right arm(2)
int32 target

# hand/gripper: nan=no command; 0=close / 1=open (target_command); else 0~1 open ratio (target_percent)
float32 hand_command
