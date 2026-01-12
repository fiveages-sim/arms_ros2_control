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

# hand/gripper command: 0=close, 1=open (applies to the arm specified by target)
int32 hand_command