# control topology (ATM detects active controller)
uint8 TOPOLOGY_UNKNOWN=0
uint8 TOPOLOGY_SPLIT_BODY=1
uint8 TOPOLOGY_FULL_BODY=2

# stick / body-thumbstick plane (0=default, 1=alt for that stick)
# Arm sticks: PLANE_XY / PLANE_Z_YAW
# Body left  (body_stick_plane):       0=XY, 1=X_ROLL (forward + roll)
# Body right (body_right_stick_plane): 0=Z_YAW (lift + yaw), 1=PITCH_YAW
uint8 PLANE_XY=0
uint8 PLANE_Z_YAW=1

# mirror processing profile (only meaningful when mirror_movement is true)
uint8 MIRROR_PROFILE_A=0
uint8 MIRROR_PROFILE_B=1

builtin_interfaces/Time stamp

bool enabled
bool update_mode
bool chassis_mode
bool yb_pause_semantics
bool left_arm_paused
bool right_arm_paused
uint8 left_stick_plane
uint8 right_stick_plane
uint8 body_stick_plane
uint8 body_right_stick_plane
bool left_gripper_percent_mode
bool right_gripper_percent_mode
bool mirror_movement
uint8 mirror_profile
uint8 topology
