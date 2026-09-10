# base
uint8 BASE_LOCKED=0
uint8 BASE_UNLOCKED=1

# body
uint8 BODY_FREE=0
uint8 BODY_VERTICAL=1
uint8 BODY_TRACKING=2
uint8 BODY_LOCKED=3
uint8 BODY_HEAD_COUPLED=4
uint8 BODY_CUSTOM_LOCKED=5

# bimanual
uint8 BIMANUAL_INDEPENDENT=0
uint8 BIMANUAL_COUPLED=1

# arm
uint8 ARM_DISABLED=0
uint8 ARM_ENABLED=1

uint8 base_state
# Selected body mode; temporarily overridden by the legacy head/waist constraints in HEAD_FORWARD.
uint8 body_state
uint8 bimanual_state
uint8 left_arm_state
uint8 right_arm_state
bool home_joint_reference_enabled

# Head task. HEAD_ENABLED remains an alias for marker/VR tracking consumers.
uint8 HEAD_DISABLED=0
uint8 HEAD_ENABLED=1
uint8 HEAD_TRACKING=1
uint8 HEAD_GAZE=2
uint8 HEAD_FORWARD=3
uint8 head_state

# Task activation, not a guarantee of alignment or reachability. Independent of body_state.
bool head_midpoint_gaze_active
