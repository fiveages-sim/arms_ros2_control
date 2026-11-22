# VR input semantic latch (topic teleop/vr_mode).
# QoS: Reliable + TransientLocal; publish once at start, then on field change.
# WBC constraint state stays on ocs2_wbc_controller/current_state — not duplicated here.

# control topology (ATM detects active controller)
uint8 TOPOLOGY_UNKNOWN=0
uint8 TOPOLOGY_SPLIT_BODY=1
uint8 TOPOLOGY_FULL_BODY=2

# stick / body-thumbstick plane
uint8 PLANE_XY=0
uint8 PLANE_Z_YAW=1

# mirror processing profile (only meaningful when mirror_movement is true)
uint8 MIRROR_PROFILE_A=0
uint8 MIRROR_PROFILE_B=1

builtin_interfaces/Time stamp

# VR handler enabled (enable_vr + runtime enable/disable)
bool enabled

# true = UPDATE (follow VR), false = STORAGE
bool update_mode

# true = chassis teleop (case 20), false = end-effector stick control
bool chassis_mode

# FULL_BODY Y/B semantics: false = WBC arm enable/disable, true = VR pause/resume
bool yb_pause_semantics

# Per-arm VR pause (pause semantics / SPLIT_BODY). Independent of WBC arm_state.
bool left_arm_paused
bool right_arm_paused

# Stick planes: PLANE_XY or PLANE_Z_YAW
uint8 left_stick_plane
uint8 right_stick_plane

# Body marker stick plane while BODY_TRACKING (PLANE_XY or PLANE_Z_YAW)
uint8 body_stick_plane

# Gripper trigger semantics: true = percent, false = open/close switch
bool left_gripper_percent_mode
bool right_gripper_percent_mode

bool mirror_movement
uint8 mirror_profile

uint8 topology
