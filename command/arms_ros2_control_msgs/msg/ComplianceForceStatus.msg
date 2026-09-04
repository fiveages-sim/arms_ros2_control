# Live COMPLIANCE force status for RViz monitoring / tuning.
# Axes order: Fx Fy Fz Mx My Mz  (base / teleop frame)
# task_selection: 1 = force axis, 0 = position axis

std_msgs/Header header

float64[6] task_selection
float64[6] force_setpoint
# Effective runtime limits after the controller's hard safety clamp.
float64 force_setpoint_limit
float64 torque_setpoint_limit
float64[6] force_measured_left
float64[6] force_measured_right

bool left_ft_active
bool right_ft_active
# Per-arm readiness; zero_cal_done is kept for backward-compatible summary UI.
bool left_tare_valid
bool right_tare_valid
bool left_force_ready
bool right_force_ready
bool zero_cal_done
float64 force_feedback_sign

# Phase-identification telemetry. mode: 0=idle, 1=joint position loop,
# 2=contact chain (actual displacement -> measured wrench).
uint8 identification_mode
bool identification_active
float64 identification_progress
string identification_arm
uint8 identification_axis
# Arm-local joint index used by mode 1.
uint8 identification_joint
uint8 left_joint_count
uint8 right_joint_count
float64 identification_frequency
float64 identification_input
float64 identification_output
uint32 identification_result_sequence
uint8 identification_result_mode
string identification_result_arm
uint8 identification_result_joint
float64 identification_result_frequency
float64 identification_gain
float64 identification_phase_deg
bool identification_result_valid
string identification_message
