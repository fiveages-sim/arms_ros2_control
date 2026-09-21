# Live COMPLIANCE force status for RViz monitoring / tuning.
# Axes order: Fx Fy Fz Mx My Mz  (base / teleop frame)
# task_selection: 1 = force axis, 0 = position axis

std_msgs/Header header

float64[6] task_selection
float64[6] force_setpoint
# Accepted target magnitudes; exceeding either limit rejects the parameter update.
float64 force_setpoint_limit
float64 torque_setpoint_limit
float64[6] force_measured_left
float64[6] force_measured_right

bool left_ft_active
bool right_ft_active
bool zero_cal_done
float64 force_feedback_sign

# Surface alignment (曲面贴合): torque-driven target-orientation bias.
# The loop turns torque error into a bounded axis-angle bias that rotates the
# TARGET pose around the estimated contact point; position axes (S=0) track it.
# Requires rx/ry/rz to be position axes, otherwise the bias is not tracked.
# Bias is expressed in header.frame_id; |bias| [rad] is the current
# compliant re-orientation angle the environment is asking for.
bool align_enabled
float64 align_max
float64 align_release
bool align_rcc
float64[3] align_bias_left
float64[3] align_bias_right

# Phase-identification telemetry. mode: 0=idle, 1=joint position loop,
# 2=contact chain (actual displacement -> measured wrench).
uint8 identification_mode
bool identification_active
string identification_arm
uint8 identification_axis
# Arm-local joint index used by mode 1.
uint8 identification_joint
uint8 left_joint_count
uint8 right_joint_count
# Controller kinematics groups used for payload standstill checks.
string[] left_joint_names
string[] right_joint_names
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
