# Live COMPLIANCE force status for RViz monitoring / tuning.
# Axes order: Fx Fy Fz Mx My Mz  (base / teleop frame)
# task_selection: 1 = force axis, 0 = position axis

std_msgs/Header header

float64[6] task_selection
float64[6] force_setpoint
float64[6] force_measured_left
float64[6] force_measured_right

bool left_ft_active
bool right_ft_active
bool zero_cal_done
float64 force_feedback_sign
