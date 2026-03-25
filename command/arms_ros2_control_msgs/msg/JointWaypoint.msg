# TrajectoryWaypoint.msg
# 关节路点信息

#目标关节位置（movej）或关节速度(speedj)
float64[] position          # 关节位置
float64[] velocity         # 关节速度
float64 blend_ratio_percent    # 转接比例百分比（0-1），用于多点轨迹的平滑转接

#规划的参数
float64[] max_velocity
float64[] max_acceleration
float64[] max_jerk

bool time_mode
float64 total_time
