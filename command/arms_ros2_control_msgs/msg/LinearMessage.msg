# linear_trajectory.msg
# 直线轨迹消息定义

# ====== 直线终点 ======
geometry_msgs/Pose endpoint  # 直线终点
geometry_msgs/Pose right_endpoint  # 双臂模式下右臂直线终点

# ====== 运动参数 ======
# 线性运动约束（单位：m/s, m/s², m/s³）
float64 max_linear_velocity
float64 max_linear_acceleration
float64 max_linear_jerk

# 角运动约束（单位：rad/s, rad/s², rad/s³）
float64 max_angular_velocity
float64 max_angular_acceleration
float64 max_angular_jerk

float64 duration                   # 时间模式定义运动时间

# ====== 配置选项 ======
bool time_mode                     # true 时间模式；false：参数约束
string frame_id                    # 坐标系id
string arm_name                    #要发哪个手，"left"、"right"或者"both"
string ik_type                     #'BFGS','DLS','AUTO','SDK'
