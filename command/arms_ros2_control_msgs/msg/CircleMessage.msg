# circle_trajectory.msg
# 圆形轨迹消息定义

# ====== 方法1：三点法定义圆弧 ======
# 需要提供3个点：起点、中间点、终点
# 其中起点通常由当前位置确定，这里只需提供中间点和终点
geometry_msgs/Pose midpoint  # 圆弧中间点
geometry_msgs/Pose endpoint  # 圆弧终点
geometry_msgs/Pose right_midpoint  # 双臂模式下右臂圆弧中间点
geometry_msgs/Pose right_endpoint  # 双臂模式下右臂圆弧终点

# ====== 方法2：参数法定义圆弧 ======
# 直接输入圆弧几何参数
geometry_msgs/Point center         # 圆心坐标
geometry_msgs/Vector3 axis         # 旋转轴（单位向量）
float64 rotate_angle               # 旋转角度（弧度）
geometry_msgs/Point right_center   # 双臂模式下右臂圆心坐标
geometry_msgs/Vector3 right_axis   # 双臂模式下右臂旋转轴（单位向量）
float64 right_rotate_angle         # 双臂模式下右臂旋转角度（弧度）

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
bool use_three_point_method        # true:使用三点法; false:使用参数法
bool use_slerp_for_orientation     # true:球面插值; false:线性插值
bool time_mode                     # true 时间模式；false：参数约束
string frame_id                    # 坐标系id
string arm_name                    #'left'左臂，'right'右臂，'both'双臂
string ik_type                     #'BFGS','DLS','AUTO','SDK'
