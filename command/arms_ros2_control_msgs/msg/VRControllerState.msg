# VRControllerState.msg
# VR控制器状态消息（统一封装所有按钮和轴值）
# 所有按钮字段表示触发事件（上升沿检测后），而非持续状态

# 左控制器
bool left_thumbstick_button      # 左摇杆按钮（按下事件）
bool left_grip_button            # 左握把按钮（按下事件）
bool left_y_button               # 左Y按钮（按下事件）
float64 left_thumbstick_x        # 左摇杆X轴值 (-1.0 ~ 1.0)
float64 left_thumbstick_y        # 左摇杆Y轴值 (-1.0 ~ 1.0)

# 右控制器
bool right_thumbstick_button     # 右摇杆按钮（按下事件）
bool right_grip_button           # 右握把按钮（按下事件）
bool right_b_button              # 右B按钮（按下事件）
float64 right_thumbstick_x       # 右摇杆X轴值 (-1.0 ~ 1.0)
float64 right_thumbstick_y       # 右摇杆Y轴值 (-1.0 ~ 1.0)
