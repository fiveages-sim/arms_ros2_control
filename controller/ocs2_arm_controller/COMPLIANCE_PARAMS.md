# COMPLIANCE 模式参数说明

进入方式：HOLD 状态下发送 FSM command=5（RViz 面板 COMPLIANCE 按钮）。

---

## 混合控制选择

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_task_selection` | `[1,0,0,0,0,0]` | 6 轴选择矩阵，`1`=力控轴，`0`=位控轴。轴序：`[Fx, Fy, Fz, Mx, My, Mz]` |

## 位置控制（S=0 的轴）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_hybrid_pos_stiffness` | `[20,20,20,10,10,10]` | 比例增益 K [1/s]，前 3 个为平移，后 3 个为旋转。v = K·(target−current)。稳定要求：K·dt ≤ 0.3（100Hz 时 K≤30，60Hz 时 K≤18） |
| `compliance_hybrid_cart_vmax` | `[0.15,0.15,0.15,0.6,0.6,0.6]` | 笛卡尔速度上限 [m/s, rad/s]，限制 TCP 运动速度 |

## 力控制（S=1 的轴）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_hybrid_force_damping` | `[5000,5000,5000,250,250,250]` | 力控阻尼 D [N·s/m, Nm·s/rad]。v = F_err / D，越大越柔/越稳 |
| `compliance_hybrid_force_ki` | `2.0` | 力控积分增益 [1/s]，消除质量/质心不准引起的重力残余。I_err = ∫ Ki·F_err |
| `compliance_hybrid_force_ki_max` | `10.0` | 积分项上限 [N, Nm]，防止 windup |
| `compliance_force_setpoint` | `[0,0,0,0,0,0]` | 力控目标值 [N, Nm]，只在 S=1 轴上生效。自由空间设为 0，接触时设目标力 |
| `compliance_force_feedback_sign` | `-1.0` | 力控方向符号：−1 表示实测力与目标力方向相反时减速。如果力方向反了，改为 1 |
| `compliance_hybrid_force_xmax_lin` | `0.2` | 力控轴平移位移上限 [m]，触限后刹车，防止奇异/重力模型不准导致的无限漂移 |
| `compliance_hybrid_force_xmax_ang` | `0.3` | 力控轴旋转位移上限 [rad] |

## 关节空间限幅

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_hybrid_joint_vmax` | `0.8` | 关节速度上限 [rad/s] |
| `compliance_hybrid_joint_limit_margin` | `0.02` | 关节限位安全距离 [rad]，触限时速度清零 |
| `compliance_hybrid_dls_lambda` | `0.05` | DLS 伪逆正则化参数，越大越平滑但越偏离精确解 |

## 力信号处理链

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_wrench_lpf_alpha` | `0.15` | EMA 滤波系数，越大响应越快但噪声越大 |
| `compliance_zero_cal_duration` | `10.0` | 零力校准时长 [s]，进入 COMPLIANCE 后需保持静置 |
| `compliance_zero_cal_settle` | `0.2` | 校准开始前的静置等待 [s] |
| `compliance_zero_cal_still_vel` | `0.02` | 静止判断阈值 [rad/s]，关节速度低于此值才采校准样本 |
| `compliance_gravity_accel` | `9.81` | 重力加速度 [m/s²] |
| `compliance_ft_timeout` | `0.2` | FT 传感器超时 [s]，超过此时间无数据则停用力控 |
| `left_dyn_param` | `[]` | 左臂工具动力学 `[mass_kg, com_x_mm, com_y_mm, com_z_mm]`，不设则重力补偿关闭 |
| `right_dyn_param` | `[]` | 右臂工具动力学 |

## 遥操作

| 参数 | 默认值 | 说明 |
|---|---|---|
| `compliance_teleop_enable` | `true` | 是否允许通过 topic（left_target/left_target/stamped）设置目标位姿 |
| `compliance_teleop_base_frame` | `base_link` | teleop target 的参考坐标系 |

---

## 常用在线调试命令

```bash
# 查看所有参数
ros2 param list /ocs2_arm_controller | grep compliance

# 全部轴力控（浮动模式，力目标全 0）
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,1,1,1,1,1]"

# X 轴输出 5N 力
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5,0,0,0,0,0]"

# 力控更软（D 翻倍）
ros2 param set /ocs2_arm_controller compliance_hybrid_force_damping "[10000,10000,10000,500,500,500]"

# 位控更快（K 从 20 提到 30，100Hz 下仍安全）
ros2 param set /ocs2_arm_controller compliance_hybrid_pos_stiffness "[30,30,30,15,15,15]"

# 调整零力校准时长
ros2 param set /ocs2_arm_controller compliance_zero_cal_duration 5.0
```
