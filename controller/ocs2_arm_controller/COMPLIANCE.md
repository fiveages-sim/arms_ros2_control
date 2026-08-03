# COMPLIANCE 力位混合控制

OCS2 控制器 **COMPLIANCE** 状态：6 自由度笛卡尔混合控制，每位可独立选 **位控 (S=0)** 或 **力控 (S=1)**。

```
位控：v = K · (target − current)           → DLS 映射到关节
力控：v = (F_des − F_meas + I_err) / D   → DLS 映射到关节
```

**进入**：HOLD 下 `fsm_command=5`（RViz **COMPLIANCE** 按钮）  
**退出**：`fsm_command=2`（**HOLD** 按钮）

---

## 快速上手

### 1. 进入并零力校准

```bash
ros2 topic pub /fsm_command std_msgs/msg/Int32 "data: 5" -1
```

手臂静止约 10 s（`compliance_zero_cal_duration`），日志出现 `zero_cal done` 后力控可用。  
状态可通过 `/compliance_force_status` 的 `zero_cal_done` 或 Panel 状态栏确认。

### 2. 发送位姿目标（S=0 轴）

- **RViz**：拖拽交互 marker
- **Topic**：
  ```bash
  ros2 topic pub /left_target geometry_msgs/msg/Pose \
    '{position: {x: 0.5, y: 0.3, z: 0.8}, orientation: {x: 0, y: 0, z: 0, w: 1}}' -1
  ```
  右臂：`/right_target`；带坐标系：`/left_target/stamped`

### 3. 设置力控轴与目标力（S=1 轴）

```bash
# X 轴力控 5 N，其余位控
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,0,0,0,0,0]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5,0,0,0,0,0]"
```

或在 RViz **ComplianceForcePanel** 编辑后点 **应用设定**。参数每控制周期重读，**即时生效**。

### 4. 退出

```bash
ros2 topic pub /fsm_command std_msgs/msg/Int32 "data: 2" -1
```

---

## RViz ComplianceForcePanel

预置于 `demo_ocs2.rviz` / `splitbody.rviz` / `fullbody.rviz` / `demo.rviz`。  
未显示：**Panels → Add Panel → ComplianceForcePanel**。

| 列 | 含义 | 写 | 读 |
|----|------|----|----|
| 力控 | S=1 力控 / S=0 位控 | `compliance_task_selection` | `task_selection[i]` |
| F_des | 目标力 [N/Nm] | `compliance_force_setpoint` | `force_setpoint[i]` |
| F_meas L/R | 左/右臂实测力 | — | `force_measured_left/right[i]` |
| err L | F_des − F_meas（左） | — | Panel 本地计算 |

轴序：`[Fx, Fy, Fz, Mx, My, Mz]`，坐标系 `header.frame_id`（默认 `base_link`）。

OCS2FSMPanel 在 COMPLIANCE 下还提供 **软/中/硬** 刚度预设（写 `compliance_hybrid_*` 参数）。

---

## 接口

### 写入（动态参数）

节点：`/ocs2_arm_controller`

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `compliance_task_selection` | `float64[6]` | `[1,0,0,0,0,0]` | 1=力控，0=位控 |
| `compliance_force_setpoint` | `float64[6]` | 全 0 | 目标力，仅 S=1 轴生效 |

```bash
ros2 param get /ocs2_arm_controller compliance_task_selection
ros2 param list /ocs2_arm_controller | grep compliance
```

### 读取（话题）

| 话题 | 类型 | 频率 |
|------|------|------|
| `/compliance_force_status` | `arms_ros2_control_msgs/ComplianceForceStatus` | ~20 Hz（仅 COMPLIANCE） |

```yaml
std_msgs/Header header
float64[6] task_selection
float64[6] force_setpoint
float64[6] force_measured_left    # 已含 force_feedback_sign
float64[6] force_measured_right
bool left_ft_active / right_ft_active
bool zero_cal_done
float64 force_feedback_sign       # 默认 -1.0
```

```bash
ros2 topic echo /compliance_force_status
ros2 interface show arms_ros2_control_msgs/msg/ComplianceForceStatus
```

### FT 原始输入

| 侧 | 话题 |
|----|------|
| 左 | `/left_ft_broadcaster/wrench` |
| 右 | `/right_ft_broadcaster/wrench` |

`config/robot.local.yaml` 中 `hardware.left_ft` / `right_ft` 为 `none` 时，该侧 broadcaster 不启动。

---

## 参数表

### 混合选择与位控（S=0）

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_task_selection` | `[1,0,0,0,0,0]` | 6 轴 S 矩阵 |
| `compliance_hybrid_pos_stiffness` | `[20,20,20,10,10,10]` | 位控 K [1/s]；稳定：K·dt ≤ 0.3 |
| `compliance_hybrid_pos_damping` | `[0,0,0,0,0,0]` | 位控阻尼 |
| `compliance_hybrid_cart_vmax` | `[0.15,…,0.6]` | 笛卡尔速度上限 [m/s, rad/s] |

### 力控（S=1）

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_force_setpoint` | 全 0 | 目标力 [N, Nm] |
| `compliance_hybrid_force_damping` | `[5000,…,250]` | 力控 D；越大越柔 |
| `compliance_hybrid_force_ki` | `2.0` | 积分增益，消重力残余 |
| `compliance_hybrid_force_ki_max` | `10.0` | 积分上限 |
| `compliance_hybrid_force_ki_leak` | `0.5` | 积分泄漏 [1/s]；拖拽振荡时调大（0.5–1.5） |
| `compliance_hybrid_force_deadband` | `0.5` | 力误差死区 |
| `compliance_force_feedback_sign` | `-1.0` | 力方向反了改为 `1.0` |
| `compliance_force_vel_lpf_alpha` | `0.3` | 导纳输出低通（虚拟惯性）；振荡调小（0.1–0.3），迟钝调大 |
| `compliance_hybrid_force_xmax_lin` | `0.2` | 力控平移软限 [m] |
| `compliance_hybrid_force_xmax_ang` | `0.3` | 力控旋转软限 [rad] |
| `compliance_hybrid_force_xmax_margin_ratio` | `0.2` | 软限渐缓区占 xmax 比例 |

### 关节限幅与求解

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_hybrid_joint_vmax` | `0.8` | 关节速度上限 [rad/s] |
| `compliance_hybrid_joint_limit_margin` | `0.02` | 关节限位裕度 [rad] |
| `compliance_hybrid_dls_lambda` | `0.05` | DLS 正则化 |

### 力信号与校准

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_wrench_lpf_alpha` | `0.15` | EMA 滤波 |
| `compliance_zero_cal_duration` | `10.0` | 零力校准时长 [s] |
| `compliance_zero_cal_settle` | `0.2` | 校准前等待 [s] |
| `compliance_zero_cal_still_vel` | `0.02` | 静止阈值 [rad/s] |
| `compliance_gravity_accel` | `9.81` | 重力加速度 |
| `compliance_ft_timeout_sec` | `0.2` | FT 超时 [s] |
| `left_dyn_param` / `right_dyn_param` | `[]` | 工具动力学 `[mass_kg, com_x/y/z_mm]` |

### 遥操作

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_teleop_enable` | `true` | 允许 target topic |
| `compliance_teleop_base_frame` | `base_link` | 力/位姿参考系 |
| `compliance_gravity_frame` | `world` | 重力补偿参考系 |

---

## 常用场景

**纯位姿跟踪**（默认）：S 矩阵保持默认，拖拽 marker；力控轴维持 F_des=0。

**全轴浮动（导纳）**：
```bash
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,1,1,1,1,1]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[0,0,0,0,0,0]"
```

**接触力控（X 向 5 N，其余位控）**：
```bash
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,0,0,0,0,0]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5,0,0,0,0,0]"
```

**在线调参示例**：
```bash
ros2 param set /ocs2_arm_controller compliance_hybrid_pos_stiffness "[30,30,30,15,15,15]"
ros2 param set /ocs2_arm_controller compliance_hybrid_force_damping "[10000,10000,10000,500,500,500]"
ros2 param set /ocs2_arm_controller compliance_hybrid_force_xmax_lin 0.5
ros2 param set /ocs2_arm_controller compliance_zero_cal_duration 5.0
```

---

## 诊断

| 日志 / 状态 | 含义 |
|-------------|------|
| `zero_cal=done FT(L=on)` | 校准完成，力控可用 |
| `zero_cal=running` | 校准中，保持静止 |
| `zero_cal=waiting_FT` | 等待 FT（超时后跳过） |
| `force_ctl_L: … v=0.0055` | 力控正常 |
| `disp_lin=[20 0 0]mm` | 力控位移触软限 |
| `ft_ok=0` | FT 掉线或未完成校准 |

| 现象 | 处理 |
|------|------|
| Panel「等待 COMPLIANCE…」 | 切到 COMPLIANCE 状态 |
| 无 status 话题 | `ros2 topic list \| grep compliance` |
| F_meas 全 0 | 查 `robot.local.yaml` FT 配置与 `ros2 control list_controllers \| grep ft` |
| 力方向反 | `compliance_force_feedback_sign` 改为 `1.0` |
| 拖拽时低频振荡（1–3 Hz） | `compliance_force_vel_lpf_alpha` 调小至 0.1–0.2；`compliance_hybrid_force_ki_leak` 调大至 0.8–1.5；必要时增大 `compliance_hybrid_force_damping` |

---

## 编译

```bash
colcon build --packages-select arms_ros2_control_msgs arms_rviz_control_plugin ocs2_arm_controller --symlink-install
source install/setup.bash
```
