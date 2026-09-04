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

| 侧 | 原始输入 | 重力补偿 + 零偏清除输出 |
|----|----------|--------------------------|
| 左 | `/left_ft_broadcaster/wrench` | `/left_ft_broadcaster/wrench_filtered` |
| 右 | `/right_ft_broadcaster/wrench` | `/right_ft_broadcaster/wrench_filtered` |

`wrench_filtered` 保持传感器原始 `frame_id`，数值为：

```text
原始传感器读数 - 工具重力力/力矩 - 静止清零残余偏置
```

控制器激活期间在所有 FSM 状态中持续发布；进入 COMPLIANCE 前尚未完成静止清零时，
零偏项为 0。若 broadcaster 自身配置了 filter chain，不要再让它发布同名
`wrench_filtered`，避免同一话题出现两个数据源。

### 手动清零

进入 COMPLIANCE 后，可在 RViz Compliance Force Panel 点击「传感器清零」，或调用：

```bash
ros2 service call /compliance_zero_wrench std_srvs/srv/Trigger '{}'
```

清零使用 `compliance_zero_cal_duration` 配置的静止采样时间。采样期间保持机械臂静止、末端无接触；
力控轴在新零偏标定完成前保持禁用。

`config/robot.local.yaml` 中 `hardware.left_ft` / `right_ft` 为 `none` 时，该侧 broadcaster 不启动。

---

## 参数表

### 混合选择与位控（S=0）

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_task_selection` | `[1,0,0,0,0,0]` | 6 轴 S 矩阵 |
| `compliance_hybrid_pos_stiffness` | `[20,20,20,10,10,10]` | 位控 K [1/s]；稳定：K·dt ≤ 0.3 |
| `compliance_hybrid_pos_damping` | `[0,0,0,0,0,0]` | 位控阻尼 |
| `compliance_hybrid_pos_vel_damping` | `0.0` | 基于上周期指令速度的实验性阻尼；指令速度不等于实测速度，默认关闭 |
| `compliance_hybrid_pos_accel_ramp` | `0.0` | 位控轴速度斜坡时间 [s]；平移/旋转组分别共享一个缩放量以保持方向，可在机器人 `compliance.yaml` 中启用 |
| `compliance_hybrid_pos_jerk_tau` | `0.0` | 拐角圆滑时间常数 τ [s]：jerk 上界 ≈ 2·vmax/(T·τ)，与加速度（T）解耦可调；0 = 自动取 T/3 |
| `compliance_hybrid_cart_vmax` | `[0.15,…,0.6]` | 笛卡尔速度上限 [m/s, rad/s] |

### 力控（S=1）

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_force_setpoint` | 全 0 | 目标力 [N, Nm] |
| `compliance_hybrid_force_damping` | `[5000,…,250]` | 力控 D；越大越柔 |
| `compliance_hybrid_force_ki` | `1.0` | 积分增益，消重力残余 |
| `compliance_hybrid_force_ki_max` | `5.0` | 积分上限 |
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
| `compliance_wrist_coupling_max` | `1.9199` | 6/7 轴耦合上限：\|q6\|+\|q7\| ≤ max [rad]（110°，说明书图 4-4 八边形可行域的斜边；直边已由 URDF 限位覆盖）。≤0 关闭 |
| `compliance_hybrid_dls_lambda` | `0.05` | DLS 正则化；QP 分层逆解的二级旋转也使用此阻尼，避免一级 QP 阻尼压低姿态校正 |

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
| `compliance_teleop_base_frame` | `.info` 的 `model_information.baseFrame`（随机型/模式：base_footprint / arm_base / …） | 力/位姿参考系（勿在 yaml 硬编码覆盖） |
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
| `ft_ok=0` | FT 掉线或未完成校准 |

| 现象 | 处理 |
|------|------|
| Panel「等待 COMPLIANCE…」 | 切到 COMPLIANCE 状态 |
| 无 status 话题 | `ros2 topic list \| grep compliance` |
| F_meas 全 0 | 查 `robot.local.yaml` FT 配置与 `ros2 control list_controllers \| grep ft` |
| 力方向反 | `compliance_force_feedback_sign` 改为 `1.0` |
| 拖拽时低频振荡（1–3 Hz） | `compliance_force_vel_lpf_alpha` 调小至 0.1–0.2；`compliance_hybrid_force_ki_leak` 调大至 0.8–1.5；必要时增大 `compliance_hybrid_force_damping` |

### 诊断日志（排查"追踪不到位/下垂"）

`compliance_diag_log`（默认 true）+ `compliance_diag_log_period`（默认 1.0 s）控制，
每个臂每周期节流输出一行：

```
[COMPLIANCE diag] left err_p=[dx dy dz] err_r=0.12 | v=[.. .. .. .. .. ..] vach=[..] res=0.003 |qdot|=0.0031 smin=0.042 jlim=[3@hi 5@lo] | f_err=[..] f_eff=[..] track_gap=0.081
```

| 字段 | 含义 | 指向的问题 |
|---|---|---|
| `err` | **位控轴**的位置/姿态误差（base 系，m/rad；力控轴显示 `--`） | 误差大且不收敛 = 没追上 |
| `v` | 各轴最终笛卡尔速度指令（限幅后） | 误差大但 v 小 → 上游（力轴死区）；v 大仍不追 → 下游（J/限位） |
| `vach` | **位控轴**实际达成的任务速度（J·qdot；力控轴显示 `--`） | `v` 与 `vach` 差距大 = 解被衰减（奇异/QP λ/限位边界） |
| `res` | 位控轴**不可达残差**（v−vach 的范数） | 持续接近 \|v\| 且 \|qdot\|≈0 → 目标不可达，已停在最近可达点（λ 阻尼收敛点） |
| `\|qdot\|` | 关节速度范数 | v 大但 qdot≈0 → 奇异衰减 |
| `smin` | J 最小奇异值 | <0.05 且伴随上述现象 → 接近奇异构型 |
| `jlim` | 本周期被限位钳制的关节（`idx@hi/lo`；`coupling` = 6/7 轴 L1 耦合投影生效） | 非空 → 目标超出可达空间，关节顶限位 |
| `f_err`/`f_eff` | 力轴误差 / 死区后误差（N、N·m） | 静止无接触时 \|f_err\|>deadband → 零力残差（重新校零或加大死区） |
| `track_gap` | 实测末端 vs 指令末端距离（m） | 明显增大 → 硬件侧重力补偿不足（下垂在指令之下） |


---

## 编译

```bash
colcon build --packages-select arms_ros2_control_msgs arms_rviz_control_plugin ocs2_arm_controller --symlink-install
source install/setup.bash
```
