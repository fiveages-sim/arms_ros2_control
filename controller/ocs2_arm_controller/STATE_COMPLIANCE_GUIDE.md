# StateCompliance 使用指南

## 概述

StateCompliance 是 OCS2 控制器的一阶力位混合控制状态。进入后：
- **位控轴**（S=0）：TCP 跟踪 teleop 目标（RViz marker 或 topic）
- **力控轴**（S=1）：TCP 按力误差运动，末端输出设定力

控制律（per axis, 100Hz）：
```
位控：v = K · (target − current)        → 经 DLS 映射到关节空间
力控：v = (F_des − F_meas + I_err) / D  → 经 DLS 映射到关节空间
```

---

## 快速上手

### 1. 进入 COMPLIANCE 模式

HOLD 状态下点击 RViz 面板的 **COMPLIANCE** 按钮，或在终端：

```bash
ros2 topic pub /fsm_command std_msgs/msg/Int32 "data: 5" -1
```

日志会显示：

```
Switched from HOLD to COMPLIANCE
COMPLIANCE zero_cal started (10.0 s). Keep arm still.
```

此时手臂静止，等待 10 秒零力校准。进度可通过以下日志观察：

```
COMPLIANCE status: zero_cal=running FT(L=on R=OFF) force_setpoint=0.0 N ...
```

### 2. 校准完成后发目标

等日志出现 `COMPLIANCE zero_cal done` 后，有两种方式发送目标：

**方式 A — RViz marker 拖拽**（直观）：
1. 在 3D 视图中拖拽左右臂的交互 marker 到目标位置
2. 首次发送时姿态会自动 fix 为当前手臂姿态（避免跳变）

**方式 B — 命令行**（精确）：
```bash
ros2 topic pub /left_target geometry_msgs/msg/Pose \
  '{position: {x: 0.5, y: 0.3, z: 0.8}, orientation: {x: 0, y: 0, z: 0, w: 1}}' -1
```

### 3. 启用/调节力输出

默认力目标全为 0（不输出额外力）。在线设置力输出：

```bash
# X 轴输出 5N 力
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5.0, 0, 0, 0, 0, 0]"

# Z 轴输出 -10N（下压）
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[0, 0, -10.0, 0, 0, 0]"
```

查看力控内部状态（每秒 1 次）：

```
COMPLIANCE force_ctl_L: zero_cal=1 ft_ok=1 dt=0.0100  F0: set=5.00 meas=0.02 err=4.98 int=10.00 v=0.0075
```

### 4. 退出 COMPLIANCE

点击 RViz 面板的 **HOLD** 按钮，或：

```bash
ros2 topic pub /fsm_command std_msgs/msg/Int32 "data: 2" -1
```

---

## 常用场景配置

### 纯位置跟踪（默认）

不设力目标，拖拽 marker 直接到位。力控轴 S=1 自动维持力目标（默认 0）。

### 自由空间力控（全部轴浮动）

```bash
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,1,1,1,1,1]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[0,0,0,0,0,0]"
```

手臂会悬浮，轻推即可移动（导纳模式）。I 项自动抵消重力残余。

### 接触力控（X 向 5N 接触，其余位控）

```bash
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,0,0,0,0,0]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5.0,0,0,0,0,0]"
```

手臂追踪目标位姿的 Y/Z/姿态轴，同时 X 轴维持 5N 接触力。

### 导纳拖动

全部轴力控（S=1）+ `compliance_hybrid_force_xmax_lin` 设大（如 0.5m）。拖动手臂时外力被力控器感知为 F_err，产生随动。

---

## 关键诊断日志

| 日志 | 含义 |
|---|---|
| `zero_cal=done FT(L=on)` | 校准完成，FT 在线，力控可用 |
| `zero_cal=running` | 校准进行中，保持静止 |
| `zero_cal=waiting_FT` | FT 传感器未连接，等待中（2s 超时后跳过） |
| `force_ctl_L: ... v=0.0055` | 力控正常，X 轴产生 5.5mm/s 速度 |
| `disp_lin=[20.0 0 0]mm` | X 轴力控位移已触软限（20cm），被刹停 |
| `ft_ok=0` | FT 掉线或校准未完成，力控已停 |

---

## 调参速查

详细参数说明见 [COMPLIANCE_PARAMS.md](COMPLIANCE_PARAMS.md)。常用调整：

```bash
# 位控跟踪更快/更慢
ros2 param set /ocs2_arm_controller compliance_hybrid_pos_stiffness "[30,30,30,15,15,15]"

# 力控更软/更硬（D 越大越柔）
ros2 param set /ocs2_arm_controller compliance_hybrid_force_damping "[3000,3000,3000,150,150,150]"

# 力控位移上限（防止漂移跑飞）
ros2 param set /ocs2_arm_controller compliance_hybrid_force_xmax_lin 0.5

# 零力校准时长（秒）
ros2 param set /ocs2_arm_controller compliance_zero_cal_duration 5.0
```
