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

若此前尚未清零，首次进入时会自动校准：手臂静止约 10 s（`compliance_zero_cal_duration`），
日志出现 `zero_cal done` 后力控可用。已在其他模式清零的结果会直接沿用。
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
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5.0,0.0,0.0,0.0,0.0,0.0]"
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

## 辨识功能

RViz **ComplianceForcePanel** 将底层位置环／接触链路辨识和末端负载辨识放在两个标签中。
当前分支的位置环辨识只有面板客户端，尚无对应服务端；负载辨识可直接使用。
未显示面板时通过 **Panels → Add Panel → ComplianceForcePanel** 添加。

### 末端负载：面板操作

安装关系：**机械臂 → 六维力／力矩传感器 → 末端负载**。
辨识传感器下游工具、夹具和工件的总质量、传感器坐标系重心以及原始六维读数的常值零偏；
不辨识机械臂自身动力学或负载惯量张量，也不会自动修改控制器／硬件参数。

1. 进入 COMPLIANCE，取消六个「力控」勾选并应用，令 `S=[0,0,0,0,0,0]`。
2. 打开「末端负载辨识」，选择左／右臂，确认原始 FT 话题、关节反馈话题和重力参考系。
   面板自动读取控制器中所选臂的全部关节，用于检查停稳，无需手动输入关节名；读取不到时不能开始。
3. 点击「开始负载辨识」，选择保存目录。
4. 通过 COMPLIANCE 目标位姿／交互 marker 调整姿态，静止且末端无接触后点击「采集当前姿态」。
   至少采集 6 个姿态，默认 9 个，需绕两个不同轴倾斜；仅绕重力方向旋转不足以辨识完整重心。
5. 采完自动拟合并显示结果。「停止采样」或关闭面板会结束采集，保留已完成姿态的数据。

进入 COMPLIANCE 不会自动切成全位控：默认 `S=[1,0,0,0,0,0]`，Fx 仍是力控。
需取消全部「力控」勾选并点击「应用设定」，等控制器回读为全 0 后再开始。
这里的六轴指 Fx/Fy/Fz/Mx/My/Mz，与机械臂关节数无关。
启动失败时，面板会区分状态缺失／超时、其他辨识运行和 S 未归零，并显示控制器实际 S 值。

面板采样不发送运动或 FSM 指令。退出 COMPLIANCE、状态超过 0.5 s 未更新、启用力控轴，
或状态报告其他辨识正在运行时，采集会中止。停止采样不会停止外部运动；需要保持机械臂时使用 HOLD。

输入应为未经工具重力补偿的 `WrenchStamped`（力 N、力矩 N·m），不要使用 `wrench_filtered`。
`frame_id` 必须对应传感器实际轴方向及力矩原点；时间戳须与关节状态、TF 使用相同时钟。
工具按每条力消息的时间查询 TF，将参考系中的重力转到传感器系；面板默认重力沿参考系 -Z。
采样期间负载组成和内部构型保持不变，不接触环境、不重新清零传感器，并避免线缆拉扯。

### 命令行与自动姿态

工具随 `ocs2_arm_controller` 安装；`collect` 为回车触发的手动采样，`fit` 为离线拟合，
`make-plan` 生成候选姿态，`run` 执行计划。各命令参数可用 `--help` 查看。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ocs2_arm_controller identify_payload.py --help

# 用实际关节名替换 joint1…joint7；生成计划时不会运动
ros2 run ocs2_arm_controller identify_payload.py make-plan \
  --motion-backend arms \
  --joints joint1 joint2 joint3 joint4 joint5 joint6 joint7 \
  --sweep-joints joint5 joint6 --angle-deg 15 \
  --wrench-topic /left_ft_broadcaster/wrench --gravity-frame world \
  --output payload_poses.yaml

# 先检查计划；不带 --execute 只校验文件
ros2 run ocs2_arm_controller identify_payload.py run --plan payload_poses.yaml
ros2 run ocs2_arm_controller identify_payload.py run --plan payload_poses.yaml \
  --execute --output-dir payload_run01

# 可对保存的原始数据重新拟合
ros2 run ocs2_arm_controller identify_payload.py fit \
  --input payload_run01/samples.csv --output payload_refit.yaml
```

`arms` 使用本仓库 MOVEJ：HOLD → MOVEJ → HOLD；这是独立于面板采样的自动运动流程。
标准轨迹控制器可改用 `--motion-backend follow_joint_trajectory --trajectory-action <action名称>`，
异常时取消本工具发起的轨迹。普通 `collect` 不依赖 FSM；`fit` 可直接运行 Python 脚本，无需 ROS。
自动计划不包含碰撞检查，执行前需检查起点、所有目标及中间路径。结束后不自动返回起点。

计划 YAML 可调整采样与运动阈值，主要默认值如下：

| 参数 | 默认值与含义 |
|---|---|
| `move_seconds` / `max_velocity` | 5 s / 0.15 rad/s；轨迹请求值，实际执行由控制器决定 |
| `max_step_rad` / `position_tolerance` | 0.7 rad 最大相邻关节变化 / 0.02 rad 到位容差 |
| `still_velocity` / `settle_seconds` | 实测速度 ≤0.01 rad/s，持续静止 1.5 s |
| `sample_seconds` / `min_samples` | 每姿态采样 2 s，至少 30 条有效数据 |
| `data_timeout` / `wait_timeout` | 数据新鲜度 0.5 s / 就绪及到位等待 30 s |
| `max_force_std` / `max_torque_std` | 单姿态三轴标准差范数上限 0.5 N / 0.05 N·m |
| `max_force_rms` / `max_torque_rms` | 拟合残差向量 RMS 上限 0.5 N / 0.05 N·m |
| `max_condition` | 归一化拟合矩阵条件数上限 1000，过大表示姿态多样性不足 |

### 输出与应用

每次使用新目录，保存 `plan.yaml`、`samples.csv`、`result.yaml`；失败时另存 `failure.txt`。
每个姿态通过检查后才写入 CSV。结果包括 `mass_kg`、`center_of_mass_m/mm`、
`force_bias_N`、`torque_bias_Nm`、`wrench_sign`、残差和条件数。
`valid: true` 仅表示数据通过当前模型的残差检查，不是精度认证。
姿态不足、秩不足或质量低于 1 g 时拒绝估算；残差超限时保存 `valid: false` 并报错。

各姿态先求均值，再等权拟合 `F = sign·m·g + bias_F`、`T = sign·(m·c) × g + bias_T`，
其中 `×` 为叉乘，`c` 是传感器原点到重心的向量；力与力矩须采用一致的符号约定。
若硬件参数使用法兰坐标系，先转换 `c_flange = R_flange_sensor·c_sensor + p_flange_sensor`，
并核对 m/mm 单位。静态结果不能直接替代厂商要求的完整 10 维动力学参数。

---

## 接口

### 写入（动态参数）

节点：`/ocs2_arm_controller`

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `compliance_task_selection` | `float64[6]` | `[1,0,0,0,0,0]` | 1=力控，0=位控 |
| `compliance_force_setpoint` | `float64[6]` | 全 0 | 目标力，仅 S=1 轴生效；力各轴 ±20 N，力矩各轴 ±5 N·m |

任一目标分量超限，`ros2 param set` 会返回失败并说明轴、数值和允许范围，整组目标保留原值。
参数服务返回 `successful=false`，终端输出 `Setting parameter failed`；本机 ROS CLI 此时退出码仍为 0，
自动化脚本应检查参数服务响应，不能仅依赖命令退出码。
边界值允许；NaN、无穷值、错误类型或非 6 维数组会被拒绝，即使该轴当前未启用力控。
启动 YAML 中的目标也进行相同检查，超限会导致控制器初始化失败。面板使用控制器发布的相同范围。
这是目标值校验，不包含实测接触力超限停机逻辑。

```bash
# Fx=21 N 超过 20 N，返回失败，原目标不变
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[21.0,0.0,0.0,0.0,0.0,0.0]"
```

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

控制器激活期间在所有 FSM 状态中持续发布；尚未完成静止清零时，
零偏项为 0。若 broadcaster 自身配置了 filter chain，不要再让它发布同名
`wrench_filtered`，避免同一话题出现两个数据源。

### 手动清零

控制器激活后，在任意 FSM 模式（包括 HOLD / HOME / MOVEJ / OCS2 等非 COMPLIANCE 模式）
均可在 RViz Compliance Force Panel 点击「传感器清零」，或调用：

```bash
ros2 service call /compliance_zero_wrench std_srvs/srv/Trigger '{}'
```

清零使用 `compliance_zero_cal_duration` 配置的静止采样时间。采样期间保持机械臂静止、末端无接触；
服务返回成功表示已接受清零请求，完成时日志输出 `zero_cal done`。清零不会切换当前控制模式；
COMPLIANCE 力控轴在新零偏标定完成前保持禁用。校准过程与零偏结果跨模式保留，
进入 COMPLIANCE 时不会覆盖已完成的清零；未请求过清零时，首次进入 COMPLIANCE 会自动启动校准。

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
| `compliance_hybrid_force_pos_stiffness` | 全 0 | 力控轴上的回弹（位控）刚度 [1/s]：`v -= K·行程`。0 = 纯导纳（现状：无恢复力，误差追不上时会漂到行程限位后停住）；旋转力控轴设 `0.5~2.0` 可稳定在有限偏置并在误差消失后回中 |
| `compliance_hybrid_force_realize_tol` | `0.25` | 响应守卫：实测/指令行程比低于此值视为"未实现"（设 `0` 关闭守卫） |
| `compliance_hybrid_force_stall_time` | `0.2` | 响应守卫：持续多久确认停滞 [s] |
| `compliance_hybrid_force_stall_release` | `1.0` | 响应守卫：确认停滞后的虚拟位移回退速率 [1/s]（决定重试节奏，≈ln(200)/该值 秒） |
| `compliance_hybrid_force_ki` | `2.0` | 积分增益，消重力残余 |
| `compliance_hybrid_force_ki_max` | `10.0` | 积分上限 |
| `compliance_hybrid_force_ki_leak` | `0.5` | 积分泄漏 [1/s]；拖拽振荡时调大（0.5–1.5） |
| `compliance_hybrid_force_deadband` | `0.5` | 力误差死区 |
| `compliance_force_feedback_sign` | `-1.0` | 力方向反了改为 `1.0` |
| `compliance_force_vel_lpf_alpha` | `0.3` | 导纳输出低通（虚拟惯性）；振荡调小（0.1–0.3），迟钝调大 |
| `compliance_hybrid_force_xmax_lin` | `0.2` | 力控各平移轴行程 [m]；实测越界切 HOLD |
| `compliance_hybrid_force_xmax_ang` | `0.3` | 力控各旋转轴行程 [rad]；实测越界切 HOLD |
| `compliance_hybrid_force_xmax_margin_ratio` | `0.2` | 软限渐缓区占 xmax 比例 |

### 鲁棒导纳：响应守卫

力控轴的信号链是 `wrench → err → 1/D 速度 → 逆解 → 关节位置指令`，**到这里就断了**——闭环必须由环境合上。力轴的环境能合上（压深必然增加反力），力矩轴常常合不上，那条通路就变成开环积分器：误差恒定、位置指令无界累积。

- **行程门限改用"指令行程"**（导纳自己的虚拟位移，有界积分），不再用实测位移——被堵住时实测不增长，旧门限永不触发。
- **响应守卫**：累积行程比 `实测/指令` ≈ 0（请求了但没动）持续 `stall_time` → **保持、不升级**，并让虚拟位移缓慢回退以自动重试。用累积行程比而非瞬时速度比，避免被底层位置环延迟误判。

### 关节限幅与求解

#### 实测工作空间越界保护

对 **S=1 的力控方向**，每周期由实测关节位置计算 TCP 位姿，在双臂混合控制求解前检查：

- 每个方向启用时，以该时刻的实测位姿作为参考；目标位姿更新不会移动这个参考。
- 平移逐轴比较参考点的偏移；旋转按 base 轴分别累加**实测姿态的逐周期增量**（小角度旋转向量），
  得到"自本轴启用以来绕该轴的行程"。
  - 不再使用 `R_actual · R_referenceᵀ` 的最短旋转向量投影：该投影在总转角接近 180° 时会失真
    甚至恒为 0（越界保护整个失效），且旋转不可交换，会把别的轴转出的姿态算进本轴行程
    （力矩轴的行程预算被别的轴吃掉 → 被软限位冻结后无法再调整）。
  - 行程是**有符号净转角**（转出去再转回来会归零），不是路径长度。
  - 力控轴顶到行程限位、输出被强制归零时：抛出节流 WARN，并在诊断日志 `fdisp` 里以 `!` 标记。
- 接近边界按原有渐缓区减速；任一臂的任一力控方向实测越界时，停止本周期混合控制输出，
  将双臂位置指令设为当前有效实测关节位置、速度前馈归零，并在 FSM 下一周期进入 HOLD。
- 日志输出手臂、方向、实测偏移及上限。触发后保持锁定，移动回范围内也不会自动恢复 COMPLIANCE；
  手动重新进入会建立新的参考。关闭力控轴的当周期仍检查其原边界，避免切换掩盖越界。
- 力控开启时，缺失／非有限位置反馈、运动学不可用或无效行程上限也会请求 HOLD。

这沿用力控行程参数，不对 S=0 的位控方向额外设置固定空间边界；上下限是逐轴值，不是平移距离球半径。
HOLD 保持触发位置，不自动规划回退，也不等同于硬件急停；持续外力仍可能推动机械臂。

#### 关节参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `compliance_hybrid_joint_vmax` | `0.8` | 关节速度上限 [rad/s] |
| `compliance_hybrid_joint_limit_margin` | `0.02` | 关节限位裕度 [rad] |
| `compliance_wrist_coupling_max` | `0.0`（关闭） | 6/7 轴耦合上限：\|q6\|+\|q7\| ≤ max [rad]（110° = `1.9199`，说明书图 4-4 八边形可行域的斜边；直边已由 URDF 限位覆盖）。≤0 关闭 |
| `compliance_hybrid_dls_lambda` | `0.05` | DLS 正则化系数 |

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
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[0.0,0.0,0.0,0.0,0.0,0.0]"
```

**接触力控（X 向 5 N，其余位控）**：
```bash
ros2 param set /ocs2_arm_controller compliance_task_selection "[1,0,0,0,0,0]"
ros2 param set /ocs2_arm_controller compliance_force_setpoint "[5.0,0.0,0.0,0.0,0.0,0.0]"
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
| 力控力矩轴不响应扭矩偏差、扭矩一直增大不回弹 | 看 `fdisp`：`?` = 响应守卫已确认"环境没有给增益"（保持中）；`!` = 行程用尽。两者都不解决物理问题——需加弹性层（造出 $k_\theta$）、把 TCP 标到接触面中心、或把该轴改回位控 `S=0` |
| 拖拽时低频振荡（1–3 Hz） | `compliance_force_vel_lpf_alpha` 调小至 0.1–0.2；`compliance_hybrid_force_ki_leak` 调大至 0.8–1.5；必要时增大 `compliance_hybrid_force_damping` |

### 诊断日志（排查"追踪不到位/下垂"）

`compliance_diag_log`（默认 true）+ `compliance_diag_log_period`（默认 1.0 s）控制，
每个臂每周期节流输出一行：

```
[COMPLIANCE diag] left err=[-- -- -- +0.012 +0.004 -0.031] | v=[+0.02 +0.01 -0.03 +0.05 +0.02 +0.16] vach=[+0.02 +0.01 -0.03 +0.05 +0.02 +0.16] res=0.003 | |qdot|=0.0031 escape=0.000/0.00 smin=0.042 kkt=0.00e+00 align=[0.0e+00 0.0e+00 pos=0.0e+00] clip=0.0e+00 bounds=[] sat=[] jlim=[3@hi 5@lo] | qgap=[j1 +0.0000] tcp_gap=[0.001 m 0.002 rad] target=[0 0.000 m 0.000 rad] | f_err=[1.20 0.20 0.10 0.90 0.05 0.02] f_eff=[0.70 0.00 0.00 0.40 0.00 0.00] fdisp=[+0.0201 +0.0000 +0.0000 +0.3000! +0.0000 +0.0000]
```

| 字段 | 含义 | 指向的问题 |
|---|---|---|
| `err` | **位控轴**的位置/姿态误差（base 系，m/rad；力控轴显示 `--`） | 误差大且不收敛 = 没追上 |
| `v` | 各轴最终笛卡尔速度指令（限幅后） | 误差大但 v 小 → 上游（力轴死区）；v 大仍不追 → 下游（J/限位） |
| `vach` | 各轴实际达成的任务速度（J·qdot；力控轴同样输出真实值） | `v` 与 `vach` 差距大 = 解被衰减（奇异/QP λ/限位边界） |
| `res` | 位控轴**不可达残差**（v−vach 的范数） | 持续接近 \|v\| 且 \|qdot\|≈0 → 目标不可达，已停在最近可达点（λ 阻尼收敛点） |
| `\|qdot\|` | 关节速度范数 | v 大但 qdot≈0 → 奇异衰减 |
| `smin` | J 最小奇异值 | <0.05 且伴随上述现象 → 接近奇异构型 |
| `jlim` | 本周期被限位钳制的关节（`idx@hi/lo`；`coupling` = 6/7 轴 L1 耦合投影生效） | 非空 → 目标超出可达空间，关节顶限位 |
| `f_err`/`f_eff` | 力轴误差 / 死区后误差（N、N·m） | 静止无接触时 \|f_err\|>deadband → 零力残差（重新校零或加大死区） |
| `fdisp` | 力控轴行程 **指令/实测**（m / rad）。指令侧涨而实测侧不涨 = 发了但没动；`!` = 指令行程顶到软限位；`?` = 已确认停滞（保持、不升级） | 出现 `?` 或 `!` 且 `f_err` 持续大 = 该轴已无调整能力（环境不给增益 / 几何增益变号 / 行程用尽） |
| `track_gap` | 实测末端 vs 指令末端距离（m） | 明显增大 → 硬件侧重力补偿不足（下垂在指令之下） |


---

## 编译

```bash
colcon build --packages-select arms_ros2_control_msgs arms_rviz_control_plugin ocs2_arm_controller --symlink-install
source install/setup.bash
```
