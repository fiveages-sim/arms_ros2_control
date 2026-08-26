# arms_ros2_control_msgs

ROS 2 接口包：[`arms_ros2_control`](../..) 系统所使用的 **msg / srv / action** 定义。按使用逻辑分类如下。

---

## 1. 遥操（Teleop）

手柄 / 键盘遥操节点发布；可视化可订阅模式快照与增量输入。

### `Inputs`（msg）

高频增量控制（如 topic `control_input`）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `x`, `y`, `z` | `float32` | 位置增量 |
| `roll`, `pitch`, `yaw` | `float32` | 姿态增量 |
| `target` | `int32` | 目标臂：`1` 左臂，`2` 右臂 |
| `hand_command` | `float32` | 夹爪：`NaN` 无命令；`0` 闭合 / `1` 张开；其它为 `0~1` 开合比例 |

### `TeleopMode`（msg）

遥操模式快照（如 topic `teleop_mode`）。建议 QoS：`Reliable` + `TransientLocal`；启动发一次，激活状态 / 模式 / 镜像 / 速度切换时再发。目标臂不在此消息中，请读 `Inputs.target`。

| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | `builtin_interfaces/Time` | 发布时间戳（`sec` + `nanosec`） |
| `active` | `bool` | 是否激活：joystick 需右摇杆按下启用后才为 `true`；keyboard 始终为 `true` |
| `control_mode` | `uint8` | `CONTROL_ARM=0` / `CONTROL_CHASSIS=1` |
| `mirror_movement` | `bool` | 镜像开关 |
| `speed_level` | `int32` | 档位：keyboard `1~10`；joystick `0`=LOW / `1`=HIGH |
| `speed_scale` | `float32` | 实际速度倍率 |

**分工**：界面用 `TeleopMode` 看模式状态，用 `Inputs.target` 看当前目标臂。joystick 在底盘模式下可能暂停周期发 `Inputs`，界面应缓存最近一次 `target`。

---

## 2. 全身控制状态（WBC）

`ocs2_wbc_controller` 能力与当前约束状态。

### `WbcCapability`（msg）

控制器具备哪些约束功能。

| 字段 | 类型 | 说明 |
|------|------|------|
| `has_mobile_base` | `bool` | 移动底盘 |
| `has_body_relative_constraint` | `bool` | 身体相对约束 |
| `has_waist_lock` | `bool` | 腰部锁定 |
| `has_head_coupling` | `bool` | 头部耦合 |
| `has_custom_joint_lock` | `bool` | 自定义关节锁定 |
| `has_bimanual_coupling` | `bool` | 双臂耦合 |
| `body_tracking_ee_enabled` | `bool` | 身体跟踪末端是否启用 |
| `has_home_joint_reference` | `bool` | HOME 关节参考约束 |

### `WbcCurrentState`（msg）

当前约束模式（常量 + 状态字段）。

| 字段 | 类型 | 取值 |
|------|------|------|
| `base_state` | `uint8` | `BASE_LOCKED=0` / `BASE_UNLOCKED=1` |
| `body_state` | `uint8` | `BODY_FREE=0` / `VERTICAL=1` / `TRACKING=2` / `LOCKED=3` / `HEAD_COUPLED=4` / `CUSTOM_LOCKED=5` |
| `bimanual_state` | `uint8` | `BIMANUAL_INDEPENDENT=0` / `BIMANUAL_COUPLED=1` |
| `left_arm_state` / `right_arm_state` | `uint8` | `ARM_DISABLED=0` / `ARM_ENABLED=1` |
| `home_joint_reference_enabled` | `bool` | HOME 关节参考约束是否打开 |

---

## 3. 笛卡尔轨迹（直线运动 / 圆弧）

末端空间规划与执行：消息描述目标，服务同步下发，action 带进度反馈。

### 参数消息

#### `LinearMessage`（msg）— 直线

| 字段 | 类型 | 说明 |
|------|------|------|
| `endpoint` | `geometry_msgs/Pose` | 直线终点 |
| `right_endpoint` | `geometry_msgs/Pose` | 双臂右臂终点 |
| `max_linear_*` / `max_angular_*` | `float64` | 线/角速度、加速度、jerk 上限 |
| `duration` | `float64` | 时间模式下运动时长 |
| `time_mode` | `bool` | `true` 时间模式；`false` 参数约束 |
| `frame_id` | `string` | 坐标系 |
| `arm_name` | `string` | `"left"` / `"right"` / `"both"` |
| `ik_type` | `string` | `"BFGS"` / `"DLS"` / `"AUTO"` |

#### `CircleMessage`（msg）— 圆弧

| 字段 | 类型 | 说明 |
|------|------|------|
| `midpoint` / `endpoint`（及 `right_*`） | `geometry_msgs/Pose` | 三点法 |
| `center` / `axis` / `rotate_angle`（及 `right_*`） | Point / Vector3 / float64 | 参数法 |
| `max_linear_*` / `max_angular_*` | `float64` | 运动约束 |
| `duration` | `float64` | 时间模式时长 |
| `use_three_point_method` | `bool` | `true` 三点法；`false` 参数法 |
| `use_slerp_for_orientation` | `bool` | 姿态球面插值 |
| `time_mode` | `bool` | 时间模式 / 参数约束 |
| `frame_id` / `arm_name` / `ik_type` | `string` | 同直线；`ik_type` 另含 `"SDK"` |

### 执行接口

| 接口 | 类型 | 请求 / Goal | 结果要点 |
|------|------|-------------|----------|
| `ExecuteLinear` | srv | `LinearMessage` | `success`, `message`, `estimated_duration` |
| `ExecuteLinear` | action | `LinearMessage` | 另含 `actual_duration`；反馈 `progress` / 时间 |
| `ExecuteCircle` | srv | `CircleMessage` | `success`, `message`, `estimated_duration` |
| `MovecUseIK` | srv / action | `CircleMessage` | srv 同圆；action 另含实际时长与进度反馈 |

### 路径点序列

按 `nav_msgs/Path` 下发笛卡尔路径（非单段直线/圆弧参数）。

| 接口 | 类型 | 请求要点 | 响应要点 |
|------|------|----------|----------|
| `ExecutePath` | srv | 左右臂 `Path` + `trajectory_duration` | `success`, `message`, `estimated_duration` |
| `CartesianPath` | srv | 左右 `Path` + `duration` | 同上 |

---

## 4. 关节空间轨迹

### `JointWaypoint`（msg）

单路点（movej / speedj）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `position` | `float64[]` | 目标关节位置 |
| `velocity` | `float64[]` | 关节速度 |
| `blend_ratio_percent` | `float64` | 多点转接比例 `0~1` |
| `max_velocity` / `max_acceleration` / `max_jerk` | `float64[]` | 规划上限；空或长度不匹配时，控制器用 `movej_max_velocity` / `movej_max_acceleration` / `movej_max_jerk`（默认 2.0 / 4.0 / 20.0，与 lina 关节默认一致） |
| `time_mode` | `bool` | 是否时间模式 |
| `total_time` | `float64` | 总时间 |

### 执行接口

| 接口 | 类型 | 请求 / Goal | 结果要点 |
|------|------|-------------|----------|
| `JointTrajectory` | srv | `joint_names` + `JointWaypoint[]` | `success`, `message`, `planned_duration` |
| `JointTrajectory` | action | 同上 | 另含 `actual_duration`；反馈进度与时间 |

---

## 5. 运动学求解

### `KinematicsService`（srv）

正/逆运动学（FK / IK）。

**请求**：`operation_type`（`"fk"` / `"ik"`）、`arm_type`（`"left"` / `"right"` / `"both"`）、`solver_type`（`"dls"` / `"bfgs"` / `"auto"`）、`joint_angles`、`target_poses`，以及迭代/容差等 IK 参数。

**响应**：`success`、`message`、`result_poses` / `result_joint_angles`，以及求解器名、迭代次数、误差、耗时、是否超工作空间等。

---

## 6. 腰部姿态

### `WaistLiftingPose`（action）

| 部分 | 字段 | 说明 |
|------|------|------|
| Goal | `mode` | `MODE_ABSOLUTE=0` / `MODE_RELATIVE=1` |
| Goal | `x`, `z`, `phi` | 目标腰部位姿参数 |
| Result | `reachable`, `success`, `error_code`, … | 含规划结果与错误码 |
| Feedback | `progress` | 执行进度 |

错误码常量：`ERROR_NONE`、`INVALID_MODE`、`CONTROLLER_NOT_READY`、`UNREACHABLE`、`RUNTIME_FAILED`、`CANCELED`。

---

## 依赖

- `builtin_interfaces`（`TeleopMode.stamp`）
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
