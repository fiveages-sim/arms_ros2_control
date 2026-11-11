# OCS2 Arm Controller

A ROS2 Control controller for arm control based on OCS2 (Optimal Control for Switched Systems).

## Overview

This controller implements a finite state machine (FSM) for arm control with the following states:

- **HOME**: Move arm to home position
- **OCS2**: OCS2 MPC optimal control
- **HOLD**: Hold current position

## Features

- Finite State Machine (FSM) implementation
- Position-based control for arm joints
- **Automatic Force Control Mode Detection** - Automatically detects and enables force control when kp, kd, velocity, effort, position interfaces are available
- OCS2 MPC integration for optimal control
- Configurable control parameters
- Support for both simulation and real hardware

## States

### StateHome
- Moves arm to predefined home position
- Uses position control to reach target joint angles



### StateOCS2
- Implements OCS2 MPC optimal control
- Integrates with OCS2 mobile manipulator interface
- Provides optimal trajectory following
- Supports real-time MPC updates

### StateHold
- Holds the current position when entering the state
- Maintains position without movement
- Useful for safety and inspection tasks

## Usage

### Building
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_arm_controller --symlink-install
```

### Running

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py type:=AG2F90-C
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=arx5 type:=r5
```


```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py hardware:=gz type:=AG2F120S
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py hardware:=isaac type:=AG2F90-C
```

### Configuration

Plugin type: `ocs2_arm_controller/Ocs2ArmController`.

`joints`, `update_rate`, `command_interfaces`, `state_interfaces`, and optional `command_prefix` are ros2_control ControllerInterface / controller_manager commons (**Startup only**). Home / MoveJ / waist names shared with `basic_joint_controller` are documented in [`arms_controller_common`](../../libraries/arms_controller_common/README.md) (Runtime items are re-read on the next matching command or state enter; there is no generic parameter callback for those).

There is no `config/ocs2_arm_controller.yaml` in this package. Robot-specific YAML is supplied by `{robot_name}_description` and launch files.

**When**

| Tag | Meaning |
|---|---|
| **Runtime** | `ros2 param set` is picked up without reloading. MoveL items are re-read by `PoseBasedReferenceManager::updateParam` on the next interpolating / stamped command. |
| **Startup only** | Loaded in `on_init` / `CtrlComponent` construction. Frame overrides are injected into the Interface then; changing a tip requires restart. |
| **Unverified** | Declared, but there is no callback or clear re-read path. |

Package-specific parameters:

| Parameter | Default | When | Notes |
|---|---|---|---|
| `home_1` … `home_N` | — | Startup only | FSM HOME targets (require at least `home_1`). Loaded by `StateHome::init()`. |
| `home_joint_reference` | (empty) | Runtime | Explicit soft HOME joint vector (same order as `joints`). Highest priority when non-empty. Param change applies live if already in OCS2; otherwise on next enter. On exit `q_home` resets to `home_1` (param unchanged). Requires `.info` `homeJointReference.activate=true`. No ON/OFF toggle (unlike WBC). |
| `home_joint_reference_id` | `0` | Runtime | 1-based home index (`2` → `home_2`). Used when vector is empty; `0` → `home_1`. Param change applies live in OCS2 / on next enter from HOLD. |
| `robot_name` | `"cr5"` | Startup only | OCS2 files come from `{robot_name}_description` (there is no `robot_pkg` parameter) |
| `default_gains` | (empty) | Startup only | HOME / HOLD impedance `[kp, kd]` when MIX interfaces are present |
| `pd_gains` | (empty) | Startup only | OCS2-state impedance `[kp, kd]` |
| `base_frame` | task.info `baseFrame` | Startup only | Model / reference base; YAML overrides info |
| `left_ee_frame` | task.info `eeFrame` | Startup only | Left tip; not hot-reloadable |
| `right_ee_frame` | task.info `eeFrame1` | Startup only | Right tip; not hot-reloadable |
| `recompile_libraries` | task.info `recompileLibraries` | Startup only | YAML overrides info; Isaac/`hardware:=isaac` defaults `false` (reuse CppAD cache). Set `true` after model/constraint changes or clear `/tmp/ocs2_ros2/...` |
| `movel_duration` | `2.0` | Runtime | MoveL duration (s); next stamped / interpolating command |
| `movel_trajectory_duration` | `2.0` | Runtime | MoveL trajectory duration |
| `movel_sample_interval` | `0.04` | Runtime | MoveL sample interval |
| `movel_max_linear_velocity` | `0.3` | Runtime | Linear vel limit |
| `movel_max_linear_acceleration` | `1.0` | Runtime | Linear acc limit |
| `movel_max_linear_jerk` | `2.0` | Runtime | Linear jerk limit |
| `movel_max_angular_velocity` | `1.0` | Runtime | Angular vel limit |
| `movel_max_angular_acceleration` | `2.0` | Runtime | Angular acc limit |
| `movel_max_angular_jerk` | `4.0` | Runtime | Angular jerk limit |
| `movel_auto_extend_duration` | `true` | Runtime | Extend MoveL duration when limits require it |

`ocs2_wbc_controller` is a private submodule and was not readable here. Its extra frame param `body_frame` is **Startup only** per existing `arms_target_manager` notes.

## Interface Configuration

### Automatic Control Mode Detection

The controller automatically detects the available control mode based on the provided interfaces:

**Position Control Mode** (Default):
- **Command Interface**: `position` only
- **State Interface**: `position` and `velocity`
- Suitable for most industrial and research robotic arms

**Force Control Mode** (Auto-detected):
- **Command Interface**: `position`, `velocity`, `effort`, `kp`, `kd`
- **State Interface**: `position`, `velocity`, `effort`
- Automatically enabled when all required interfaces are available
- Provides full force control capabilities with impedance control

### Configuration Examples

**Position Control Configuration:**
```yaml
command_interfaces:
  - position
state_interfaces:
  - position
  - velocity
```

**Force Control Configuration:**
```yaml
command_interfaces:
  - position
  - velocity
  - effort
  - kp
  - kd
state_interfaces:
  - position
  - velocity
  - effort

# Impedance gains [kp, kd] (Startup only)
default_gains: [100.0, 10.0]  # HOME / HOLD
pd_gains: [100.0, 10.0]       # OCS2 state
```

### Force Control Gains

`default_gains` and `pd_gains` are `[kp, kd]` vectors loaded at **Startup only** (the older `force_gains` name is not a parameter):

- **kp** (Position gain): Controls the stiffness of the position control loop
  - Higher values make the robot more rigid
  - Lower values make the robot more compliant
  - Typical range: 10.0 - 1000.0

- **kd** (Velocity gain): Controls the damping of the velocity control loop
  - Higher values reduce oscillations and improve stability
  - Lower values allow more natural motion
  - Typical range: 1.0 - 100.0

**Example configurations:**
- High stiffness: `default_gains: [500.0, 50.0]` / `pd_gains: [500.0, 50.0]` — precise positioning
- Medium compliance: `[100.0, 10.0]` — general manipulation
- High compliance: `[50.0, 5.0]` — contact tasks and human interaction

## State Transitions

The controller supports state transitions based on control input commands:

- **Command 1**: Transition to HOME state  
- **Command 2**: Transition to HOLD state
- **Command 3**: Transition to OCS2 state

States can transition between each other based on the control input received on the `/control_input` topic.

**Note**: The controller starts in HOLD state by default. OCS2 state can only transition back to HOLD state.

**State Transition Rules:**
- **HOLD → OCS2**: Command 3
- **OCS2 → HOLD**: Command 2  
- **HOLD → HOME**: Command 1
- **HOME → HOLD**: Command 2

**Note**: The controller starts in HOLD state by default. OCS2 state can only transition back to HOLD state.

## OCS2 Integration

The OCS2 state integrates with the OCS2 mobile manipulator framework:

- **Task File**: Located at `{robot_name}_description/config/ocs2/task.info`
- **Planning URDF**: Xacro-generated cache via `robot_common_launch` (`planning_urdf_path`); static `urdf/*.urdf` is not used. The same xacro path applies to `manipulator_ocs2.launch.py` and `humanoid_ocs2.launch.py` in `robot_common_launch` (via `resolve_planning_urdf_file_or_fail`).
- **Generated Library**: Located at `{robot_name}_description/config/ocs2/generated`

The controller automatically loads these files from `{robot_name}_description` (`robot_name` default `"cr5"`).
## Action 时间输入（新版消息，不兼容旧生成代码）

JointTrajectory Goal 使用整条任务的 `time_mode` 和 `movej_trajectory_duration`（秒，0 表示未指定），
JointWaypoint 使用 `builtin_interfaces/Duration time_from_start` 累计到达时间，
不再有逐点 time_mode/movej_trajectory_duration。Result/Feedback 不变。

时间模式需显式设置 `time_mode: true`（ROS bool 默认 false）：

- 正 movej_trajectory_duration + 各点时间全零：按路径长度分配总时长，并在任务实际开始规划时
  更新 ROS 参数 movej_trajectory_duration；新值持续影响后续默认时间任务。
- movej_trajectory_duration=0 + 各点非负且非递减的累计时间：相邻时间相减得到段时长，不更新参数。
- movej_trajectory_duration=0 + 全零路点时间：使用当前 movej_trajectory_duration 分配。
- 总时间和逐点时间同时提供、累计时间倒序或非法 Duration：拒绝。

所有插值器接收同一组移动段时长。只给总时长/约束时合并相邻重复位置后规划；
逐点累计时间允许相等，但零时间只能对应零位移并被跳过；零时间有位移拒绝。
逐点时间中的正时间零位移表示停留，按指定时长保持。DOUBLES 在停留边界停车，
不跨停留转接；运动块内部继续使用原有 blending。全部段被跳过时立即成功，
planned_duration=actual_duration=0，不启动运动。DOUBLES 移动段必须长于一个控制周期。
首点是目标；若首点 t=0，仅当其与真实当前位置相同时跳过。
约束模式 `time_mode: false` 仅支持 DOUBLES，所有时间字段须为零；
linear/tanh 明确拒绝约束模式，时间模式也不新增 max_* 约束保证。
DOUBLES 非零 blending 可能改变实际轨迹时间及经过中间点的行为，查看 planned_duration。

Goal 拒绝、执行前取消或被替换不更新参数。延后 Goal 在实际应用时才更新；
更新成功但随后规划失败时，参数仍保留新值，无事务回滚。正在执行的轨迹使用
已生成的时间快照，后续外部参数更新不重新定时。持久设置失败会返回失败结果。

旧客户端每段 [2,3] 秒应迁移为累计 [2,5] 秒，不可只重命名字段。
旧 JointTrajectory Service 类型、生成配置和公共实现已删除。
需要重建消息包及所有依赖消费者并重启，禁止混用新旧类型生成代码。
本次只改 Action，Topic 现有时间机制保持原样。

例如原生 Goal（两个关节、两个目标，2 秒和 5 秒到达）：

```yaml
joint_names: [joint1, joint2]
time_mode: true
movej_trajectory_duration: 0.0
waypoints:
  - position: [0.1, 0.2]
    time_from_start: {sec: 2, nanosec: 0}
  - position: [0.3, 0.4]
    time_from_start: {sec: 5, nanosec: 0}
```

若只要整条轨迹共 5 秒，改为 movej_trajectory_duration: 5.0，且两个路点 time_from_start 都留零。
Python 接口及其示例需要另行迁移后才能使用本版消息。
