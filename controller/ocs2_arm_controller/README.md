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
| `home_pos` | (empty) | Startup only | Fallback HOME target if `home_1` … are absent |
| `rest_pos` | (empty) | Startup only | Rest pose (this is not `zero_pos`; that name is not a parameter) |
| `robot_name` | `"cr5"` | Startup only | OCS2 files come from `{robot_name}_description` (there is no `robot_pkg` parameter) |
| `default_gains` | (empty) | Startup only | HOME / HOLD impedance `[kp, kd]` when MIX interfaces are present |
| `pd_gains` | (empty) | Startup only | OCS2-state impedance `[kp, kd]` |
| `base_frame` | task.info `baseFrame` | Startup only | Model / reference base; YAML overrides info |
| `left_ee_frame` | task.info `eeFrame` | Startup only | Left tip; not hot-reloadable |
| `right_ee_frame` | task.info `eeFrame1` | Startup only | Right tip; not hot-reloadable |
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