# OCS2 Arm Controller

A ROS2 Control controller for arm control based on OCS2 (Optimal Control for Switched Systems).

## Overview

This controller implements a finite state machine (FSM) for arm control with the following states:

- **HOME**: Move arm to home position
- **ZERO**: Move arm to zero position
- **OCS2**: OCS2 MPC optimal control
- **HOLD**: Hold current position

## Features

- Finite State Machine (FSM) implementation
- Position-based control for arm joints
- OCS2 MPC integration for optimal control
- Configurable control parameters
- Support for both simulation and real hardware

## States

### StateHome
- Moves arm to predefined home position
- Uses position control to reach target joint angles

### StateZero
- Moves arm to predefined zero position
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

The controller can be configured through the YAML configuration file located at:
`config/ocs2_arm_controller.yaml`

Key parameters:
- `joints`: List of joint names
- `home_pos`: Home position for each joint
- `zero_pos`: Zero position for each joint
- `robot_pkg`: Robot package name for OCS2 configuration
- `update_rate`: Controller update rate in Hz

## Interface Configuration

The controller uses the following interfaces:
- **Command Interface**: `position` only (realistic for most arm hardware)
- **State Interface**: `position` and `velocity` (for state estimation)

This configuration is suitable for most industrial and research robotic arms.

## State Transitions

The controller supports state transitions based on control input commands:

- **Command 1**: Transition to HOME state  
- **Command 2**: Transition to ZERO state
- **Command 3**: Transition to HOLD state
- **Command 4**: Transition to OCS2 state

States can transition between each other based on the control input received on the `/control_input` topic.

## OCS2 Integration

The OCS2 state integrates with the OCS2 mobile manipulator framework:

- **Task File**: Located at `{robot_pkg}/config/ocs2/task.info`
- **URDF File**: Located at `{robot_pkg}/urdf/robot.urdf`
- **Generated Library**: Located at `{robot_pkg}/config/ocs2/generated`

The controller automatically loads these files based on the `robot_pkg` parameter. 