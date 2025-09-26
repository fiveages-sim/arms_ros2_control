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
* OCS2 Arm Controller
```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_arm_controller --symlink-install
```
* Dobot CR5 description package with config files
```bash
cd ~/ros2_ws
colcon build --packages-up-to cr5_description --symlink-install
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
- `force_gains`: Default force control gains [kp, kd] for impedance control

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

# Force control gains [kp, kd]
force_gains: [100.0, 10.0]  # kp=100, kd=10
```

### Force Control Gains

The `force_gains` parameter defines the default impedance control gains for force control mode:

- **kp** (Position gain): Controls the stiffness of the position control loop
  - Higher values make the robot more rigid
  - Lower values make the robot more compliant
  - Typical range: 10.0 - 1000.0

- **kd** (Velocity gain): Controls the damping of the velocity control loop
  - Higher values reduce oscillations and improve stability
  - Lower values allow more natural motion
  - Typical range: 1.0 - 100.0

**Example configurations:**
- High stiffness: `force_gains: [500.0, 50.0]` - For precise positioning tasks
- Medium compliance: `force_gains: [100.0, 10.0]` - For general manipulation tasks
- High compliance: `force_gains: [50.0, 5.0]` - For contact tasks and human interaction

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

- **Task File**: Located at `{robot_pkg}/config/ocs2/task.info`
- **URDF File**: Located at `{robot_pkg}/urdf/robot.urdf`
- **Generated Library**: Located at `{robot_pkg}/config/ocs2/generated`

The controller automatically loads these files based on the `robot_pkg` parameter. 