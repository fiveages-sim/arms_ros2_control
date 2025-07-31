# OCS2 Arm Controller

A ROS2 Control controller for arm control based on OCS2 (Optimal Control for Switched Systems).

## Overview

This controller implements a finite state machine (FSM) for arm control with the following states:

- **HOME**: Move arm to home position
- **ZERO**: Move arm to zero position

## Features

- Finite State Machine (FSM) implementation
- Position-based control for arm joints
- Configurable control parameters
- Simplified interface: position command and position state only

## States

### StateHome
- Moves arm to predefined home position
- Uses position control to reach target joint angles

### StateZero
- Moves arm to predefined zero position
- Uses position control to reach target joint angles

## Usage

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select ocs2_arm_controller --symlink-install
```

### Running

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py type:=AG2F90-C
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py hardware:=gz type:=AG2F120S
```

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py hardware:=isaac
```


### Configuration

The controller can be configured through the YAML configuration file located at:
`config/ocs2_arm_controller.yaml`

Key parameters:
- `joints`: List of joint names
- `home_pos`: Home position for each joint
- `zero_pos`: Zero position for each joint
- `update_rate`: Controller update rate in Hz

## Interface Configuration

The controller uses simplified interfaces:
- **Command Interface**: `position` only
- **State Interface**: `position` only

This simplifies the control architecture and focuses on position-based control for arm manipulation.

## State Transitions

The controller supports state transitions based on control input commands:

- **Command 1**: Transition to HOME state  
- **Command 2**: Transition to ZERO state

States can transition between each other based on the control input received on the `/control_input` topic.

## Future Enhancements

- Integration with OCS2 optimization framework
- Trajectory following capabilities
- Advanced control algorithms
- Safety features and limits
- Support for different arm configurations

## Dependencies

- ROS2 Control
- control_input_msgs
- std_msgs
- rclcpp
- pluginlib

## License

Apache-2.0 