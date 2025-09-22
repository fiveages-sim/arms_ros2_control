# Arms Teleop Package

This package provides unified teleoperation interfaces for robotic arms, supporting both joystick and keyboard input methods.

## Features

- **Joystick Teleop**: Support for various joystick controllers (Xbox, PlayStation, etc.)
- **Keyboard Teleop**: Keyboard-based control for robotic arms
- **Unified Interface**: Both methods publish to the same `/control_input` topic
- **Configurable**: Easy to configure sensitivity and control mappings

## Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_teleop --symlink-install
```

## Usage

source ~/XZN/integrate/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py type:=AG2F90-C

source ~/XZN/integrate/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=agibot_g1

### Joystick Teleop

Launch joystick teleop:
```bash
source ~/XZN/integrate/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
```

Or with custom joystick device:
```bash
source ~/XZN/integrate/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

### Keyboard Teleop

Launch keyboard teleop:
```bash
source ~/XZN/integrate/install/setup.bash
ros2 run arms_teleop keyboard_teleop
```

## Control Mappings

### Joystick Controls
- **Left Stick**: Control arm movement
- **Right Stick**: Control end-effector orientation
- **LB + B**: Command mode 1
- **RB + A**: Command mode 2
- **Triggers**: Gripper control

### Keyboard Controls
- **WASD**: Control arm movement
- **Arrow Keys**: Control end-effector orientation
- **1-9**: Different command modes
- **Space**: Toggle gripper open/close
- **Q**: Quit

## Topics

### Published Topics
- `/control_input` (arms_ros2_control_msgs/msg/Inputs): Control input commands

### Subscribed Topics
- `/joy` (sensor_msgs/msg/Joy): Joystick input (joystick_teleop only)

## Parameters

### Joystick Teleop Parameters
- `joy_dev` (string, default: "/dev/input/js0"): Joystick device file

### Keyboard Teleop Parameters
- `sensitivity_left` (float, default: 0.05): Left stick sensitivity
- `sensitivity_right` (float, default: 0.05): Right stick sensitivity

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `sensor_msgs`: Sensor message types
- `arms_ros2_control_msgs`: Control input message definitions 