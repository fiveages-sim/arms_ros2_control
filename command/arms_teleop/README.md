# Arms Teleop Package

This package provides joystick teleoperation interface for robotic arms.

## Features

- **Joystick Teleop**: Support for various joystick controllers (Xbox, PlayStation, etc.)
- **Unified Interface**: Publishes to the same `/control_input` topic
- **Configurable**: Easy to configure sensitivity and control mappings

## Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_teleop --symlink-install
```

## Usage
### Joystick Teleop

Launch joystick teleop:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
```

Or with custom joystick device:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

## Control Mappings
- **Left Stick**: Control arm movement
- **Right Stick**: Control end-effector orientation
- **LB + B**: Command mode 1
- **RB + A**: Command mode 2
- **Triggers**: Gripper control

## Topics

### Published Topics
- `/control_input` (arms_ros2_control_msgs/msg/Inputs): Control input commands

### Subscribed Topics
- `/joy` (sensor_msgs/msg/Joy): Joystick input

## Parameters

- `joy_dev` (string, default: "/dev/input/js0"): Joystick device file

## Dependencies

- `rclcpp`: ROS2 C++ client library
- `sensor_msgs`: Sensor message types
- `arms_ros2_control_msgs`: Control input message definitions 