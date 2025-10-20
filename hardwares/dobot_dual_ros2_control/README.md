# Dobot Dual ROS2 Control

ROS2 Control Hardware Interface for controlling two independent Dobot CR series robots simultaneously via TCP/IP.

## Overview

This package provides a `SystemInterface` implementation that manages two Dobot CR robots with independent IP addresses, enabling synchronized dual-arm control through the ROS2 Control framework.

## Features

- **Dual TCP Connections**: Independent connections to two Dobot CR robots
- **12-DOF Control**: Simultaneous control of 6 joints per arm (12 total)
- **Dual Gripper Support**: Optional gripper control for both arms via Modbus
- **Direct ServoJ Commands**: Low-latency joint position streaming
- **Configurable Parameters**: Independent gain, speed, and timing settings per arm

## Hardware Requirements

- 2x Dobot CR series robots (CR3, CR5, CR7, CR10, CR12, CR16)
- Network connection to both robots
- Different IP addresses configured for each robot

## Configuration

### Default IP Addresses

- **Left Arm**: `192.168.5.38`
- **Right Arm**: `192.168.5.39`

### Changing IP Addresses

Edit the XACRO file at:
```
cr5_dual_description/xacro/ros2_control/robot.xacro
```

Modify the parameters:
```xml
<param name="left_robot_ip">192.168.5.38</param>
<param name="right_robot_ip">192.168.5.39</param>
```

### Tuning Parameters

**Per-Arm Parameters:**
- `left_gain` / `right_gain`: ServoJ proportional gain (default: 800)
- `left_aheadtime` / `right_aheadtime`: Feed-forward time (default: 50ms)
- `left_speed_factor` / `right_speed_factor`: Global speed limit 1-100% (default: 100)
- `left_servo_time` / `right_servo_time`: ServoJ execution time (default: 0.5s)

**Global Parameters:**
- `verbose`: Enable detailed logging and frequency statistics (default: false)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to dobot_dual_ros2_control cr5_dual_description --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Usage

### Prerequisites

1. Ensure both Dobot robots are powered on
2. Configure the correct IP addresses on each robot
3. Verify network connectivity:
   ```bash
   ping 192.168.5.38
   ping 192.168.5.39
   ```

### Launch Dual Arm Control

**With Mock Hardware (Testing):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=mock_components type:=AG2F90-C-Soft
```

**With Real Hardware:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=real type:=AG2F90-C-Soft
```

**With Gazebo Simulation:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5_dual hardware:=gz type:=AG2F90-C-Soft
```

### Monitoring

**Check controller status:**
```bash
ros2 control list_controllers
```

**View joint states:**
```bash
ros2 topic echo /joint_states
```

**Check hardware status:**
```bash
ros2 control list_hardware_interfaces
```

## Architecture

### Joint Mapping

| Index | Joint Name     | Robot |
|-------|----------------|-------|
| 0-5   | left_joint1-6  | Left  |
| 6-11  | right_joint1-6 | Right |

### Hardware Interface

The `DobotDualHardware` class:
- Inherits from `hardware_interface::SystemInterface`
- Manages two `CRCommanderRos2` instances (one per robot)
- Implements independent read/write cycles for each arm
- Handles gripper control via separate threads

### Control Flow

```
OCS2 Arm Controller
    ↓ (joint commands)
DobotDualHardware::write()
    ├─→ left_commander_->servoJ(...)  → Left Robot (192.168.5.38)
    └─→ right_commander_->servoJ(...) → Right Robot (192.168.5.39)
```

## Gripper Control

Grippers are automatically detected based on joint names containing:
- `left_gripper` or `left_hand` → Left gripper
- `right_gripper` or `right_hand` → Right gripper

Gripper position range: `0.0` (closed) to `1.0` (open)

Modbus configuration:
- Baud rate: 1000K
- Register 0x03E8: Position control
- Update rate: 10Hz

## Troubleshooting

### Connection Timeout

**Error:** `Timeout waiting for LEFT/RIGHT arm connection!`

**Solutions:**
1. Verify robot is powered and in ready state
2. Check network connectivity: `ping <robot_ip>`
3. Ensure firewall allows TCP connections
4. Verify IP address configuration

### ServoJ Command Failure

**Error:** `Failed to send ServoJ command`

**Solutions:**
1. Check if robot is in remote control mode
2. Reduce `servo_time` if commands are too slow
3. Increase `gain` for more responsive tracking
4. Enable `verbose: true` for detailed diagnostics

### Asymmetric Behavior

If one arm behaves differently:
1. Verify both arms have same firmware version
2. Check individual parameter settings (gain, aheadtime)
3. Test each arm independently with single-arm hardware interface
4. Ensure network latency is similar for both connections

## Safety Notes

- Always test with mock hardware first
- Use reduced `speed_factor` for initial testing
- Keep emergency stop accessible
- Monitor joint limits and singularities
- Both robots should have clear working space

## Dependencies

- `dobot_ros2_control`: Single-arm hardware interface (for CRCommanderRos2)
- `hardware_interface`: ROS2 Control base classes
- `rclcpp`: ROS2 C++ client library
- `pluginlib`: Plugin system

## License

Apache License 2.0

## Author

Dobot Team
