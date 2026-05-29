# CAN ROS2 Control

This package provides `ros2_control` `SystemInterface` plugins for dexterous
hands over Linux SocketCAN.

## Supported Hands

- LinkerHand O6/L6: `can_ros2_control/O6CanHardware`
- Freedom: `can_ros2_control/FreedomCanHardware`

## LinkerHand O6/L6

Protocol:

- Right hand CAN ID: `0x27`
- Left hand CAN ID: `0x28`
- Command frame: `0x01` followed by 6 raw angle bytes
- Raw angle mapping: `255` is open/extended, `0` is closed/flexed

Example:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py type:=o6 hardware:=real_can can_interface:=can0 direction:=-1
```

## Freedom

Freedom uses CAN2.0 extended frames. The hand description resolves the default
device ID from `direction`: left hand `0`, right hand `1`.

Example:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real_can can_interface:=can0
```
