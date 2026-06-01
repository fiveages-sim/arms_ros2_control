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

Bring up the SocketCAN interface before launching ROS2 control:

```bash
source ~/ros2_ws/install/setup.bash
ip -details link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

```

Left hand (`direction:=1`, default, CAN ID `0x28`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=linkerhand type:=o6 hardware:=real_can
```

Right hand (`direction:=-1`, CAN ID `0x27`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=linkerhand type:=o6 hardware:=real_can direction:=-1
```

## Freedom

Freedom uses CAN2.0 extended frames. The hand description resolves the default
device ID from `direction`: left hand `0`, right hand `1`.

Example:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller hand.launch.py hand:=freedom type:=freedom hardware:=real_can
```
