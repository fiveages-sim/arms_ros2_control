# Unitree ROS2 Control

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to control the Unitree robot in Mujoco simulator or real Unitree Robots supported by sdk2. 

Please use mujoco simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add foot force sensor support.

## 1. Interfaces

Required hardware interfaces:

* command:
  * joint position
  * joint velocity
  * joint effort
  * KP
  * KD
* state:
  * joint effort
  * joint position
  * joint velocity
  * imu sensor
    * linear acceleration
    * angular velocity
    * orientation
  * foot force sensor

## 2. Build

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_ros2_control --symlink-install
```

## 3. Config network and domain
Since the real unitree robot has different network and domain name, you need to set the network and domain name in the xacro file.
```xml
<hardware>
    <plugin>hardware_unitree_sdk2/HardwareUnitree</plugin>
    <param name="domain">1</param>
    <param name="network_interface">lo</param>
</hardware>
```

* Unitree Mujoco
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_ros2_control visualize.launch.py
  ```
* Real Unitree G1
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_ros2_control visualize.launch.py hardware:=unitree_real
  ```


https://github.com/user-attachments/assets/54511d3c-b88c-48aa-b4fe-45aee4746d9e

