# Unitree ROS2 Control

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to
control the Unitree robot in Mujoco simulator or real Unitree Robots supported by sdk2.

For quadruped robot simulation, please use mujoco simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add
foot force sensor support.

## 1. Interfaces
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

* Ubuntu 24.04 ROS2 Jazzy

```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_ros2_control --symlink-install
```

## 3. Deploy

Since the real unitree robot has different network and domain name, you need to set the network and domain name in the
xacro file. Complete `ROS2-Control` xacro could be found at [Unitree G1 Description](https://github.com/fiveages-sim/robot_descriptions/tree/main/humanoid/Unitree/unitree_g1_description).

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
  ros2 launch ocs2_arm_controller demo.launch.py robot:=unitree_g1 hardware:=unitree_sim
  ```
* Real Unitree G1
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=unitree_g1 type:=revo2 hardware:=unitree_real
  ```

https://github.com/user-attachments/assets/54511d3c-b88c-48aa-b4fe-45aee4746d9e

