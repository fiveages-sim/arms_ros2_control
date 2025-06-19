# Dobot CR5 Description

This package contains the description files for Dobot CR5 Manipulator.

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to cr5_description --symlink-install
```

## 2. Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cr5_description visualize.launch.py
```

## 3. OCS2 Demo
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cr5_description ocs2.launch.py
```