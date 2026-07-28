# ROS2 Control Hardware

## 1. ARX LIFT2S ROS2 Control (Stanford arms + official lift)
```bash
cd ~/ros2_ws
# Requires external/arx5-sdk symlink + external/SOEM (see arxlift2s_ros2_control/README.md)
colcon build --packages-up-to arxlift2s_ros2_control --symlink-install
```

## 2. Topic Based ROS2 Control
```bash
cd ~/ros2_ws
colcon build --packages-up-to topic_based_ros2_control --symlink-install
```

## 3. Unitree ROS2 Control
```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_ros2_control --symlink-install
```

## 4. Dobot ROS2 Control
```bash
cd ~/ros2_ws
colcon build --packages-up-to dobot_ros2_control --symlink-install
```

## 5. Rokae ROS2 Control
```bash
cd ~/ros2_ws
colcon build --packages-up-to rokae_ros2_control --symlink-install
```
