# ROS2 Control Hardware

## ARX LIFT2S ROS2 Control (official SDK)
```bash
cd ~/ros2_ws
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