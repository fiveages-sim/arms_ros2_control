# PINK Teleop

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to pink_teleop --symlink-install
```

## 2. Launch
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch pink_teleop pink_teleop.launch.py 
```