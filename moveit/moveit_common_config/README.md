# MoveIt Common Config

This package contains common MoveIt configuration files for robotic arms, including:

- Common launch files for demo, gazebo simulation, and servo control
- Shared RViz configurations
- Common world files for simulation
- Shared configuration templates

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to moveit_common_config --symlink-install
```