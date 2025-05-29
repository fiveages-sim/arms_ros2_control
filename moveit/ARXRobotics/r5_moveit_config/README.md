# ARX R5 Moveit Config

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to r5_moveit_config --symlink-install
```

## 2. Moveit2
* Mock Components
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r5_moveit_config demo.launch.py 
```
* Isaac Sim
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r5_moveit_config isaac.launch.py
```

## 3. Moveit Servo
* Mock Components
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r5_moveit_config servo.launch.py ros2_control_hardware_type:=mock_components
```
* Isaac Sim
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch r5_moveit_config servo.launch.py
```