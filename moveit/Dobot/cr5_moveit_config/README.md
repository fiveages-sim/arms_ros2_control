# Dobot CR5 Moveit Config

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to cr5_moveit_config --symlink-install
```

## 2. Moveit2

* Mock Components
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config demo.launch.py robot:=cr5
  ```
* Isaac Sim
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config demo.launch.py hardware:=isaac robot:=cr5
  ```
* Gazebo
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config demo.launch.py hardware:=gz robot:=cr5
  ```
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config demo.launch.py hardware:=gz robot:=cr5 world:=warehouse
  ```

## 3. Moveit Servo

* Mock Components
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py robot:=cr5 
  ```
* Isaac Sim
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py robot:=cr5 hardware:=isaac
  ```
* Gazebo
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py hardware:=gz robot:=cr5 world:=warehouse
  ```