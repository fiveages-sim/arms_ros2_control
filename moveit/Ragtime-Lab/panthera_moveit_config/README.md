# Panthera Moveit Config

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to panthera_moveit_config --symlink-install
```

## 2. Moveit2
* Mock Components
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py robot:=panthera
    ```
* Gazebo
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=gz robot:=panthera
    ```
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=gz robot:=panthera type:=d405
    ```

## 3. Moveit Servo
* Mock Components
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config servo.launch.py robot:=panthera
    ```
* Gazebo
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config servo.launch.py hardware:=gz robot:=panthera
    ```
* Isaac Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch arx5_moveit_config servo.launch.py
    ```