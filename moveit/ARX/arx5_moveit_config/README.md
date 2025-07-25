# ARX R5 Moveit Config

## 1. Build
```bash
cd ~/ros2_ws
colcon build --packages-up-to arx5_moveit_config --symlink-install
```

## 2. Moveit2
* Mock Components
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py 
    ```
* Isaac Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch arx5_moveit_config isaac.launch.py
    ```
* Gazebo
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=gz
    ```
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=gz type:=r5
    ```

## 3. Moveit Servo
* Mock Components
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config servo.launch.py
    ```
* Gazebo
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config servo.launch.py hardware:=gz type:=r5
    ```
* Isaac Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch arx5_moveit_config servo.launch.py
    ```