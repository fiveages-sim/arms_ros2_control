# Panthera Moveit2 Config
This package provides MoveIt2 configuration for the Panthera robot, including mock components and Gazebo simulation support. The ros2 description is available at [Ragtime-Lab/panthera_description](https://github.com/fiveages-sim/robot_descriptions/tree/main/manipulator/Ragtime-Lab/panthera_description)

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
* Isaac Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=isaac robot:=panthera
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
  ros2 launch moveit_common_config servo.launch.py hardware:=isaac robot:=panthera
    ```