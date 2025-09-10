colcon build --packages-select unitree_g1_controller
source install/setup.bash
ros2 launch unitree_g1_controller unitree_g1_controller.launch.py