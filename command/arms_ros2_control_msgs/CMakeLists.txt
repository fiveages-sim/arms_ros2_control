cmake_minimum_required(VERSION 3.8)
project(arms_ros2_control_msgs)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
        "msg/VRControllerState.msg"
)

ament_package()
