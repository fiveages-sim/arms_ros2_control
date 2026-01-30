cmake_minimum_required(VERSION 3.8)
project(arms_ros2_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
        "msg/CircleMessage.msg"
        DEPENDENCIES geometry_msgs
)

ament_package()
