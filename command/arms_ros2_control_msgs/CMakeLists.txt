cmake_minimum_required(VERSION 3.8)
project(arms_ros2_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
        "msg/CircleMessage.msg"
        "srv/ExecuteCircle.srv"
        "srv/ExecutePath.srv"
        DEPENDENCIES std_msgs geometry_msgs nav_msgs
)
# 添加export依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)

ament_package()
