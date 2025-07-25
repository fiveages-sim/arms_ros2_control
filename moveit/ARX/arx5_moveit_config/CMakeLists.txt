cmake_minimum_required(VERSION 3.8)
project(arx5_moveit_config)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY config xacro
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()