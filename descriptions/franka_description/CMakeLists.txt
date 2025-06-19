cmake_minimum_required(VERSION 3.10.2)
project(franka_description)
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY meshes launch urdf xacro config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()