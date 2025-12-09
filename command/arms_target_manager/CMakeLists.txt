cmake_minimum_required(VERSION 3.14)
project(arms_target_manager)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TARGET_MANAGER_INCLUDE_DEPENDS
        rclcpp
        geometry_msgs
        visualization_msgs
        interactive_markers
        arms_ros2_control_msgs
        arms_controller_common
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${TARGET_MANAGER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/ArmsTargetManager.cpp
        src/MarkerFactory.cpp
        src/ControlInputHandler.cpp
        src/VRInputHandler.cpp
        src/marker/HeadMarker.cpp
        src/marker/ArmMarker.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)

ament_target_dependencies(${PROJECT_NAME}
        ${TARGET_MANAGER_INCLUDE_DEPENDS}
)

add_executable(arms_target_manager_node src/arms_target_manager_node.cpp)
ament_target_dependencies(arms_target_manager_node ${TARGET_MANAGER_INCLUDE_DEPENDS})
target_link_libraries(arms_target_manager_node ${PROJECT_NAME})

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        TARGETS arms_target_manager_node
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

install(
        DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(${TARGET_MANAGER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
