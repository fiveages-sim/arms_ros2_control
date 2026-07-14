cmake_minimum_required(VERSION 3.8)
project(arms_controller_common)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        hardware_interface
        rclcpp
        rclcpp_action
        pinocchio
        urdf
        ament_index_cpp
        tf2_geometry_msgs
        tf2
        geometry_msgs
        trajectory_msgs
        arms_ros2_control_msgs
)

find_package(ament_cmake REQUIRED)
find_package(lina_planning QUIET)
find_package(Eigen3 REQUIRED)

foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()


set(ARMS_CONTROLLER_COMMON_SOURCES
        src/FSM/StateHome.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
        src/FSM/StateCompliance.cpp
        src/utils/GravityCompensation.cpp
        src/utils/JointLimitsManager.cpp
        src/utils/JointTrajectoryManager.cpp
        src/utils/JointSpeedStopPlanner.cpp
        src/utils/WaistLiftingPlaner.cpp
        src/utils/Kinematics.cpp
)

if(lina_planning_FOUND)
    list(APPEND ARMS_CONTROLLER_COMMON_SOURCES
            src/utils/CartesianTrajectoryManager.cpp
    )
endif()

add_library(${PROJECT_NAME} SHARED
        ${ARMS_CONTROLLER_COMMON_SOURCES}
)

# Pinocchio headers (ROS Jazzy) can trigger -Wreorder in template instantiation paths.
# Suppress it only for this target to keep workspace-wide warnings strict.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-reorder)
endif()

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${rclcpp_TARGETS}
        pinocchio::pinocchio
        ${urdf_TARGETS}
        ament_index_cpp::ament_index_cpp
        ${tf2_geometry_msgs_TARGETS}
        ${tf2_TARGETS}
        ${geometry_msgs_TARGETS}
        ${trajectory_msgs_TARGETS}
        ${arms_ros2_control_msgs_TARGETS}
        Eigen3::Eigen
)

# Conditionally link lina_planning if available
if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "lina_planning found - DOUBLES interpolation enabled")
else()
    message(STATUS "lina_planning not found - DOUBLES interpolation disabled")
endif()

# 只链接 libKine.so（运动学库）
target_link_libraries(${PROJECT_NAME} PRIVATE
        pthread
        rt
)


install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
if(lina_planning_FOUND)
    ament_export_dependencies(lina_planning)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
