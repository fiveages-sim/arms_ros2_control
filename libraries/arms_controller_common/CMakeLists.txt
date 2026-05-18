cmake_minimum_required(VERSION 3.8)
project(arms_controller_common)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)

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
find_package(hpp-fcl REQUIRED)
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()


add_library(${PROJECT_NAME} SHARED
        src/FSM/StateHome.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
        src/utils/GravityCompensation.cpp
        src/utils/JointLimitsManager.cpp
        src/utils/JointTrajectoryManager.cpp
        src/utils/WaistLiftingPlaner.cpp
        src/utils/Kinematics.cpp
        src/utils/CartesianTrajectoryManager.cpp
        src/utils/CollisionDetector.cpp
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

find_package(hpp-fcl REQUIRED)
if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "lina_planning found - DOUBLES interpolation enabled")
else()
    message(STATUS "lina_planning not found - DOUBLES interpolation disabled")
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})

target_compile_definitions(${PROJECT_NAME} PRIVATE PINOCCHIO_WITH_HPP_FCL)

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
