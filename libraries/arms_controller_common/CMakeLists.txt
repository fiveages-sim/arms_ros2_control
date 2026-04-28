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

foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

# 获取 SDK 路径，兼容 arms_ros2_control 内置子模块和工作区顶层源码包两种布局。
set(_MARVIN_DIR_CANDIDATES
        "${CMAKE_CURRENT_SOURCE_DIR}/../../hardwares/marvin_ros2_control"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../marvin-ros2-control"
)

unset(MARVIN_DIR)
foreach(_MARVIN_DIR_CANDIDATE IN LISTS _MARVIN_DIR_CANDIDATES)
    get_filename_component(_MARVIN_DIR_ABS "${_MARVIN_DIR_CANDIDATE}" ABSOLUTE)
    if(EXISTS "${_MARVIN_DIR_ABS}/external/TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/FxRobot.h")
        set(MARVIN_DIR "${_MARVIN_DIR_ABS}")
        break()
    endif()
endforeach()

if(NOT MARVIN_DIR)
    message(FATAL_ERROR
            "Cannot find marvin_ros2_control SDK. Expected external/TJ_FX_ROBOT_CONTRL_SDK "
            "under one of: ${_MARVIN_DIR_CANDIDATES}")
endif()

set(TJ_SDK_BASE_DIR "${MARVIN_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK")
set(KINE_SDK_DIR "${TJ_SDK_BASE_DIR}/kinematicsSDK")

message(STATUS "KINE_SDK_DIR: ${KINE_SDK_DIR}")

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

# 添加 SDK 头文件路径。Kinematics.h 公开包含 FxRobot.h，构建期下游目标也需要该路径。
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
        "$<BUILD_INTERFACE:${KINE_SDK_DIR}>"
)

# 添加 SDK 库的链接目录
target_link_directories(${PROJECT_NAME} PRIVATE
        ${KINE_SDK_DIR}
)

if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "lina_planning found - DOUBLES interpolation enabled")
else()
    message(STATUS "lina_planning not found - DOUBLES interpolation disabled")
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})

# 只链接 libKine.so（运动学库）
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${KINE_SDK_DIR}/libKine.so
        pthread
        rt
)

# 设置 RPATH
set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${KINE_SDK_DIR}"
        INSTALL_RPATH "$ORIGIN"
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
install(FILES
        ${KINE_SDK_DIR}/libKine.so
        DESTINATION lib
)
if(EXISTS "${TJ_SDK_BASE_DIR}/ccs_m6.MvKDCfg")
    install(FILES
            ${TJ_SDK_BASE_DIR}/ccs_m6.MvKDCfg
            DESTINATION share/${PROJECT_NAME}/config
    )
endif()
# 安装 SDK 头文件到 arms_controller_common 的 include 目录
install(
        DIRECTORY ${KINE_SDK_DIR}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)
ament_export_dependencies(${dependencies})
if(lina_planning_FOUND)
    ament_export_dependencies(lina_planning)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
