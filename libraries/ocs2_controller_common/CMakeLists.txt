cmake_minimum_required(VERSION 3.14)
project(ocs2_controller_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_wheel_humanoid QUIET)
find_package(lina_planning QUIET)

add_library(${PROJECT_NAME} SHARED
    src/reference/PoseBasedReferenceManager.cpp
    src/visualization/Ocs2PinocchioVisualizer.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

# pinocchio is already transitively brought in PUBLIC by ocs2_pinocchio_interface.
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${arms_ros2_control_msgs_TARGETS}
    ${ocs2_core_TARGETS}
    ${ocs2_oc_TARGETS}
    ${ocs2_mpc_TARGETS}
    ${ocs2_pinocchio_interface_TARGETS}
    ${ocs2_self_collision_TARGETS}
    ${ocs2_self_collision_visualization_TARGETS}
    ${ocs2_ros_interfaces_TARGETS}
)

if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "ocs2_controller_common: lina_planning found (circle services enabled)")
else()
    message(STATUS "ocs2_controller_common: lina_planning not found (circle services disabled)")
endif()

if(ocs2_wheel_humanoid_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_OCS2_WHEEL_HUMANOID)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${ocs2_wheel_humanoid_TARGETS})
    message(STATUS "ocs2_controller_common: ocs2_wheel_humanoid found (wheel-humanoid coupling sync enabled)")
else()
    message(STATUS "ocs2_controller_common: ocs2_wheel_humanoid not found (wheel-humanoid coupling sync disabled)")
endif()

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

ament_export_dependencies(
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    nav_msgs
    visualization_msgs
    std_msgs
    tf2_ros
    tf2_geometry_msgs
    arms_ros2_control_msgs
    ocs2_core
    ocs2_oc
    ocs2_mpc
    ocs2_pinocchio_interface
    ocs2_self_collision
    ocs2_self_collision_visualization
    ocs2_ros_interfaces
)
if(lina_planning_FOUND)
    ament_export_dependencies(lina_planning)
endif()
if(ocs2_wheel_humanoid_FOUND)
    ament_export_dependencies(ocs2_wheel_humanoid)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
