cmake_minimum_required(VERSION 3.14)
project(ocs2_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Only the packages whose headers are #included directly by this controller.
# Indirect deps propagate transitively:
#   - controller_interface brings hardware_interface, rclcpp_lifecycle,
#     rcl_interfaces, rclcpp, std_msgs and the full rosidl typesupport set.
#   - ocs2_mobile_manipulator brings ocs2_ddp -> ocs2_mpc -> ocs2_oc -> ocs2_core
#     and ocs2_self_collision -> ocs2_pinocchio_interface -> pinocchio.
#   - ocs2_controller_common brings ocs2_ros_interfaces (-> ocs2_msgs etc.).
set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        controller_interface
        ament_index_cpp
        ocs2_mobile_manipulator
        ocs2_controller_common
        arms_controller_common
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
        src/Ocs2ArmController.cpp
        src/control/CtrlComponent.cpp
        src/FSM/StateHome.cpp
        src/FSM/StateOCS2.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${pluginlib_TARGETS}
        ${controller_interface_TARGETS}
        ament_index_cpp::ament_index_cpp
        ${ocs2_mobile_manipulator_TARGETS}
        ${ocs2_controller_common_TARGETS}
        ${arms_controller_common_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface ocs2_arm_controller.xml)

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
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
