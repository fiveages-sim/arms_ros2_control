cmake_minimum_required(VERSION 3.14)
project(basic_joint_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_hand_launch test/test_hand_launch.py)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        std_msgs
        tf2_ros
        tf2_geometry_msgs
        arms_controller_common
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# 添加 Eigen3 查找
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/BasicJointController.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${pluginlib_TARGETS}
        ${rcpputils_TARGETS}
        ${controller_interface_TARGETS}
        ${std_msgs_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        arms_controller_common::arms_controller_common
        Eigen3::Eigen
        pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(controller_interface basic_joint_controller.xml)

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
        OPTIONAL
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
