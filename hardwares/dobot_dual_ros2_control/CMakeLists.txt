cmake_minimum_required(VERSION 3.16)
project(dobot_dual_ros2_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        sensor_msgs
        dobot_ros2_control
)

find_package(ament_cmake REQUIRED)

foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

## COMPILE
add_library(
        ${PROJECT_NAME}
        SHARED
        src/dobot_dual_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/dobot_dual_ros2_control>
)

# 使用ament_target_dependencies自动处理所有依赖(包括dobot_ros2_control)
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface dobot_dual_ros2_control.xml)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
