cmake_minimum_required(VERSION 3.18)  # for cmake_language(CALL ...)
project(arms_rviz_control_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(arms_controller_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)

# rviz2 is built against Qt5 on Jazzy and Qt6 on Kilted+ / Lyrical. A plugin
# compiled against the wrong Qt major still loads, but QObject vtable layout
# differs between Qt5 and Qt6, so the first virtual call into a panel SIGSEGV-s
# inside rviz2 (exit code -11). Discover the major version dynamically by
# probing Qt6 first (the newer / preferred one) and falling back to Qt5,
# matching the convention used by rviz_default_plugins.
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# qtN_wrap_cpp is exported by both Qt5 and Qt6 packages, but CMake does not
# allow variable expansion in a command name position, so dispatch via
# cmake_language(CALL ...) to keep the rest of this file version-agnostic.
cmake_language(CALL qt${QT_VERSION_MAJOR}_wrap_cpp MOC_FILES
        include/arms_rviz_control_plugin/ocs2_fsm_panel.hpp
        include/arms_rviz_control_plugin/gripper_control_panel.hpp
        include/arms_rviz_control_plugin/joint_control_panel.hpp
        include/arms_rviz_control_plugin/wbc_capability_panel.hpp
        include/arms_rviz_control_plugin/wbc_current_state_panel.hpp
        include/arms_rviz_control_plugin/switch_button.hpp
)

# 设置源代码文件列表
set(SRC_FILES
        src/ocs2_fsm_panel.cpp
        src/gripper_control_panel.cpp
        src/joint_control_panel.cpp
        src/wbc_capability_panel.cpp
        src/wbc_current_state_panel.cpp
        src/switch_button.cpp
)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED ${MOC_FILES} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt${QT_VERSION_MAJOR}::Widgets
  ${rclcpp_TARGETS}
  rviz_common::rviz_common
  arms_controller_common::arms_controller_common
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${arms_ros2_control_msgs_TARGETS}
)

# Export the plugin
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION include
)

# 使用宏 pluginlib_export_plugin_description_file 导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
