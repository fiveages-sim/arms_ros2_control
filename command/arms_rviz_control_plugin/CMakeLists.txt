cmake_minimum_required(VERSION 3.5)
project(arms_rviz_control_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# 使用 qt5_wrap_cpp 宏生成MOC_FILES，用于处理Qt元对象编译
qt5_wrap_cpp(MOC_FILES
        include/arms_rviz_control_plugin/ocs2_fsm_panel.hpp
        include/arms_rviz_control_plugin/gripper_control_panel.hpp
)

# 设置源代码文件列表
set(SRC_FILES
        src/ocs2_fsm_panel.cpp
        src/gripper_control_panel.cpp
)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED ${MOC_FILES} ${SRC_FILES})

# 链接所需的库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  arms_ros2_control_msgs
)

# Export the plugin
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)
ament_export_dependencies(arms_ros2_control_msgs)

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
