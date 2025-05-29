cmake_minimum_required(VERSION 3.8)
project(gripper_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 添加可执行文件
add_executable(gripper_control src/GripperControl.cpp)
ament_target_dependencies(gripper_control rclcpp rclcpp_action control_msgs)

# 安装可执行文件
install(TARGETS
        gripper_control
        DESTINATION lib/${PROJECT_NAME})

# 安装launch文件夹下所有launch文件
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
