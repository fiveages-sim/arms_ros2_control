cmake_minimum_required(VERSION 3.8)
project(arms_teleop)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Joystick teleop node
add_executable(joystick_teleop src/joystick_teleop.cpp)
target_include_directories(joystick_teleop
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        joystick_teleop PUBLIC
        rclcpp
        sensor_msgs
        std_msgs
        arms_ros2_control_msgs
        control_msgs
        rclcpp_action
)

# Keyboard teleop node
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_include_directories(keyboard_teleop
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        keyboard_teleop PUBLIC
        rclcpp
        std_msgs
        arms_ros2_control_msgs
)

install(TARGETS
        joystick_teleop
        keyboard_teleop
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

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