cmake_minimum_required(VERSION 3.8)
project(arms_target_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(include)

# 创建库
add_library(${PROJECT_NAME} SHARED
  src/ArmsTargetManager.cpp
  src/ControlInputHandler.cpp
  src/VRInputHandler.cpp
)

# 链接库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
  arms_ros2_control_msgs
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# 创建可执行文件
add_executable(arms_target_manager_node
  src/arms_target_manager_node.cpp
)

# 链接库到可执行文件
ament_target_dependencies(arms_target_manager_node
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
  arms_ros2_control_msgs
)

target_link_libraries(arms_target_manager_node
  ${PROJECT_NAME}
)

# 安装
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS
  arms_target_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
  arms_ros2_control_msgs
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
