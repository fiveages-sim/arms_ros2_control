cmake_minimum_required(VERSION 3.16)
project(arx_x5_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=return-type)
  # 注意：移除了部分-Werror选项，因为arx5-sdk头文件有相关警告
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============== 依赖 ==============
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# arx5-sdk 依赖
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(spdlog REQUIRED)

# ============== ARX5 SDK ==============
set(ARX5_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/arx5-sdk)

# 定义SDK_ROOT宏，SDK需要它来找URDF等资源文件
add_compile_definitions(SDK_ROOT="${ARX5_SDK_DIR}")

# SDK头文件
include_directories(
  include
  ${ARX5_SDK_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/$ENV{ROS_DISTRO}/include/kdl_parser
)

# SDK库文件路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/aarch64)
else()
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/x86_64)
endif()

# ============== Hardware Interface 库 ==============
add_library(${PROJECT_NAME} SHARED
  src/arx_x5_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 链接ARX5 SDK库
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ARX5_SDK_DIR}/build/libArxJointController.so
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
)

# ============== 导出插件 ==============
pluginlib_export_plugin_description_file(hardware_interface arx_x5_ros2_control.xml)

# ============== 设置 RPATH ==============
# 确保运行时能找到 SDK 库
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# ============== 安装 ==============
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

# 安装 ARX5 SDK 库到 lib 目录
install(
  FILES
    ${ARX5_SDK_DIR}/build/libArxJointController.so
    ${ARX5_LIB_DIR}/libhardware.so
    ${ARX5_LIB_DIR}/libsolver.so
  DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
