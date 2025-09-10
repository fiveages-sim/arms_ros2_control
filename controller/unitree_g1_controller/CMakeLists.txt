cmake_minimum_required(VERSION 3.8)
project(unitree_g1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create controller library as SHARED library (动态库)
add_library(unitree_g1_controller SHARED 
  src/MyRobotController.cpp
  src/SimpleController.cpp
)
ament_target_dependencies(unitree_g1_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
)

# Install controller library
install(TARGETS
  unitree_g1_controller
  DESTINATION lib/unitree_g1_controller
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install plugin description file
install(FILES
  my_robot_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Export plugin description file
pluginlib_export_plugin_description_file(controller_interface my_robot_controller_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()