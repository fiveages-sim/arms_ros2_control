cmake_minimum_required(VERSION 3.8)
project(arxlift2s_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Official closed-source SDK (vendored for standalone / hardwares submodule use)
set(ARX_X5_TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party/arx_x5_src)
set(ARX_LIFT_TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party/arx_lift_src)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ARX_X5_LIB ${ARX_X5_TP}/libarx_x5_src-arm64.so)
else()
  set(ARX_X5_LIB ${ARX_X5_TP}/libarx_x5_src.so)
endif()
set(ARX_LIFT_LIB ${ARX_LIFT_TP}/libarx_lift_src.so)

add_library(${PROJECT_NAME} SHARED
  src/arx_x5_hardware.cpp
  src/arx_lift_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ARX_X5_TP}/include>
  $<BUILD_INTERFACE:${ARX_LIFT_TP}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${ARX_X5_LIB}
  ${ARX_LIFT_LIB}
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Resolve vendored SDK .so next to the plugin at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ARX_X5_TP};${ARX_LIFT_TP}"
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

pluginlib_export_plugin_description_file(hardware_interface arxlift2s_ros2_control.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES arxlift2s_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

# Vendor SDK headers for dependents
install(DIRECTORY
  ${ARX_X5_TP}/include/
  DESTINATION include
)
install(DIRECTORY
  ${ARX_LIFT_TP}/include/
  DESTINATION include
)

install(FILES
  ${ARX_X5_LIB}
  ${ARX_LIFT_LIB}
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
