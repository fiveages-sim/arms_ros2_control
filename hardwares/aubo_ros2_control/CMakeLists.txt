cmake_minimum_required(VERSION 3.16)
project(aubo_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(_aubo_sdk_package_root_candidates)

if(DEFINED ENV{AUBO_SDK_PACKAGE_ROOT} AND NOT "$ENV{AUBO_SDK_PACKAGE_ROOT}" STREQUAL "")
  list(APPEND _aubo_sdk_package_root_candidates "$ENV{AUBO_SDK_PACKAGE_ROOT}")
endif()

list(APPEND _aubo_sdk_package_root_candidates
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../aubo"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../AUBO/SDK应用开发包"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../evol/src/AUBO/SDK应用开发包"
)

if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
  list(APPEND _aubo_sdk_package_root_candidates "$ENV{HOME}/evol/src/AUBO/SDK应用开发包")
endif()

set(AUBO_RUNTIME_ROOT "")
set(AUBO_LAYOUT "")
foreach(_candidate IN LISTS _aubo_sdk_package_root_candidates)
  if(_candidate STREQUAL "")
    continue()
  endif()

  get_filename_component(_candidate_abs "${_candidate}" ABSOLUTE)
  if(
    EXISTS "${_candidate_abs}/include/serviceinterface.h"
    AND EXISTS "${_candidate_abs}/lib/libauborobotcontroller.so.1.3.2"
    AND EXISTS "${_candidate_abs}/lib/liblog4cplus-1.2.so.5"
  )
    set(AUBO_RUNTIME_ROOT "${_candidate_abs}")
    set(AUBO_LAYOUT "flat")
    break()
  endif()

  if(EXISTS "${_candidate_abs}/C++/AuboRobotSdkExample/dependents/robotSDK/inc/serviceinterface.h")
    set(AUBO_RUNTIME_ROOT "${_candidate_abs}")
    set(AUBO_LAYOUT "legacy")
    break()
  endif()
endforeach()

if(AUBO_RUNTIME_ROOT STREQUAL "")
  message(FATAL_ERROR
    "AUBO SDK package root not found. Set AUBO_SDK_PACKAGE_ROOT to the directory containing "
    "either include/ + lib/ or C++/AuboRobotSdkExample + python/linux/python3.x.")
endif()

if(AUBO_LAYOUT STREQUAL "flat")
  set(AUBO_SDK_INCLUDE_DIR "${AUBO_RUNTIME_ROOT}/include")
  set(AUBO_SDK_LIB_DIR "${AUBO_RUNTIME_ROOT}/lib")
  set(AUBO_RUNTIME_LIB_DIR "${AUBO_RUNTIME_ROOT}/lib")
  set(AUBO_LOG4CPLUS_SHARED_LIB "${AUBO_RUNTIME_ROOT}/lib/liblog4cplus-1.2.so.5")
  set(AUBO_SDK_SHARED_LIB "${AUBO_RUNTIME_ROOT}/lib/libauborobotcontroller.so.1.3.2")
elseif(AUBO_LAYOUT STREQUAL "legacy")
  set(AUBO_SDK_ROOT
    "${AUBO_RUNTIME_ROOT}/C++/AuboRobotSdkExample/dependents/robotSDK")
  set(AUBO_SDK_INCLUDE_DIR "${AUBO_SDK_ROOT}/inc")

  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(AUBO_SDK_LIB_DIR "${AUBO_SDK_ROOT}/lib/linux_x64")
    set(AUBO_RUNTIME_LIB_DIR "${AUBO_RUNTIME_ROOT}/python/linux/python3.x")
    set(AUBO_LOG4CPLUS_SHARED_LIB
      "${AUBO_RUNTIME_ROOT}/python/linux/python3.x/liblog4cplus-1.2.so.5")
  else()
    set(AUBO_SDK_LIB_DIR "${AUBO_SDK_ROOT}/lib/linux_x32")
    message(FATAL_ERROR "32-bit AUBO runtime dependencies are not configured for this package.")
  endif()

  set(AUBO_SDK_SHARED_LIB "${AUBO_SDK_LIB_DIR}/libauborobotcontroller.so.1.3.2")
else()
  message(FATAL_ERROR "Unsupported AUBO SDK layout at ${AUBO_RUNTIME_ROOT}")
endif()

if(NOT EXISTS "${AUBO_SDK_INCLUDE_DIR}/serviceinterface.h")
  message(FATAL_ERROR "AUBO SDK headers not found at ${AUBO_SDK_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${AUBO_SDK_SHARED_LIB}")
  message(FATAL_ERROR "AUBO SDK shared library not found at ${AUBO_SDK_SHARED_LIB}")
endif()

if(NOT EXISTS "${AUBO_LOG4CPLUS_SHARED_LIB}")
  message(FATAL_ERROR "AUBO log4cplus runtime not found at ${AUBO_LOG4CPLUS_SHARED_LIB}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/aubo_commander.cpp
  src/aubo_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${AUBO_SDK_INCLUDE_DIR})

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--no-as-needed")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${AUBO_SDK_SHARED_LIB}
    ${AUBO_LOG4CPLUS_SHARED_LIB}
    Threads::Threads
    dl
    m
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${AUBO_SDK_LIB_DIR};${AUBO_RUNTIME_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN"
)

add_executable(bluedot_force_sensor_node
  src/bluedot_force_sensor.cpp
  src/bluedot_force_sensor_node.cpp
)

target_compile_features(bluedot_force_sensor_node PUBLIC cxx_std_17)
target_include_directories(bluedot_force_sensor_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(bluedot_force_sensor_node PUBLIC geometry_msgs rclcpp)

pluginlib_export_plugin_description_file(hardware_interface aubo_ros2_control.xml)

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

install(
  FILES
    "${AUBO_SDK_SHARED_LIB}"
    "${AUBO_LOG4CPLUS_SHARED_LIB}"
  DESTINATION lib
)

install(CODE "
  execute_process(
    COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
            libauborobotcontroller.so.1.3.2
            \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libauborobotcontroller.so.1\")
")

install(
  TARGETS bluedot_force_sensor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES aubo_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
