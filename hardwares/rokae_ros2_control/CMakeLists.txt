cmake_minimum_required(VERSION 3.8)
project(rokae_ros2_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
pkg_check_modules(OROCOS_KDL REQUIRED orocos-kdl)

# Create library
add_library(${PROJECT_NAME} SHARED
        src/rokae_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE external/librokae/include
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
        PRIVATE ${OROCOS_KDL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        pluginlib
)

# Detect architecture and set library path
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ROKAE_LIB_DIR "aarch64")
else()
    set(ROKAE_LIB_DIR "x86_64")
endif()

# Link librokae libraries (v0.5.1)
target_link_libraries(${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/external/librokae/lib/Linux/${ROKAE_LIB_DIR}/libxCoreSDK.a
        ${CMAKE_CURRENT_SOURCE_DIR}/external/librokae/lib/Linux/${ROKAE_LIB_DIR}/libxMateModel.a
        ${EIGEN3_LIBRARIES}
        ${OROCOS_KDL_LIBRARIES}
        pthread
        rt
)

pluginlib_export_plugin_description_file(hardware_interface rokae_ros2_control.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION ib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
