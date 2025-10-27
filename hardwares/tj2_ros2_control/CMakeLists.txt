cmake_minimum_required(VERSION 3.8)
project(tj2_ros2_control)

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

set(HEADER_KINE_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/")
set(HEADER_CONTROL_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/")
set(LIB_KINE_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/kinematicsSDK/")
set(LIB_CONTROL_DIR "${CMAKE_SOURCE_DIR}/external/TJ_FX_ROBOT_CONTRL_SDK/contrlSDK/")

# Link to the existing .so library (replace with your actual .so file name and path)
find_library(EXTERNAL_KINEMATICSO NAMES "libKine.so" PATHS ${LIB_KINE_DIR} NO_DEFAULT_PATH)
# Link to the existing .so library (replace with your actual .so file name and path)
find_library(EXTERNAL_CONTROLSO NAMES "libMarvinSDK.so" PATHS ${LIB_CONTROL_DIR} NO_DEFAULT_PATH)


# Create library
add_library(${PROJECT_NAME} SHARED
        src/tj2_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${HEADER_KINE_DIR}>"
        "$<BUILD_INTERFACE:${HEADER_CONTROL_DIR}>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        pluginlib
)

# # Detect architecture and set library path
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
#     set(XCORE_LIB_DIR "aarch64")
# else()
#     set(XCORE_LIB_DIR "x86_64")
# endif()

# # Link xCoreSDK libraries
# target_link_libraries(${PROJECT_NAME}
#         #${CMAKE_CURRENT_SOURCE_DIR}/external/xCoreSDK/lib/Linux/${XCORE_LIB_DIR}/libxCoreSDK.a
#         #${CMAKE_CURRENT_SOURCE_DIR}/external/xCoreSDK/lib/Linux/${XCORE_LIB_DIR}/libxMateModel.a
#         ${EIGEN3_LIBRARIES}
#         pthread
#         rt
# )

pluginlib_export_plugin_description_file(hardware_interface tj2_ros2_control.xml)

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

# if (BUILD_TESTING)
#     find_package(ament_lint_auto REQUIRED)
#     ament_lint_auto_find_test_dependencies()
# endif ()

# ament_export_dependencies(${dependencies})
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ament_package()
