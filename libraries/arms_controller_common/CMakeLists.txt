cmake_minimum_required(VERSION 3.8)
project(arms_controller_common)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        hardware_interface
        rclcpp
        pinocchio
        urdf
        ament_index_cpp
        tf2_geometry_msgs
        tf2
        geometry_msgs
        trajectory_msgs
)

find_package(ament_cmake REQUIRED)
# lina_planning is optional - DOUBLES interpolation requires it
find_package(lina_planning QUIET)
# Find Eigen3 (required by Pinocchio)
find_package(Eigen3 REQUIRED)

foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/FSM/StateHome.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
        src/utils/GravityCompensation.cpp
        src/utils/JointLimitsManager.cpp
        src/utils/JointTrajectoryManager.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)

# Conditionally link lina_planning if available
if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "lina_planning found - DOUBLES interpolation enabled")
else()
    message(STATUS "lina_planning not found - DOUBLES interpolation disabled (basic interpolation still available)")
endif()

ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${dependencies}
)

# Link Eigen3
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

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

ament_export_dependencies(${dependencies})
# Conditionally export lina_planning dependency
if(lina_planning_FOUND)
    ament_export_dependencies(lina_planning)
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

