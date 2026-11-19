cmake_minimum_required(VERSION 3.14)
project(base_pose_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BASE_POSE_SIM_DEPENDS
        rclcpp
        geometry_msgs
        nav_msgs
        tf2_ros
        tf2_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${BASE_POSE_SIM_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_executable(base_pose_sim_node src/base_pose_sim_node.cpp)

target_include_directories(base_pose_sim_node
        PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_link_libraries(base_pose_sim_node
        ${rclcpp_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_msgs_TARGETS})

install(TARGETS base_pose_sim_node
        DESTINATION lib/${PROJECT_NAME})

install(FILES README.md
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
