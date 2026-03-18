cmake_minimum_required(VERSION 3.14)
project(basic_joint_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        std_msgs
        arms_controller_common
        lina_planning
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# 添加 Eigen3 查找
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/BasicJointController.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
ament_target_dependencies(${PROJECT_NAME}
        ${CONTROLLER_INCLUDE_DEPENDS}
)

# lina_planning linking using keyword signature (after ament_target_dependencies which uses plain signature)
#if(lina_planning_FOUND)
#    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
#endif()

# 然后再链接 Eigen3（也使用 plain signature 保持一致）
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
)

pluginlib_export_plugin_description_file(controller_interface basic_joint_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
        OPTIONAL
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

