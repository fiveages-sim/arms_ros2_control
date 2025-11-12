cmake_minimum_required(VERSION 3.16)
project(eyou_ros2_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

## COMPILE
add_library(
        ${PROJECT_NAME}
        SHARED
        src/eyou_hardware.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external/libeyou/include
)

ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 检测架构并设置库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(EYOU_LIB_DIR "aarch64")
else()
    set(EYOU_LIB_DIR "x86_64")
endif()

# 设置库文件路径
set(EYOU_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/libeyou/lib/Linux/${EYOU_LIB_DIR})

# 设置RPATH，让运行时能找到库文件
# 将所有库安装到 lib/ 目录，使用 $ORIGIN 表示相对于插件库的位置
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${EYOU_LIB_PATH}"
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH FALSE
)

# 链接Eyou SDK库
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${EYOU_LIB_PATH}/libeu_harmonic.so
        ${EYOU_LIB_PATH}/libcontrolcan.so
        ${EYOU_LIB_PATH}/libcyhcs_log.so
        ${EYOU_LIB_PATH}/libeu_canable.so
        ${EYOU_LIB_PATH}/libeu_candrv.so
        ${EYOU_LIB_PATH}/libusbcanfd.so
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface eyou_ros2_control.xml)

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

# 安装Eyou SDK库文件到install目录（与主库同一目录，方便RPATH查找）
install(
        FILES
                ${EYOU_LIB_PATH}/libeu_harmonic.so
                ${EYOU_LIB_PATH}/libcontrolcan.so
                ${EYOU_LIB_PATH}/libcyhcs_log.so
                ${EYOU_LIB_PATH}/libeu_canable.so
                ${EYOU_LIB_PATH}/libeu_candrv.so
                ${EYOU_LIB_PATH}/libusbcanfd.so
        DESTINATION lib
        OPTIONAL
)

# 安装后设置依赖库的RPATH，使它们能找到同目录下的其他库
# 使用 install(CODE ...) 在安装后运行命令
find_program(PATCHELF patchelf)
if(PATCHELF)
    # 在 install(CODE ...) 中需要使用 @PATCHELF@ 来传递变量
    install(CODE "
        message(STATUS \"Setting RPATH for Eyou SDK libraries...\")
        set(LIB_DIR \"\${CMAKE_INSTALL_PREFIX}/lib\")
        set(PATCHELF_PATH \"${PATCHELF}\")
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libeu_harmonic.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libcontrolcan.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libcyhcs_log.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libeu_canable.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libeu_candrv.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
        execute_process(
            COMMAND \"\${PATCHELF_PATH}\" --set-rpath \"\\$ORIGIN\" \"\${LIB_DIR}/libusbcanfd.so\"
            RESULT_VARIABLE RESULT
            ERROR_QUIET
        )
    ")
else()
    message(WARNING "patchelf not found. Eyou SDK libraries may need LD_LIBRARY_PATH set to find dependencies.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

