cmake_minimum_required(VERSION 3.16)
project(arxlift2s_ros2_control)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Stanford arx5-sdk (arms) — HT-style external/, no conda
# ---------------------------------------------------------------------------
set(ARX5_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/arx5-sdk)
if(NOT EXISTS "${ARX5_SDK_DIR}/include/app/joint_controller.h")
  message(FATAL_ERROR
    "Stanford arx5-sdk not found at ${ARX5_SDK_DIR}.\n"
    "  ln -sfn <path-to>/arx-ros2-control/external/arx5-sdk "
    "${CMAKE_CURRENT_SOURCE_DIR}/external/arx5-sdk")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/aarch64)
else()
  set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/x86_64)
endif()

# Vendored SOEM 1.4.0 (replaces conda-forge::soem)
set(SOEM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/SOEM)
if(NOT EXISTS "${SOEM_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "SOEM not found at ${SOEM_DIR}. Clone v1.4.0:\n"
    "  git clone --depth 1 --branch v1.4.0 "
    "https://github.com/OpenEtherCATsociety/SOEM.git ${SOEM_DIR}")
endif()
set(BUILD_TESTS FALSE CACHE BOOL "SOEM tests" FORCE)
# Required: static libsoem linked into shared ArxJointController
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Soften vendored SOEM CMake (v1.4.0 enables -Werror; fails on modern GCC)
# Build SHARED so runtime matches Stanford libhardware.so (needs libsoem.so)
set(_soem_cmake ${SOEM_DIR}/CMakeLists.txt)
file(READ "${_soem_cmake}" _soem_cmake_content)
string(REPLACE "set(BUILD_TESTS TRUE)" "set(BUILD_TESTS FALSE)" _soem_cmake_content "${_soem_cmake_content}")
string(REPLACE " -Werror" " -Wno-error" _soem_cmake_content "${_soem_cmake_content}")
string(REPLACE "add_library(soem STATIC" "add_library(soem SHARED"
  _soem_cmake_content "${_soem_cmake_content}")
file(WRITE "${_soem_cmake}" "${_soem_cmake_content}")
add_subdirectory(${SOEM_DIR} ${CMAKE_CURRENT_BINARY_DIR}/soem EXCLUDE_FROM_ALL)
# SOEM is old C; silence pedantic-as-error on modern GCC
if(TARGET soem)
  set_target_properties(soem PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
  target_compile_options(soem PRIVATE
    -Wno-error
    -Wno-pedantic
    -Wno-error=pedantic
    -Wno-error=stringop-truncation
  )
endif()

add_library(ArxJointController SHARED
  ${ARX5_SDK_DIR}/src/app/joint_controller.cpp
  ${ARX5_SDK_DIR}/src/app/controller_base.cpp
  ${ARX5_SDK_DIR}/src/utils.cpp
)
target_compile_definitions(ArxJointController PUBLIC SDK_ROOT="${ARX5_SDK_DIR}")
target_include_directories(ArxJointController PUBLIC
  ${ARX5_SDK_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ArxJointController PUBLIC
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
  Eigen3::Eigen
  Threads::Threads
  spdlog::spdlog
  fmt::fmt
  kdl_parser::kdl_parser
  ${orocos_kdl_LIBRARIES}
  soem
  Boost::container
)
set_target_properties(ArxJointController PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  BUILD_RPATH "${ARX5_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

# ---------------------------------------------------------------------------
# Official lift SDK only (arm InterfacesThread retired)
# ---------------------------------------------------------------------------
set(ARX_LIFT_TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party/arx_lift_src)
set(ARX_LIFT_LIB ${ARX_LIFT_TP}/libarx_lift_src.so)
if(NOT EXISTS "${ARX_LIFT_LIB}")
  message(FATAL_ERROR "Official lift SDK missing: ${ARX_LIFT_LIB}")
endif()

# ---------------------------------------------------------------------------
# Hardware interface plugins
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/arx_x5_hardware.cpp
  src/arx_lift_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ARX5_SDK_DIR}/include>
  $<BUILD_INTERFACE:${ARX_LIFT_TP}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ArxJointController
  ${ARX_LIFT_LIB}
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ARX5_LIB_DIR};${ARX_LIFT_TP}"
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

pluginlib_export_plugin_description_file(hardware_interface arxlift2s_ros2_control.xml)

install(TARGETS ${PROJECT_NAME} ArxJointController
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ${ARX_LIFT_TP}/include/
  DESTINATION include
)

install(FILES
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
  ${ARX_LIFT_LIB}
  DESTINATION lib
)

# Install SOEM shared library (required at runtime by Stanford libhardware.so)
install(TARGETS soem
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(FILES arxlift2s_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
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
