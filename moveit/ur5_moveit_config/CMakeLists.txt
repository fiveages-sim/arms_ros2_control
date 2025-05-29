cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
