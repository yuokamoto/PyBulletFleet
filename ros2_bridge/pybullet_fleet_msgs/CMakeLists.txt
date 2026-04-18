cmake_minimum_required(VERSION 3.8)
project(pybullet_fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Auto-collect all .msg, .srv, .action files
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")
set(INTERFACE_FILES ${MSG_FILES} ${SRV_FILES} ${ACTION_FILES})

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
