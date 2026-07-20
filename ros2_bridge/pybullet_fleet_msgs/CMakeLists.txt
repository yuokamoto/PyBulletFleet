cmake_minimum_required(VERSION 3.8)
project(pybullet_fleet_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandAck.msg"
  "msg/ExecuteActionGoal.msg"
  "msg/FleetAttach.msg"
  "msg/FleetExecuteAction.msg"
  "msg/FleetJointCommand.msg"
  "msg/FleetNavigate.msg"
  "msg/FleetState.msg"
  "msg/FleetStop.msg"
  "msg/RobotActionCommand.msg"
  "msg/RobotAttachCommand.msg"
  "msg/RobotGoal2D.msg"
  "msg/RobotGoal3D.msg"
  "msg/RobotJointPositionsCommand.msg"
  "msg/RobotNamedJointPositionsCommand.msg"
  "msg/RobotState3D.msg"
  "srv/AttachObject.srv"
  "srv/FleetAttach.srv"
  "srv/FleetExecuteAction.srv"
  "srv/FleetJointCommand.srv"
  "srv/FleetNavigate.srv"
  "srv/FleetStop.srv"
  "action/ExecuteAction.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
