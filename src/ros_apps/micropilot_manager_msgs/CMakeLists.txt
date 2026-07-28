cmake_minimum_required(VERSION 3.8)
project(micropilot_manager_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LeafState.msg"
  "msg/SystemState.msg"
  "msg/TransitionEvent.msg"
  "msg/HealthReport.msg"
  "msg/Waypoint.msg"
  "msg/MissionStatus.msg"
  "msg/VehicleTelemetry.msg"
  "msg/TeleoperationCommand.msg"
  "msg/OperatorSession.msg"
  "msg/StreamStatus.msg"
  "msg/ComponentHealth.msg"
  "msg/SARCheckpoint.msg"
  "msg/RiskLevel.msg"
  "msg/NetworkStatus.msg"
  "msg/SafetyRuntimeData.msg"
  "msg/ScenarioContext.msg"
  "srv/FsmTransition.srv"
  "srv/NodeGroupCommand.srv"
  "srv/NodeProcessCommand.srv"
  "srv/SetMission.srv"
  "srv/MissionCommand.srv"
  "srv/MissionHistory.srv"
  "srv/FaultBehaviour.srv"
  "srv/UrbanMissionCommand.srv"
  "srv/FsmCommand.srv"
  "srv/StreamCommand.srv"
  "srv/MissionSetup.srv"
  "srv/MissionAction.srv"
  "srv/MissionHistoryQuery.srv"
  "srv/SessionControl.srv"
  DEPENDENCIES builtin_interfaces nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
