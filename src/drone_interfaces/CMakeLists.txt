cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannedTrajectory.msg"
  "msg/PlannerStatus.msg"
  "msg/MissionCommand.msg"
  "msg/MissionEvent.msg"
  DEPENDENCIES builtin_interfaces trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces trajectory_msgs)
ament_package()
