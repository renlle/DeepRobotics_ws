cmake_minimum_required(VERSION 3.8)
project(master_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeviceInfo.msg"
  "msg/NavigationTelemetry.msg"
  "msg/NavigationTaskCommand.msg"
  "msg/NavigationTaskStatus.msg"
  "msg/WaypointReachStatus.msg"
  "msg/WaypointReachStatusArray.msg"
  "srv/NavigateToPoint.srv"
  "srv/NavigateToPoints.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
