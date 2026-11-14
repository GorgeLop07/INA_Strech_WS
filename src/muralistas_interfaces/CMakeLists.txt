cmake_minimum_required(VERSION 3.8)
project(muralistas_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwitchEvent.msg"
  "msg/ScanState.msg"
  "msg/StrokeRequest.msg"
  "msg/Board.msg"
  "msg/Decision.msg"
  "srv/SetBrushColor.srv"
  "srv/CalibrateTool.srv"
  "action/DrawStroke.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
