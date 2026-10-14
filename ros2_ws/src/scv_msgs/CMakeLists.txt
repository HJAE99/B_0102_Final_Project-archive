cmake_minimum_required(VERSION 3.8)
project(scv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NetRaw.msg"
  "msg/NetSample.msg"
  "msg/RobotStatus.msg"
  "msg/Candidate.msg"
  "msg/InspectionResult.msg"
  "msg/Alert.msg"
  "srv/SyncSamples.srv"
  "srv/Reassign.srv"
  "srv/InspectFacility.srv"
  "action/ExecuteTask.action"
  "action/PreciseMeasure.action"
  "action/InspectFacility.action"   # IF-05 옛 액션 — 10/14 서비스로 바뀜, T2-6 통합 때 mission_executor 를 바꾸고 지운다
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
