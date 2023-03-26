cmake_minimum_required(VERSION 3.8)
project(hexapod_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StepManagement.action"
  "action/StepAnimator.action"
  "msg/TargetAngles.msg"
  "msg/TargetPositions.msg"
  "msg/LegReferencing.msg"
  "msg/WaypointSetter.msg"
  "msg/GaitConstants.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
