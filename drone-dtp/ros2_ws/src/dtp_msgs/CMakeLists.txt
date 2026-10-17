cmake_minimum_required(VERSION 3.8)
project(dtp_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# This macro is the actual code generator: it reads each .msg file below and emits both
# Python and C++ bindings at build time (so e.g. `from dtp_msgs.msg import Target` works
# from Python nodes even though this whole package is ament_cmake, not ament_python).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/TargetArray.msg"
  "msg/GoalStatus.msg"
  "msg/LossEvent.msg"
  "msg/VisibilityField.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
