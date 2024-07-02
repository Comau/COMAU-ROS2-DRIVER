# Copyright 2024 Comau Robotics S.p.A.

cmake_minimum_required(VERSION 3.8)
project(comau_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)

## Generate added messages and services with any dependencies listed here
# find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ActionFeedback.msg"
  "msg/ActionFeedbackStatusConstants.msg"
  "msg/ActionRequest.msg"
  "msg/ActionResult.msg"
  "msg/ActionResultStatusConstants.msg"
  "msg/CartesianPoseStamped.msg"
  "msg/ComauOperationMode.msg"
  "msg/ComauRobotStatus.msg"
  "msg/ComauServerError.msg"
  "msg/Digital.msg"
  "msg/IOStates.msg"
  "msg/JointPose.msg"
  "msg/SnsTrkPlot.msg"
)

set(srv_files
  "srv/ControllerWrapper.srv"
  "srv/GripperCommand.srv"
  "srv/OpenConnection.srv"
  "srv/SetArmState.srv"
  "srv/SetIO.srv"
  "srv/SetMoveFlyParams.srv"
  "srv/SetSnsTrkParams.srv"
  "srv/SnsTrkPlotReset.srv"
)

set(action_files
  "action/ExecuteCartesianTrajectory.action"
  "action/ExecuteJointTrajectory.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies (std_msgs geometry_msgs actionlib_msgs trajectory_msgs shape_msgs rosidl_default_runtime)
# ament_export_include_directories(...)
# ament_export_libraries(...)
ament_package()
