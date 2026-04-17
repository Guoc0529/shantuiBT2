cmake_minimum_required(VERSION 3.8)
project(autonomous_loader_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  TaskCommand.msg
  ScoopPointRequest.msg
  TaskProgress.msg
)

add_service_files(
  FILES
  spadingPose.srv
)

add_action_files(
  FILES
  Navigate.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs
)
