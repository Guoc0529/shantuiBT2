cmake_minimum_required(VERSION 3.0.2)
project(shuju)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
add_service_files(FILES cangdou.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
