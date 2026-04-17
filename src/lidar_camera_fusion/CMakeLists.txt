cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_fusion)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Obstacle.msg
  ObstacleArray.msg
  originObs.msg
  originObss.msg
  PerceptInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

