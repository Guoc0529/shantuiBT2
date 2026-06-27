cmake_minimum_required(VERSION 3.0.2)
project(center_articulation_planner_legacy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

# ------------------------------
# 1. Core (pure C++)
# ------------------------------
# 备注：如果将来添加纯 C++ core 库（如 add_library(center_articulation_planner_core ...)
# 再为该目标设置 include 路径。当前未创建该目标，避免对不存在的目标设置属性。

# ------------------------------
# 2. ROS Wrapper
# ------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  actionlib_msgs
  my_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  local_planner
  message_generation
  autonomous_loader_msgs
  hybrid_a_star
)

add_message_files(
  DIRECTORY msg
  FILES
    IsForwardState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    PlanHybridPath.srv
    AlignInPlace.srv
    SetPlannerParams.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs nav_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  center_articulation_planner_ros
  CATKIN_DEPENDS roscpp roslib actionlib actionlib_msgs my_msgs geometry_msgs nav_msgs std_msgs sensor_msgs tf2 tf2_geometry_msgs local_planner message_runtime autonomous_loader_msgs hybrid_a_star
)

add_library(center_articulation_planner_ros
  src/ros/remote_planner_backend.cpp
  src/ros/planner_ros.cpp
  src/ros/planner_ros_legacy.cpp
  src/ros/ros_io.cpp
)

add_dependencies(center_articulation_planner_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(center_articulation_planner_ros
  PRIVATE ${catkin_LIBRARIES}
)

target_include_directories(center_articulation_planner_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE ${catkin_INCLUDE_DIRS}
)

add_executable(center_articulation_planner_node src/planner_node.cpp)

add_dependencies(center_articulation_planner_node
  center_articulation_planner_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(center_articulation_planner_node center_articulation_planner_ros ${catkin_LIBRARIES})

target_include_directories(center_articulation_planner_node PRIVATE ${catkin_INCLUDE_DIRS})



install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.yaml"
    PATTERN "*.json"
    PATTERN "*.launch"
)

catkin_install_python(PROGRAMS
  scripts/goal_guard_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
