cmake_minimum_required(VERSION 3.8)
project(autonomous_loader_bt)

# 步骤 1: 找到 Catkin 和所有 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rosbag
  actionlib
  actionlib_msgs
  move_base_msgs
  autonomous_loader_msgs
  shuju
)

# 步骤 2: 找到外部库
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 步骤 3: (非常重要) 添加 catkin_package()
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs rosbag actionlib actionlib_msgs move_base_msgs autonomous_loader_msgs shuju
)

# --- 编译设置 ---

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Inject package source path as C++ macro for runtime resource location
add_definitions(-DAUTONOMOUS_LOADER_BT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# 步骤 4: 包含头文件目录 (使用变量)
# [修正] 链接 BT::behaviortree_cpp 会自动处理它的头文件
# 所以我们不再需要 ${behaviortree_cpp_INCLUDE_DIRS}
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 步骤 5: 创建可执行文件
add_executable(autonomous_loader_bt_node 
  src/autonomous_loader_bt_node.cpp 
  src/loader_bt_nodes.cpp
)

# 步骤 6: 添加消息依赖
add_dependencies(autonomous_loader_bt_node ${catkin_EXPORTED_TARGETS})

# 步骤 7: 链接库 (使用变量)
# [修正!] 这是解决 "undefined reference" 错误的关键
# 我们链接到 CMake 目标 "BT::behaviortree_cpp"
# 而不是旧的变量 ${behaviortree_cpp_LIBRARIES}
target_link_libraries(autonomous_loader_bt_node
  ${catkin_LIBRARIES}
  BT::behaviortree_cpp
  ${YAML_CPP_LIBRARIES}
  pthread # pthread 和 dl 是 BT.CPP 可能需要的
  dl
)

# 步骤 8: 安装规则 (与您原来的一致)
install(TARGETS autonomous_loader_bt_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
  FILES_MATCHING PATTERN "*.xml"
)

install(PROGRAMS 
  scripts/module_simulator.py
  scripts/test_full_workflow.py
  scripts/interactive_tester.py
  scripts/mock_spading_service.py
  DESTINATION lib/${PROJECT_NAME}
)