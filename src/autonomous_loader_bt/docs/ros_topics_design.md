# 自主装载机器人ROS话题接口设计

## 调度模块话题

### 订阅话题
- `/scheduler/start_task` (autonomous_loader_msgs/TaskCommand)
  - 任务编号
  - 料斗标号
  - 料仓标号

- `/scheduler/pause_task` (std_msgs/Bool)
  - 暂停任务指令

- `/scheduler/end_task` (std_msgs/Bool)
  - 结束任务指令

## 工作装置模块话题

### 发布话题
- `/work_device/raise_arm` (std_msgs/Bool)
  - 抬大臂指令

- `/work_device/lower_bucket` (std_msgs/Bool)
  - 放平铲斗指令

### 订阅话题
- `/work_device/arm_raised` (std_msgs/Bool)
  - 抬大臂完成回令

- `/work_device/bucket_lowered` (std_msgs/Bool)
  - 放平铲斗完成回令

- `/work_device/task_completed` (std_msgs/Bool)
  - 工作装置任务完成状态

## 铲料模块话题

### 发布话题
- `/scoop/start_scoop` (std_msgs/Bool)
  - 开始铲料指令

### 订阅话题
- `/scoop/scoop_completed` (std_msgs/Bool)
  - 铲料完成回令

## 导航模块话题

### 发布话题
- `/navigation/goal` (geometry_msgs/PoseStamped)
  - 导航目标位置

- `/navigation/pause` (std_msgs/Bool)
  - 暂停导航指令

### 订阅话题
- `/navigation/planning_success` (std_msgs/Bool)
  - 路径规划成功状态

- `/navigation/arrived` (std_msgs/Bool)
  - 到达目标位置状态

- `/navigation/distance_to_goal` (std_msgs/Float64)
  - 距离目标点的距离

- `/navigation/current_pose` (geometry_msgs/PoseStamped)
  - 当前机器人位置

## 铲料点计算模块话题

### 发布话题
- `/scoop_point/request` (autonomous_loader_msgs/ScoopPointRequest)
  - 请求铲料点位置

### 订阅话题
- `/scoop_point/response` (geometry_msgs/PoseStamped)
  - 铲料点位置响应

## 状态机话题

### 发布话题
- `/state_machine/status` (std_msgs/String)
  - 状态机当前状态

- `/state_machine/task_progress` (autonomous_loader_msgs/TaskProgress)
  - 任务执行进度

### 订阅话题
- `/state_machine/emergency_stop` (std_msgs/Bool)
  - 紧急停止指令
