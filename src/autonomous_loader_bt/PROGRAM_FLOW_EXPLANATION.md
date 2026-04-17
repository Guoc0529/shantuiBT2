# 自主装载机器人行为树程序运行详解

## 一、程序启动流程

### 1. 节点初始化（main函数）
```
位置: autonomous_loader_bt_node.cpp 第323-335行
```

**执行顺序：**
1. `ros::init(argc, argv, "autonomous_loader_bt_node")` - 初始化ROS节点
2. 创建 `AutonomousLoaderBTNode` 对象
3. 进入 `ros::spin()` - 开始ROS消息循环

### 2. 构造函数初始化（AutonomousLoaderBTNode构造）
```
位置: autonomous_loader_bt_node.cpp 第17-77行
```

**执行步骤：**
```
1. 读取配置参数（tree_file, config_file等）
2. loadConfigFile() - 加载YAML配置文件
3. setupROSTopics() - 设置ROS话题（订阅者+发布者）
4. 初始化ROSTopicManager
5. 创建BehaviorTreeFactory并注册所有自定义节点
6. 加载行为树XML文件
7. 创建定时器（10Hz，调用executeTree）
8. initializeTestTasks() - 初始化测试任务
```

---

## 二、ROS话题订阅和回调函数映射表

### 订阅话题列表（第101-122行）

| 订阅话题 | 消息类型 | 回调函数 | 作用 |
|---------|---------|---------|------|
| `scheduler/start_task` | TaskCommand | `startTaskCallback()` | 接收新任务指令 |
| `scheduler/pause_task` | Bool | `pauseTaskCallback()` | 接收暂停指令 |
| `scheduler/end_task` | Bool | `endTaskCallback()` | 接收结束指令 |
| `work_device/task_completed` | Bool | `workDeviceCompletedCallback()` | 工作装置完成状态 |
| `work_device/arm_raised` | Bool | `armRaisedCallback()` | 抬大臂完成回令 |
| `work_device/bucket_lowered` | Bool | `bucketLoweredCallback()` | 放平铲斗完成回令 |
| `scoop/scoop_completed` | Bool | `scoopCompletedCallback()` | 铲料完成状态 |
| `navigation/planning_success` | Bool | `navigationPlanningSuccessCallback()` | 导航规划成功 |
| `navigation/arrived` | Bool | `navigationArrivedCallback()` | 导航到达 |
| `navigation/distance_to_goal` | Float64 | `distanceToGoalCallback()` | 距离目标点的距离 |
| `navigation/current_pose` | PoseStamped | `currentPoseCallback()` | 当前位姿 |

### 回调函数详解

#### 1. startTaskCallback (第203-217行)
```
话题: scheduler/start_task
触发: 调度模块发送任务
执行流程:
  1. 接收任务信息（task_id, bin_id, hopper_id, task_type）
  2. 创建Task对象
  3. 调用 state.addTask(task) 添加到队列
  4. 任务进入GlobalState的任务队列，等待行为树处理
```

#### 2. pauseTaskCallback (第219-224行)
```
话题: scheduler/pause_task
触发: 调度模块发送暂停指令
执行流程:
  1. 接收Bool消息
  2. 调用 state.setPauseTask(msg->data)
  3. 行为树中的CheckPauseTask节点检测到暂停标志
```

#### 3. endTaskCallback (第226-231行)
```
话题: scheduler/end_task
触发: 调度模块发送结束指令
执行流程:
  1. 接收Bool消息
  2. 调用 state.setEndTask(msg->data)
  3. 行为树中的CheckEndTask节点检测到结束标志
```

#### 4. armRaisedCallback (第240-248行)
```
话题: work_device/arm_raised
触发: 工作装置完成抬大臂动作
执行流程:
  1. 接收Bool消息
  2. 设置 arm_raised_ack = true（抬大臂应答标志）
  3. 设置 work_device_completed = true（工作装置完成标志）
  4. 行为树中等待该标志的节点会被唤醒
```

#### 5. bucketLoweredCallback (第250-258行)
```
话题: work_device/bucket_lowered
触发: 工作装置完成放平铲斗动作
执行流程:
  1. 接收Bool消息
  2. 设置 bucket_lowered_ack = true
  3. 设置 work_device_completed = true
```

#### 6. distanceToGoalCallback (第281-285行)
```
话题: navigation/distance_to_goal
触发: 导航模块持续发送距离信息
执行流程:
  1. 接收Float64消息（米）
  2. 更新 state.distance_to_goal_
  3. 行为树中的CheckDistanceThreshold节点会根据此距离判断
```

**重要：** 此话题持续发送，实现距离的"时时刻刻"监控！

---

## 三、行为树执行流程

### 行为树在主循环中的执行
```
位置: 第141-163行（executeTree函数）
```

**触发机制：**
- 定时器以10Hz频率调用 `executeTree()`
- 每次调用执行 `tree_.tickOnce()` - 行为树的一个tick
- 行为树会重复执行直到达到终止条件

### 行为树结构（ReactiveSelector）

```
ROOT (ReactiveSelector - 响应式选择器)
├─ SEQ_STOP (处理停止 - 最高优先级)
│  ├─ CheckEndTask
│  ├─ ResetWorkDeviceCompleted
│  ├─ ResetScoopCompleted
│  ├─ ResetNavigationArrived
│  ├─ ResetNavigationPlanningSuccess
│  ├─ SendNavigationGoalParking
│  └─ WaitForNavigationArrival
│
├─ SEQ_PAUSE (处理暂停 - 中等优先级)
│  ├─ CheckPauseTask
│  ├─ PauseNavigation
│  └─ UpdateTaskProgress
│
└─ SEL_WORK_IDLE (正常工作或空闲 - 常规优先级)
   ├─ SEQ_TASK_LOOP (尝试工作)
   │  ├─ CheckNewTask
   │  ├─ GetAndSetCurrentTask
   │  └─ RepeatUntilFail
   │     └─ SEQ_ONE_CYCLE (一个完整循环)
   │        ├─ SEQ_PREPARE (准备：抬大臂)
   │        ├─ SEQ_SCOOP (铲料)
   │        ├─ SEQ_DUMP (卸料)
   │        └─ CheckForNextCycle
   │
   └─ SEQ_IDLE_PARK (无任务：停车)
      └─ SendNavigationGoalParking
```

---

## 四、完整执行流程示例：铲料任务

### 阶段1：任务接收与初始化

```
1. 调度模块发布任务到 scheduler/start_task
   ↓
2. startTaskCallback() 被调用
   ↓
3. 任务被添加到GlobalState.task_queue_
   ↓
4. 行为树开始执行（每10ms执行一次）
   ↓
5. CheckNewTask 节点检测到任务存在
   ↓
6. GetAndSetCurrentTask 节点从队列取出任务并设置为当前任务
```

### 阶段2：准备阶段（抬大臂）

```
1. 行为树执行到 SEQ_PREPARE
   ↓
2. ResetArmRaisedAck - 重置抬大臂标志
   ↓
3. ResetWorkDeviceCompleted - 重置工作装置完成标志
   ↓
4. SendRaiseArmCommand 节点：
   - 调用 topic_manager.publishRaiseArmCommand()
   - 发布消息到 work_device/raise_arm
   ↓
5. WaitForWorkDeviceCompletion 节点：
   - 检查 work_device_completed_ 标志
   - 如果为false，返回RUNNING（持续等待）
   ↓
6. 工作装置模块执行抬大臂动作
   ↓
7. 工作装置完成后发布到 work_device/arm_raised
   ↓
8. armRaisedCallback() 被调用：
   - 设置 arm_raised_ack = true
   - 设置 work_device_completed = true
   ↓
9. WaitForWorkDeviceCompletion 检测到标志变为true，返回SUCCESS
   ↓
10. 准备阶段完成，进入铲料阶段
```

### 阶段3：铲料阶段

#### 3.1 请求铲料点

```
1. RequestScoopPoint 节点执行
   - 向铲料点计算模块请求位置
   - 将结果存储到 state.scoop_point_
   ↓
2. CheckScoopPointZero 节点判断铲料点是否为(0,0)
```

#### 3.2 处理铲料点（两种情况）

**情况A：铲料点为(0,0)**

```
1. 进入 SEQ_ZERO_CASE
   ↓
2. SendNavigationGoalBinCenter：
   - 调用 config.getBinCenter(bin_id)
   - 发布到 navigation/goal
   ↓
3. WaitForNavigationPlanningSuccess：
   - 持续检查 navigation_planning_success_
   - 等待导航模块规划成功
   ↓
4. 导航模块发布到 navigation/planning_success
   ↓
5. navigationPlanningSuccessCallback() 设置标志为true
   ↓
6. WaitForNavigationArrival：
   - 持续检查 navigation_arrived_
   - 等待到达目标
   ↓
7. 导航模块发布到 navigation/arrived
   ↓
8. navigationArrivedCallback() 设置标志为true
   ↓
9. RequestScoopPoint 重新请求铲料点（此时应该得到有效坐标）
```

**情况B：铲料点非(0,0)**

```
1. 进入 SEQ_NORMAL_CASE
   ↓
2. SendNavigationGoalScoopPoint：
   - 发布铲料点到 navigation/goal
   ↓
3. WaitForNavigationPlanningSuccess：
   - 等待规划成功
```

#### 3.3 铲料过程（持续监控距离）

```
第141-181行的行为树逻辑

关键：CheckDistanceThreshold 节点会持续检查距离！
每10ms（10Hz）执行一次，每次都重新评估距离条件。
```

**步骤1：距离<=6米，放平铲斗**

```
1. CheckDistanceThreshold (threshold_name="lower_bucket_distance")
   - 从state获取 distance_to_goal_
   - 与配置的6.0米比较
   - 如果 <= 6米，返回SUCCESS，否则返回FAILURE
   ↓
2. 距离满足条件后：
   - ResetBucketLoweredAck - 重置标志
   - SendLowerBucketCommand - 发布到 work_device/lower_bucket
   - WaitForWorkDeviceCompletion - 等待完成
   ↓
3. 工作装置收到指令，执行放平铲斗
   ↓
4. 完成后发布到 work_device/bucket_lowered
   ↓
5. bucketLoweredCallback() 被调用：
   - 设置 bucket_lowered_ack = true
   - 设置 work_device_completed = true
   ↓
6. WaitForWorkDeviceCompletion 返回SUCCESS
```

**步骤2：距离<=3米且已放平铲斗，开始铲料**

```
1. CheckBucketLoweredAck - 检查放平铲斗回令
   ↓
2. CheckDistanceThreshold (threshold_name="start_scoop_distance")
   - 比较 distance_to_goal_ 与 3.0米
   ↓
3. 同时满足后：
   - SendScoopCommand - 发布到 scoop/start_scoop
   - WaitForScoopCompletion - 等待完成
   ↓
4. 铲料模块执行铲料动作
   ↓
5. 完成后发布到 scoop/scoop_completed
   ↓
6. scoopCompletedCallback() 设置 scoop_completed = true
   ↓
7. WaitForScoopCompletion 返回SUCCESS
   ↓
8. 铲料阶段完成，进入卸料阶段
```

**重要说明：**
- `navigation/distance_to_goal` 持续发送（时时刻刻）
- `distanceToGoalCallback()` 持续更新距离
- `CheckDistanceThreshold` 每次tick都重新评估距离
- 这样就实现了"时时刻刻订阅导航发送来的距离"的要求！

### 阶段4：卸料阶段

#### 4.1 导航到料斗

```
1. SendNavigationGoalHopper：
   - 从config获取料斗位置
   - 发布到 navigation/goal
   ↓
2. 等待导航规划成功
   ↓
3. 等待导航到达料斗
```

#### 4.2 卸料过程（持续监控距离）

**步骤1：距离<=6米，抬大臂**

```
1. CheckDistanceThreshold (threshold_name="raise_arm_distance")
   - 距离 <= 6米
   ↓
2. ResetArmRaisedAck - 重置标志
   ↓
3. SendRaiseArmCommand - 发布指令
   ↓
4. WaitForWorkDeviceCompletion - 等待完成
```

**步骤2：距离<=0米且已抬大臂，卸料**

```
1. CheckArmRaisedAck - 检查回令
   ↓
2. CheckDistanceThreshold (threshold_name="start_dump_distance")
   - 距离 <= 0米
   ↓
3. SendScoopCommand - 发布卸料指令
   ↓
4. WaitForScoopCompletion - 等待完成
   ↓
5. 卸料阶段完成
```

### 阶段5：检查下一轮任务

```
1. CheckForNextCycle 节点：
   - 检查 GlobalState 中是否还有任务
   ↓
2. 如果有：
   - RepeatUntilFail 返回 RUNNING
   - 重新开始 SEQ_ONE_CYCLE
   ↓
3. 如果没有：
   - 导航到过道停车点
   - 保持空闲状态
```

---

## 五、数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                      调度模块 (Scheduler)                     │
└────────────────────────────┬────────────────────────────────┘
                             │ 发布任务
                             ↓
                    scheduler/start_task (TopicCommand)
                             │
                             ↓
┌─────────────────────────────────────────────────────────────┐
│              autonomous_loader_bt_node                       │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  订阅: scheduler/start_task                          │   │
│  │  → startTaskCallback()                               │   │
│  │  → state.addTask(task)  ← 添加到队列                 │   │
│  └─────────────────────────────────────────────────────┘   │
│                            │                                │
│                            ↓                                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  执行: tree_.tickOnce() (10Hz)                       │   │
│  │  1. CheckNewTask → 检测到任务                        │   │
│  │  2. GetAndSetCurrentTask → 取任务                   │   │
│  │  3. SendRaiseArmCommand → 发布                      │   │
│  └────────┬───────────────────────────────────────────┘   │
│           │ 发布命令                                         │
│           ↓                                                  │
└───────────┼─────────────────────────────────────────────────┘
            │
            ↓
┌─────────────────────────────────────────────────────────────┐
│              工作装置模块 (Work Device)                       │
│  订阅: work_device/raise_arm                                 │
│  执行: 抬大臂                                                 │
│  发布: work_device/arm_raised (Bool)                         │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ↓
            work_device/arm_raised
                        │
                        ↓
┌─────────────────────────────────────────────────────────────┐
│  回调: armRaisedCallback()                                   │
│  → setArmRaisedAck(true)                                    │
│  → setWorkDeviceCompleted(true)                             │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ↓
┌─────────────────────────────────────────────────────────────┐
│  行为树 WaitForWorkDeviceCompletion                         │
│  检测到标志为true → 返回SUCCESS                              │
│  继续执行下一个节点                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 六、关键机制说明

### 1. 持续距离监控机制

```
导航模块 (10Hz) → navigation/distance_to_goal
                 ↓
    distanceToGoalCallback() (持续更新)
                 ↓
    GlobalState.distance_to_goal_
                 ↓
    行为树 CheckDistanceThreshold (每次tick都重新评估)
                 ↓
    当距离满足条件时触发动作
```

### 2. 状态同步机制

```
外部模块发布消息 → ROS话题
                 ↓
回调函数更新GlobalState状态
                 ↓
行为树节点读取GlobalState
                 ↓
根据状态决定执行路径
```

### 3. 任务队列机制

```
调度模块发送任务
        ↓
startTaskCallback() 添加到队列
        ↓
GetAndSetCurrentTask 取出任务
        ↓
RepeatUntilFail 执行循环
        ↓
CheckForNextCycle 检查是否还有任务
```

---

## 七、调试方法

### 查看发布的话题消息
```bash
# 监听任务发布
rostopic echo /scheduler/start_task

# 监听状态机状态
rostopic echo /state_machine/status

# 监听距离信息
rostopic echo /navigation/distance_to_goal
```

### 手动发布测试消息
```bash
# 发布测试任务
rostopic pub /scheduler/start_task autonomous_loader_msgs/TaskCommand "
task_id: 999
bin_id: 1
hopper_id: 1
task_type: 'scoop'"

# 发布抬大臂回令
rostopic pub /work_device/arm_raised std_msgs/Bool "data: true"

# 发布距离信息
rostopic pub /navigation/distance_to_goal std_msgs/Float64 "data: 5.5"
```

### 查看行为树执行日志
```bash
# 查看节点日志
rosnode info /autonomous_loader_bt_node

# 查看行为树日志
tail -f ~/.ros/autonomous_loader_logs.btlog
```

---

## 八、总结

程序运行的核心特点：

1. **事件驱动 + 定时循环**：
   - ROS回调函数异步处理事件
   - 行为树定时器同步执行逻辑

2. **状态集中管理**：
   - GlobalState作为全局状态中心
   - 所有模块通过GlobalState共享状态

3. **响应式执行**：
   - ReactiveSelector确保高优先级指令立即响应
   - 持续监控距离，实时响应

4. **模块化设计**：
   - 每个功能模块独立
   - 通过ROS话题通信
   - 行为树协调整体流程

这就是整个程序从订阅到完成任务的完整流程！

