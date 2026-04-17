# 自主装载机器人行为树系统 - 代码逻辑梳理

## 一、系统架构概览

### 1.1 整体架构
```
┌─────────────────────────────────────────────────────────────┐
│                    调度模块 (Scheduler)                       │
│  - 通过Service (/liaodou) 或Topic (/scheduler/start_task)   │
│    下发任务到状态机                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ↓
┌─────────────────────────────────────────────────────────────┐
│          autonomous_loader_bt_node (主节点)                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  1. ROS话题订阅/发布管理                                │  │
│  │  2. 行为树执行引擎 (10Hz定时器)                        │  │
│  │  3. 回调函数处理外部消息                                │  │
│  └──────────────────────────────────────────────────────┘  │
│                       │                                      │
│                       ↓                                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  GlobalState (全局状态管理 - 单例模式)                │  │
│  │  - 任务队列管理                                        │  │
│  │  - 各模块状态标志                                      │  │
│  │  - 距离、位姿等实时数据                                │  │
│  └──────────────────────────────────────────────────────┘  │
│                       │                                      │
│                       ↓                                      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  BehaviorTree (行为树执行逻辑)                        │  │
│  │  - ReactiveFallback根节点 (响应式优先级)               │  │
│  │  - 条件节点 (检查状态)                                 │  │
│  │  - 动作节点 (发送指令)                                 │  │
│  └──────────────────────────────────────────────────────┘  │
└──────────────────────┬──────────────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        ↓              ↓              ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│  工作装置模块  │ │  铲料模块    │ │  导航模块    │
│  (WorkDevice) │ │  (Scoop)     │ │ (Navigation) │
└──────────────┘ └──────────────┘ └──────────────┘
```

### 1.2 核心组件

#### 1.2.1 主节点 (autonomous_loader_bt_node.cpp)
- **职责**：ROS节点初始化、消息循环、行为树执行
- **关键功能**：
  - 加载配置文件（YAML）
  - 设置ROS话题订阅/发布
  - 创建行为树工厂并注册节点
  - 10Hz定时器执行行为树
  - 处理所有ROS回调函数

#### 1.2.2 行为树节点 (loader_bt_nodes.cpp/h)
- **职责**：定义行为树的所有节点类型
- **节点分类**：
  - **条件节点**：检查状态（如CheckNewTask、CheckDistanceThreshold）
  - **动作节点**：执行操作（如SendRaiseArmCommand、RequestScoopPoint）
  - **等待节点**：等待异步操作完成（如WaitForWorkDeviceCompletion）

#### 1.2.3 全局状态管理 (GlobalState)
- **职责**：集中管理所有状态信息
- **关键状态**：
  - 任务队列（task_queue_）
  - 当前任务（current_task_）
  - 暂停/结束标志（pause_task_、end_task_）
  - 各模块完成标志（work_device_completed_、scoop_completed_等）
  - 导航状态（navigation_arrived_、distance_to_goal_等）
  - 铲料点位置（scoop_point_）

#### 1.2.4 配置管理 (ConfigManager)
- **职责**：管理YAML配置文件
- **配置内容**：
  - 料仓位置（cang）
  - 料斗位置（dou）
  - 停车点位置（parking）
  - 工作参数（距离阈值、超时时间等）

#### 1.2.5 ROS话题管理器 (ROSTopicManager)
- **职责**：统一管理ROS话题发布和服务调用
- **功能**：
  - 发布指令到各模块
  - 调用服务获取铲料点

---

## 二、数据流和消息传递

### 2.1 任务接收流程

```
调度模块
  │
  ├─ Service方式: /liaodou (shuju::cangdou)
  │   └─→ taskServiceCallback()
  │       └─→ GlobalState::addTask()
  │
  └─ Topic方式: /scheduler/start_task (TaskCommand)
      └─→ startTaskCallback()
          └─→ GlobalState::addTask()
```

### 2.2 指令发送流程

```
行为树节点 (如SendRaiseArmCommand)
  │
  └─→ ROSTopicManager::publishRaiseArmCommand()
      └─→ 发布到 /work_device/raise_arm
          └─→ 工作装置模块接收并执行
```

### 2.3 状态反馈流程

```
外部模块 (如工作装置)
  │
  └─→ 发布到ROS话题 (如 /work_device/arm_raised)
      └─→ 回调函数 (如armRaisedCallback())
          └─→ GlobalState::setArmRaisedAck(true)
              └─→ GlobalState::setWorkDeviceCompleted(true)
                  └─→ 行为树节点检测到状态变化
                      └─→ WaitForWorkDeviceCompletion返回SUCCESS
```

### 2.4 距离监控流程（关键机制）

```
导航模块 (持续10Hz发布)
  │
  └─→ /navigation/distance_to_goal (Float64)
      └─→ distanceToGoalCallback()
          └─→ GlobalState::setDistanceToGoal(distance)
              └─→ 行为树每10ms执行一次
                  └─→ CheckDistanceThreshold节点持续检查
                      └─→ 当距离满足条件时触发动作
```

**关键点**：这是"时时刻刻监控距离"的实现方式，通过持续订阅和持续检查实现实时响应。

---

## 三、行为树结构详解

### 3.1 根节点：ReactiveFallback（响应式回退）

```
ReactiveFallback (ROOT)
├─ SEQ_STOP (最高优先级 - 处理停止)
│  ├─ CheckEndTask
│  ├─ ResetWorkDeviceCompleted
│  ├─ ResetScoopCompleted
│  ├─ ResetNavigationArrived
│  ├─ ResetNavigationPlanningSuccess
│  ├─ SendNavigationGoalParking
│  └─ WaitForNavigationArrival
│
├─ SEQ_PAUSE (中等优先级 - 处理暂停)
│  ├─ CheckPauseTask
│  ├─ PauseNavigation
│  └─ UpdateTaskProgress
│
└─ SEL_WORK_IDLE (常规优先级 - 工作/空闲)
   ├─ SEQ_TASK_LOOP (尝试工作)
   │  ├─ CheckNewTask
   │  ├─ GetAndSetCurrentTask
   │  ├─ ResetWorkDeviceCompleted
   │  ├─ ResetScoopCompleted
   │  ├─ ResetNavigationArrived
   │  ├─ ResetNavigationPlanningSuccess
   │  ├─ ResetArmRaisedAck
   │  ├─ ResetBucketLoweredAck
   │  └─ KeepRunningUntilFailure
   │     └─ SEQ_ONE_CYCLE (一个完整循环)
   │        ├─ SEQ_PREPARE (准备阶段)
   │        ├─ SEQ_SCOOP (铲料阶段)
   │        ├─ SEQ_DUMP (卸料阶段)
   │        └─ CheckForNextCycle
   │
   └─ UpdateTaskProgress (无任务时保持空闲)
```

**ReactiveFallback特性**：
- 从左到右依次检查子节点
- 如果子节点返回RUNNING，会持续监控
- 如果高优先级节点条件满足，会立即中断低优先级节点
- 这确保了"停止"和"暂停"指令能够立即响应

### 3.2 准备阶段 (SEQ_PREPARE)

```
Sequence (SEQ_PREPARE)
├─ UpdateTaskProgress (phase="preparing", progress=10%)
├─ ResetArmRaisedAck
├─ ResetWorkDeviceCompleted
├─ SendRaiseArmCommand
│  └─→ 发布到 /work_device/raise_arm
└─ WaitForWorkDeviceCompletion
   └─→ 持续检查 work_device_completed_ 标志
       └─→ 当 armRaisedCallback() 设置标志为true时返回SUCCESS
```

### 3.3 铲料阶段 (SEQ_SCOOP)

#### 3.3.1 请求铲料点

```
RequestScoopPoint
└─→ 调用服务 /spadingPose
    ├─ 成功：设置 scoop_point_
    └─ 失败：使用默认位置（料仓中心+0.5米偏移）
```

#### 3.3.2 处理铲料点（两种情况）

**情况A：铲料点为(0,0)**
```
Fallback (SEL_HANDLE_SCOOP_POINT)
└─ Sequence (SEQ_ZERO_CASE)
   ├─ CheckScoopPointZero (检查是否为(0,0))
   ├─ ResetNavigationArrived
   ├─ ResetNavigationPlanningSuccess
   ├─ SendNavigationGoalBinCenter
   │  └─→ 发布料仓中心到 /navigation/goal
   ├─ WaitForNavigationPlanningSuccess
   ├─ WaitForNavigationArrival
   └─ RequestScoopPoint (重新请求)
```

**情况B：铲料点非(0,0)**
```
Fallback (SEL_HANDLE_SCOOP_POINT)
└─ Sequence (SEQ_NORMAL_CASE)
   ├─ ResetNavigationArrived
   ├─ ResetNavigationPlanningSuccess
   ├─ SendNavigationGoalScoopPoint
   │  └─→ 发布铲料点到 /navigation/goal
   └─ WaitForNavigationPlanningSuccess
```

#### 3.3.3 铲料过程（持续监控距离）

```
ReactiveFallback (SEQ_SCOOP_PROCESS)
├─ CheckScoopCompleted (如果已完成，直接成功)
│
└─ ReactiveFallback (SEQ_SCOOP_MONITOR)
   ├─ ReactiveSequence (SEQ_LOWER_BUCKET) - 放平铲斗
   │  ├─ CheckDistanceThreshold (threshold="lower_bucket_distance", <=6米)
   │  ├─ Inverter
   │  │  └─ CheckBucketLoweredAck (检查是否已执行过)
   │  ├─ ResetBucketLoweredAck
   │  ├─ ResetWorkDeviceCompleted
   │  ├─ SendLowerBucketCommand
   │  │  └─→ 发布到 /work_device/lower_bucket
   │  └─ WaitForWorkDeviceCompletion
   │
   └─ ReactiveSequence (SEQ_START_SCOOP) - 开始铲料
      ├─ CheckBucketLoweredAck (必须已放平铲斗)
      ├─ CheckDistanceThreshold (threshold="start_scoop_distance", <=3米)
      ├─ SendScoopCommand
      │  └─→ 发布到 /scoop/start_scoop
      └─ WaitForScoopCompletion
```

**关键机制**：
- `ReactiveFallback`确保持续重试
- `ReactiveSequence`确保条件持续检查
- 距离检查每10ms执行一次（行为树执行频率）
- 只有当距离满足条件且未执行过时才会发送指令

### 3.4 卸料阶段 (SEQ_DUMP)

```
Sequence (SEQ_DUMP)
├─ UpdateTaskProgress (phase="dumping", progress=70%)
├─ ResetNavigationArrived
├─ ResetNavigationPlanningSuccess
├─ SendNavigationGoalHopper
│  └─→ 发布料斗位置到 /navigation/goal
├─ WaitForNavigationPlanningSuccess
├─ WaitForNavigationArrival
└─ ReactiveFallback (SEQ_DUMP_PROCESS)
   ├─ Sequence (CheckDumpCompleted)
   │  ├─ CheckScoopCompleted (检查卸料是否完成)
   │  └─ ResetScoopCompleted
   │
   └─ ReactiveFallback (SEQ_DUMP_MONITOR)
      ├─ ReactiveSequence (SEQ_RAISE_ARM_DUMP) - 抬大臂
      │  ├─ CheckDistanceThreshold (threshold="raise_arm_distance", <=6米)
      │  ├─ Inverter
      │  │  └─ CheckArmRaisedAck (检查是否已执行过)
      │  ├─ ResetArmRaisedAck
      │  ├─ ResetWorkDeviceCompleted
      │  ├─ SendRaiseArmCommand
      │  │  └─→ 发布到 /work_device/raise_arm
      │  └─ WaitForWorkDeviceCompletion
      │
      └─ ReactiveSequence (SEQ_START_DUMP) - 开始卸料
         ├─ CheckArmRaisedAck (必须已抬大臂)
         ├─ CheckDistanceThreshold (threshold="start_dump_distance", <=0米)
         ├─ SendScoopCommand
         │  └─→ 发布到 /scoop/start_scoop (语义上是卸料)
         └─ WaitForScoopCompletion
```

**注意**：卸料和铲料使用同一个话题`/scoop/start_scoop`，但通过上下文（距离、位置）区分。

---

## 四、关键节点实现逻辑

### 4.1 条件节点

#### CheckDistanceThreshold
```cpp
BT::NodeStatus CheckDistanceThreshold::tick()
{
    // 1. 获取阈值名称（从XML配置）
    std::string threshold_name;
    getInput("threshold_name", threshold_name);
    
    // 2. 从ConfigManager获取阈值
    double threshold = config.getDistanceThreshold(threshold_name);
    
    // 3. 从GlobalState获取当前距离
    double current_distance = state.getDistanceToGoal();
    
    // 4. 比较
    bool satisfied = current_distance <= threshold;
    
    // 5. 返回结果
    if (satisfied) {
        return BT::NodeStatus::SUCCESS;
    }
    // 返回RUNNING而不是FAILURE，避免上层节点提前判定失败
    return BT::NodeStatus::RUNNING;
}
```

**设计要点**：
- 返回RUNNING而不是FAILURE，确保持续监控
- 每10ms执行一次，实现实时响应

#### CheckNewTask
```cpp
BT::NodeStatus CheckNewTask::tick()
{
    auto& state = GlobalState::getInstance();
    bool has_task = state.hasNewTask();
    return has_task ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
```

### 4.2 动作节点

#### SendRaiseArmCommand
```cpp
BT::NodeStatus SendRaiseArmCommand::tick()
{
    // 1. 发布指令
    auto& topic_manager = ROSTopicManager::getInstance();
    topic_manager.publishRaiseArmCommand();
    
    // 2. 立即返回SUCCESS（不等待完成）
    // 完成状态通过回调函数异步更新
    return BT::NodeStatus::SUCCESS;
}
```

**设计要点**：
- 动作节点只负责发送指令，不等待完成
- 完成状态由等待节点（WaitForWorkDeviceCompletion）处理

#### RequestScoopPoint
```cpp
BT::NodeStatus RequestScoopPoint::tick()
{
    // 1. 获取当前任务
    Task current_task = state.getCurrentTask();
    uint8_t pileID = static_cast<uint8_t>(current_task.bin_id);
    
    // 2. 调用服务
    geometry_msgs::PoseStamped scoop_point;
    if (topic_manager.requestScoopPoint(pileID, scoop_point)) {
        // 成功：设置铲料点
        state.setScoopPoint(scoop_point);
        return BT::NodeStatus::SUCCESS;
    } else {
        // 失败：使用默认位置
        auto& config = ConfigManager::getInstance();
        geometry_msgs::PoseStamped bin_center = config.getBinCenter(current_task.bin_id);
        scoop_point = bin_center;
        scoop_point.pose.position.x += 0.5;
        scoop_point.pose.position.y += 0.5;
        state.setScoopPoint(scoop_point);
        return BT::NodeStatus::SUCCESS;  // 即使失败也继续流程
    }
}
```

### 4.3 等待节点

#### WaitForWorkDeviceCompletion
```cpp
BT::NodeStatus WaitForWorkDeviceCompletion::onStart()
{
    ROS_INFO("Waiting for work device completion");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForWorkDeviceCompletion::onRunning()
{
    auto& state = GlobalState::getInstance();
    if (state.isWorkDeviceCompleted()) {
        ROS_INFO("Work device task completed");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;  // 持续等待
}
```

**设计要点**：
- 使用StatefulActionNode实现异步等待
- 每10ms检查一次状态
- 当回调函数设置标志为true时返回SUCCESS

#### WaitForNavigationArrival
```cpp
BT::NodeStatus WaitForNavigationArrival::onRunning()
{
    auto& state = GlobalState::getInstance();
    
    // 特殊处理：在等待导航时检查新任务
    // 如果新任务到达，中断导航等待
    bool is_ending = state.isEndTask();
    if (!is_ending && state.hasNewTask()) {
        ROS_INFO("New task arrived while waiting for navigation, interrupting navigation wait");
        return BT::NodeStatus::FAILURE;  // 返回FAILURE让ReactiveFallback检查工作分支
    }
    
    if (state.isNavigationArrived()) {
        ROS_INFO("Navigation arrived at target position");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
```

**设计要点**：
- 在等待导航时也检查新任务，实现任务优先级
- 这允许在导航过程中响应新任务

---

## 五、状态管理机制

### 5.1 线程安全

所有GlobalState的访问都通过互斥锁保护：
```cpp
class GlobalState {
private:
    mutable std::mutex mutex_;
    
public:
    bool hasNewTask() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !task_queue_.empty();
    }
};
```

### 5.2 状态标志管理

#### 完成标志的典型流程
```
1. 发送指令前：ResetXXXCompleted() - 重置标志
2. 发送指令：SendXXXCommand() - 发布到ROS话题
3. 等待完成：WaitForXXXCompletion() - 持续检查标志
4. 外部模块完成：发布到ROS话题
5. 回调函数：setXXXCompleted(true) - 设置标志
6. 等待节点检测到：返回SUCCESS
```

#### 回令标志的典型流程
```
1. 发送指令前：ResetXXXAck() - 重置回令标志
2. 发送指令：SendXXXCommand() - 发布到ROS话题
3. 外部模块完成：发布回令到ROS话题
4. 回调函数：setXXXAck(true) - 设置回令标志
5. 后续节点检查：CheckXXXAck() - 验证是否已执行
```

### 5.3 距离监控机制

```
导航模块 (10Hz)
  └─→ /navigation/distance_to_goal
      └─→ distanceToGoalCallback()
          └─→ GlobalState::setDistanceToGoal(distance)
              └─→ 行为树每10ms执行
                  └─→ CheckDistanceThreshold持续检查
                      └─→ 当距离满足条件时触发动作
```

**关键点**：
- 距离更新频率：10Hz（导航模块发布频率）
- 距离检查频率：10Hz（行为树执行频率）
- 这实现了"时时刻刻监控距离"的要求

---

## 六、任务执行流程

### 6.1 完整任务流程

```
1. 任务接收
   └─→ Service (/liaodou) 或 Topic (/scheduler/start_task)
       └─→ GlobalState::addTask()

2. 任务处理
   └─→ CheckNewTask (检测到任务)
       └─→ GetAndSetCurrentTask (取出任务)
           └─→ 重置所有状态标志
               └─→ KeepRunningUntilFailure (循环执行)
                   └─→ SEQ_ONE_CYCLE (一个完整循环)
                       ├─ SEQ_PREPARE (准备：抬大臂)
                       ├─ SEQ_SCOOP (铲料)
                       ├─ SEQ_DUMP (卸料)
                       └─ CheckForNextCycle (检查下一循环)
                           └─→ 如果有新任务，继续循环
                               └─→ 如果没有，退出循环
```

### 6.2 一个完整循环的详细步骤

#### 步骤1：准备阶段（抬大臂）
```
1. UpdateTaskProgress (phase="preparing")
2. ResetArmRaisedAck
3. ResetWorkDeviceCompleted
4. SendRaiseArmCommand
   └─→ 发布到 /work_device/raise_arm
5. WaitForWorkDeviceCompletion
   └─→ 等待 /work_device/arm_raised 回调
       └─→ armRaisedCallback() 设置标志
           └─→ 返回SUCCESS
```

#### 步骤2：铲料阶段
```
2.1 请求铲料点
    └─→ RequestScoopPoint
        └─→ 调用服务 /spadingPose
            └─→ 设置 scoop_point_

2.2 处理铲料点
    ├─ 如果(0,0)：导航到料仓中心 → 重新请求
    └─ 如果有效：导航到铲料点

2.3 铲料过程（持续监控距离）
    ├─ 距离<=6米：放平铲斗
    │  └─→ SendLowerBucketCommand
    │      └─→ 等待 /work_device/bucket_lowered
    └─ 距离<=3米且已放平铲斗：开始铲料
       └─→ SendScoopCommand
           └─→ 等待 /scoop/scoop_completed
```

#### 步骤3：卸料阶段
```
3.1 导航到料斗
    └─→ SendNavigationGoalHopper
        └─→ 等待导航到达

3.2 卸料过程（持续监控距离）
    ├─ 距离<=6米：抬大臂
    │  └─→ SendRaiseArmCommand
    │      └─→ 等待 /work_device/arm_raised
    └─ 距离<=0米且已抬大臂：卸料
       └─→ SendScoopCommand (语义上是卸料)
           └─→ 等待 /scoop/scoop_completed
```

#### 步骤4：检查下一循环
```
CheckForNextCycle
└─→ 检查 GlobalState::hasNewTask()
    ├─ 如果有：KeepRunningUntilFailure继续循环
    └─ 如果没有：退出循环，返回空闲状态
```

---

## 七、优先级和中断机制

### 7.1 ReactiveFallback优先级

```
ReactiveFallback (ROOT)
├─ SEQ_STOP (最高优先级)
│  └─→ 如果CheckEndTask返回SUCCESS，立即中断其他分支
│
├─ SEQ_PAUSE (中等优先级)
│  └─→ 如果CheckPauseTask返回SUCCESS，中断工作分支
│
└─ SEL_WORK_IDLE (常规优先级)
   └─→ 正常工作流程
```

### 7.2 中断场景

#### 场景1：工作过程中收到停止指令
```
1. 行为树正在执行SEQ_SCOOP（铲料阶段）
2. 调度模块发布 /scheduler/end_task (true)
3. endTaskCallback() 设置 end_task_ = true
4. 下一次tick时，CheckEndTask返回SUCCESS
5. ReactiveFallback立即切换到SEQ_STOP分支
6. 重置所有状态，导航到停车点
```

#### 场景2：导航过程中收到新任务
```
1. WaitForNavigationArrival正在等待导航到达
2. 调度模块发布新任务
3. startTaskCallback() 添加任务到队列
4. WaitForNavigationArrival检测到hasNewTask()
5. 返回FAILURE，中断导航等待
6. ReactiveFallback检查工作分支
7. CheckNewTask返回SUCCESS，开始处理新任务
```

---

## 八、关键设计模式

### 8.1 单例模式
- `GlobalState::getInstance()` - 全局状态管理
- `ConfigManager::getInstance()` - 配置管理
- `ROSTopicManager::getInstance()` - 话题管理

### 8.2 观察者模式
- ROS话题订阅/发布机制
- 回调函数响应外部事件

### 8.3 状态机模式
- 行为树本身就是状态机的实现
- 通过条件节点检查状态
- 通过动作节点执行状态转换

### 8.4 命令模式
- 动作节点封装指令发送
- 通过ROS话题发送命令
- 通过回调函数接收反馈

---

## 九、潜在问题和改进建议

### 9.1 已发现的问题

#### 问题1：SendScoopCommand的语义混淆
- **现状**：铲料和卸料使用同一个节点和话题
- **影响**：通过距离和位置判断，逻辑复杂
- **建议**：可以考虑分离为SendDumpCommand节点

#### 问题2：距离重置时机
- **现状**：注释说明距离重置由导航模块负责
- **风险**：如果导航模块未正确重置，可能导致误判
- **建议**：在发送导航目标时显式重置距离

#### 问题3：任务队列无大小限制
- **现状**：task_queue_是std::queue，无大小限制
- **风险**：如果任务产生速度过快，可能导致内存问题
- **建议**：添加队列大小限制和任务丢弃策略

### 9.2 改进建议

#### 建议1：添加超时机制
- 为每个等待节点添加超时检查
- 超时后执行恢复策略

#### 建议2：添加错误恢复
- 当服务调用失败时，提供重试机制
- 当导航失败时，提供重规划机制

#### 建议3：增强日志记录
- 记录每个节点的执行时间
- 记录状态转换的完整路径
- 记录所有ROS消息的发送/接收时间

---

## 十、总结

### 10.1 系统特点

1. **模块化设计**：各功能模块独立，通过ROS话题通信
2. **响应式执行**：ReactiveFallback确保高优先级指令立即响应
3. **实时监控**：通过持续订阅和检查实现距离实时监控
4. **状态集中管理**：GlobalState作为全局状态中心
5. **配置灵活**：YAML配置文件支持参数调整

### 10.2 核心机制

1. **任务队列机制**：支持多任务排队执行
2. **状态标志机制**：通过标志位同步异步操作
3. **距离监控机制**：10Hz持续监控，实时响应
4. **优先级中断机制**：ReactiveFallback实现优先级响应
5. **循环执行机制**：KeepRunningUntilFailure支持连续任务

### 10.3 执行流程

```
任务接收 → 任务处理 → 准备阶段 → 铲料阶段 → 卸料阶段 → 检查下一循环
   ↓          ↓          ↓          ↓          ↓          ↓
Service    CheckNew  抬大臂    请求铲料点  导航到料斗   CheckForNext
或Topic    Task      等待完成  导航到铲料点  卸料       Cycle
           GetTask   完成      放平铲斗    完成
                     完成      铲料
                               完成
```

这个系统通过行为树实现了复杂的自主装载机器人控制逻辑，具有良好的模块化、可扩展性和实时响应能力。



