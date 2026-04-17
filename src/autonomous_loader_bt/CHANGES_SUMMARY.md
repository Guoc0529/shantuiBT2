# 自主装载机器人行为树修改总结

## 修改内容

### 0. 新增开始任务订阅（2024-12-19）

添加了接收调度模块发送的任务指令的功能：

#### 修改文件
- `src/autonomous_loader_bt/src/autonomous_loader_bt_node.cpp`
  - 添加 `#include "autonomous_loader_msgs/TaskCommand.h"`
  - 添加 `start_task_sub_` 订阅者
  - 添加 `startTaskCallback()` 回调函数
  - 订阅话题：`scheduler/start_task` (类型: `TaskCommand`)

- `src/autonomous_loader_bt/CMakeLists.txt`
  - 添加 `AUTONOMOUS_LOADER_MSGS_LIBRARY` 依赖
  - 链接 `autonomous_loader_msgs` 库

- `src/autonomous_loader_bt/scripts/send_task_example.py`
  - 创建示例脚本，演示如何发送任务

#### 使用方法
```python
# 通过ROS发布任务
from autonomous_loader_msgs.msg import TaskCommand

pub = rospy.Publisher('scheduler/start_task', TaskCommand, queue_size=10)
task = TaskCommand()
task.task_id = 1
task.bin_id = 1
task.hopper_id = 1
task.task_type = "scoop"
pub.publish(task)
```

或者使用提供的脚本：
```bash
rosrun autonomous_loader_bt send_task_example.py
```

### 1. 新增状态管理功能

在 `GlobalState` 类中添加了以下新功能：
- **状态重置方法**：
  - `resetWorkDeviceCompleted()` - 重置工作装置完成标志
  - `resetScoopCompleted()` - 重置铲料完成标志
  - `resetNavigationArrived()` - 重置导航到达标志
  - `resetNavigationPlanningSuccess()` - 重置导航规划成功标志
  - `resetDistanceToGoal()` - 重置距离目标标志

- **应答标志管理**：
  - `setArmRaisedAck()/isArmRaisedAck()/resetArmRaisedAck()` - 抬大臂应答标志
  - `setBucketLoweredAck()/isBucketLoweredAck()/resetBucketLoweredAck()` - 放平铲斗应答标志

- **循环计数**：
  - `getCycleCount()` - 获取当前循环次数
  - `incrementCycleCount()` - 增加循环次数
  - `resetCycleCount()` - 重置循环次数

### 2. 新增行为树节点

#### 条件节点
- **CheckDistanceGreaterThan** - 检查距离是否大于阈值（用于避免重复执行）
- **CheckArmRaisedAck** - 检查抬大臂应答标志
- **CheckBucketLoweredAck** - 检查放平铲斗应答标志

#### 动作节点
- **GetAndSetCurrentTask** - 获取并设置当前任务
- **ResetWorkDeviceCompleted** - 重置工作装置完成标志
- **ResetScoopCompleted** - 重置铲料完成标志
- **ResetNavigationArrived** - 重置导航到达标志
- **ResetNavigationPlanningSuccess** - 重置导航规划成功标志
- **ResetArmRaisedAck** - 重置抬大臂应答标志
- **ResetBucketLoweredAck** - 重置放平铲斗应答标志

### 3. 修改行为树结构

根据需求规范，重新设计了行为树的工作流程：

#### 主流程
1. **处理停止指令**（最高优先级）
   - 重置所有状态标志
   - 导航到停车点

2. **处理暂停指令**（中等优先级）
   - 暂停导航

3. **正常工作流程**（常规优先级）
   - 获取任务
   - 执行铲料和卸料循环

#### 铲料流程
1. 抬大臂并等待完成
2. 请求铲料点
3. 根据铲料点结果：
   - 如果铲料点为(0,0)：
     - 导航到料仓中心
     - 到达后重新请求铲料点
   - 如果铲料点非(0,0)：
     - 直接导航到铲料点
4. 距离监控：
   - 距离<=6米：放平铲斗并等待完成
   - 距离<=3米且已放平铲斗：执行铲料并等待完成

#### 卸料流程
1. 导航到料斗
2. 距离监控：
   - 距离<=6米：抬大臂并等待完成
   - 距离<=0米且已抬大臂：执行卸料并等待完成

### 4. 修改ROS回调

在 `autonomous_loader_bt_node.cpp` 中更新了以下回调：
- `armRaisedCallback()` - 设置抬大臂应答标志
- `bucketLoweredCallback()` - 设置放平铲斗应答标志

### 5. 状态重置机制

在关键执行点添加了状态重置：
- 任务开始时重置所有完成标志
- 每个阶段开始前重置相应的完成标志
- 确保状态正确过渡

### 6. 距离持续监控

通过行为树的重复执行机制，实现了距离的持续监控：
- 使用 `CheckDistanceThreshold` 节点检查距离是否满足条件
- 使用 `CheckDistanceGreaterThan` 节点避免重复触发
- 条件满足后才执行相应的动作

## 符合的功能模块

### 1. 调度功能 ✓
- ✓ 开始任务（从YAML文件读取任务）
- ✓ 暂停任务（导航暂停）
- ✓ 结束任务（导航回到起点）

### 2. 工作装置 ✓
- ✓ 是否完成任务（通过回令确认）

### 3. 铲料模块 ✓
- ✓ 是否完成任务（通过回令确认）

### 4. 导航模块 ✓
- ✓ 是否规划成功
- ✓ 是否到位
- ✓ 距离目标点的距离（持续监控）

## 工作流程实现

### 铲料过程（按需求实现）
1. ✓ 调度下发开始任务 → 状态机
2. ✓ 状态机收到任务后，抬大臂 → 工作装置模块
3. ✓ 收到工作装置完成回令后，查询铲料点 → 铲料点计算模块
4. ✓ 根据铲料点结果：
   - a. 收到(0,0)：导航到料仓中点 → 重新请求铲料点
   - b. 收到非(0,0)：直接导航到铲料点
5. ✓ 距离<=6米：放平铲斗 → 工作装置
6. ✓ 距离<=3米且收到放平铲斗回令：铲料 → 铲料模块
7. ✓ 铲料完成

### 卸料过程（按需求实现）
1. ✓ 收到铲料完成回令后，导航到料斗位置
2. ✓ 距离<=6米：抬大臂 → 工作装置
3. ✓ 距离<=0米且收到抬大臂回令：卸料 → 铲料模块
4. ✓ 卸料完成

### 第二轮任务（按需求实现）
1. ✓ 卸料完成后检查是否有新任务
2. ✓ 有则继续执行任务循环
3. ✓ 无则导航到过道停车点

## 技术特点

1. **状态管理**：使用单例模式的全局状态管理
2. **线程安全**：使用互斥锁保护共享资源
3. **响应式设计**：使用ReactiveSelector确保高优先级指令可立即响应
4. **状态重置**：在关键时刻重置标志，确保状态正确
5. **距离监控**：通过行为树的重复执行机制实现持续监控
6. **回令确认**：通过应答标志确保动作完成后再继续

## 配置文件

配置文件的参数：
- `lower_bucket_distance: 6.0` - 放平铲斗距离（米）
- `start_scoop_distance: 3.0` - 开始铲料距离（米）
- `raise_arm_distance: 6.0` - 抬大臂距离（米）
- `start_dump_distance: 0.0` - 开始卸料距离（米）

## 使用方法

1. 准备任务：通过YAML文件或ROS话题添加任务
2. 启动节点：`roslaunch autonomous_loader_bt autonomous_loader_bt.launch`
3. 任务会自动从队列中取出并执行
4. 支持暂停和停止指令

## 注意事项

1. 距离阈值可在配置文件中调整
2. 所有模块的回令都通过ROS话题传递
3. 行为树会持续监控状态变化
4. 确保所有模块都正确配置ROS话题

