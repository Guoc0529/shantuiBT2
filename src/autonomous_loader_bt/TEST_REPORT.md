# 状态话题测试报告

**测试时间**: 2026-04-23
**测试人员**: AI Assistant
**测试目标**: 验证 `/bt_action/status` 和 `/vehicle/task_status` 两个话题的功能

---

## 测试环境

- ROS 版本: Noetic
- 工作空间: `/home/wh/shantui/behaviorTreev4_ws1/behaviorTreev4_ws`
- 测试节点: `autonomous_loader_bt_node`

---

## 测试结果总结

| 测试项 | 状态 | 说明 |
|--------|------|------|
| `/bt_action/status` | :warning: 部分正常 | 话题存在，但发布时机需要触发 |
| `/vehicle/task_status` | :x: 未发布 | 话题无发布者，初始化可能有问题 |

---

## 详细测试结果

### 1. `/bt_action/status`

**话题信息**:
- 类型: `std_msgs/String`
- 状态: 有发布者

**测试结果**:
- 话题可以正常订阅
- 消息格式: 动作状态描述字符串
- 需要任务下发后才会触发发布

### 2. `/vehicle/task_status`

**话题信息**:
- 类型: `std_msgs/Int32MultiArray`
- 状态: **无发布者**

**问题分析**:
`TaskStatusReporter::init()` 方法应该在节点初始化时调用，但实际检查发现 `/vehicle/task_status` 没有发布者。

可能原因:
1. `TaskStatusReporter::init()` 未被正确调用
2. 初始化顺序问题
3. 内存分配错误（`malloc(): invalid size`）

---

## 代码分析

### `TaskStatusReporter` 类位置
- 头文件: `include/autonomous_loader_bt/loader_bt_nodes.h` (第 176-274 行)

### 初始化调用
- 文件: `src/autonomous_loader_bt_node.cpp` (第 67 行)
- 代码: `autonomous_loader_bt::TaskStatusReporter::instance().init(nh_);`

### 话题配置
```cpp
status_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/vehicle/task_status", 10, true);
```

### 消息格式
| 索引 | 字段 | 说明 |
|------|------|------|
| [0] | state | 状态码 (0-8) |
| [1] | timestamp | Unix 时间戳 |
| [2] | task_id | 任务编号 (-1 表示无) |
| [3] | reserved | 预留字段 |

### 状态码定义
| 值 | 状态名 | 说明 |
|----|--------|------|
| 0 | NOT_STARTED | 未启动 |
| 1 | IDLE | 空闲中 |
| 2 | AUTO_WORKING | 自动作业中 |
| 3 | TEMP_WORKING | 临时作业中 |
| 4 | EMERGENCY_STOP | 紧急停车中 |
| 5 | OBSTACLE_STOP | 避障停车中 |
| 6 | REMOTE_CONTROL | 远程接管中 |
| 7 | ENDING | 结束作业中 |
| 8 | ENDED | 已结束 |

---

## 手动测试指南

### 终端 1: 启动 ROS 核心
```bash
roscore
```

### 终端 2: 启动主节点
```bash
cd /home/wh/shantui/behaviorTreev4_ws1/behaviorTreev4_ws
source devel/setup.bash
roslaunch autonomous_loader_bt autonomous_loader_bt.launch
```

### 终端 3: 监听状态话题
```bash
cd /home/wh/shantui/behaviorTreev4_ws1/behaviorTreev4_ws
source devel/setup.bash

# 监听动作状态
rostopic echo /bt_action/status

# 监听任务状态（需要先修复发布问题）
rostopic echo /vehicle/task_status
```

### 终端 4: 下发任务
```bash
cd /home/wh/shantui/behaviorTreev4_ws1/behaviorTreev4_ws
source devel/setup.bash

# 调用任务服务
rosservice call /autonomous_loader_bt_node/liaodou "taskID: 1" "cang: 1" "dou: 1"
```

---

## 发现的问题

### 问题 1: `/vehicle/task_status` 无发布者

**现象**: `rostopic info /vehicle/task_status` 显示 `Publishers: None`

**影响**: 任务状态无法上报

**建议排查**:
1. 检查节点启动日志是否有 `[TaskStatusReporter] Initialized` 输出
2. 检查是否有 `malloc(): invalid size` 错误
3. 在 `TaskStatusReporter::init()` 中添加更多调试日志

### 问题 2: taskID 范围限制

**现象**: `cangdou.srv` 中 `taskID` 是 `int8` 类型，范围为 -128 到 127

**当前测试脚本使用**:
```python
task_id=1001  # 超出 int8 范围！
```

**已修复为**:
```python
task_id=1  # 有效范围
```

---

## 测试脚本

已创建测试脚本: `scripts/test_status_topics.py`

**功能**:
- 自动监听 `/bt_action/status` 和 `/vehicle/task_status`
- 自动下发测试任务
- 自动模拟导航回令
- 生成测试报告

**使用方法**:
```bash
cd /home/wh/shantui/behaviorTreev4_ws1/behaviorTreev4_ws
source devel/setup.bash
rosrun autonomous_loader_bt test_status_topics.py --duration 30
```

---

## 下一步建议

1. **排查 `/vehicle/task_status` 初始化问题**
   - 在 `TaskStatusReporter::init()` 中添加 ROS_INFO 日志
   - 检查是否有内存相关错误

2. **验证任务下发流程**
   - 使用 `rosservice call` 手动下发任务
   - 观察 `/bt_action/status` 是否有输出

3. **检查 CHANGELOG.md 记录的功能**
   - 动作状态消息发布功能
   - 任务状态上报功能

---

## 附录: 状态上报调用链

```
任务下发 (rosservice call)
    ↓
liaodou service callback
    ↓
GlobalState::addTask()
GlobalState::setTaskId()
TaskStatusReporter::setTaskId()
    ↓
行为树执行
    ↓
SetWorkState BT Node
    ↓
TaskStatusReporter::setState()
    ↓
TaskStatusReporter::report()
    ↓
发布到 /vehicle/task_status
```
