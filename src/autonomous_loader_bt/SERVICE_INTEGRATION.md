# 铲料点服务集成说明

## 修改内容

### 1. 创建服务定义文件

**文件**: `src/autonomous_loader_msgs/srv/spadingPose.srv`

```srv
uint8 pileID
---
geometry_msgs/Pose pose_output
```

- **请求**: `pileID` - 料仓编号
- **响应**: `pose_output` - 铲料点的位置（geometry_msgs/Pose格式）

### 2. 更新CMakeLists.txt

**文件**: `src/autonomous_loader_msgs/CMakeLists.txt`

添加了服务文件的生成配置：
```cmake
add_service_files(
  FILES
  spadingPose.srv
)
```

### 3. 扩展ROSTopicManager

**文件**: `src/autonomous_loader_bt/include/autonomous_loader_bt/ros_topic_manager.h`

#### 新增内容：
1. 添加Service客户端：
   ```cpp
   ros::ServiceClient spading_pose_client_;
   ```

2. 添加requestScoopPoint方法：
   ```cpp
   bool requestScoopPoint(uint8_t pileID, geometry_msgs::PoseStamped& result)
   ```
   
   功能：
   - 调用 `/spadingPose` 服务
   - 传入 pileID（料仓编号）
   - 获取铲料点位置
   - 转换为 PoseStamped 格式并返回

### 4. 修改RequestScoopPoint节点

**文件**: `src/autonomous_loader_bt/src/loader_bt_nodes.cpp`

#### 新的实现逻辑：

```cpp
BT::NodeStatus RequestScoopPoint::tick()
{
    1. 获取当前任务的 bin_id 作为 pileID
    2. 调用 topic_manager.requestScoopPoint(pileID, scoop_point)
    3. 如果成功：
       - 将铲料点存储到 GlobalState
       - 返回 SUCCESS
    4. 如果失败：
       - 使用默认位置(0,0)
       - 返回 SUCCESS (以继续流程)
}
```

**关键改进：**
- 不再使用随机生成的位置
- 通过ROS Service获取真实铲料点
- 失败时使用(0,0)作为降级方案

---

## 工作流程

### 完整调用链

```
行为树执行 RequestScoopPoint 节点
         ↓
获取当前任务的 bin_id
         ↓
调用 topic_manager.requestScoopPoint()
         ↓
ROS Service客户端调用 /spadingPose 服务
         ↓
服务端(pile包)处理请求
   - 根据 pileID 计算铲料点
   - 返回 geometry_msgs/Pose
         ↓
客户端接收响应
         ↓
转换为 PoseStamped 格式
         ↓
存储到 GlobalState.scoop_point_
         ↓
行为树继续执行后续节点
```

### 服务调用示例

**服务名称**: `/spadingPose`

**请求格式**:
```cpp
uint8 pileID  // 例如: 1
```

**响应格式**:
```cpp
geometry_msgs/Pose {
    position: {x, y, z}
    orientation: {x, y, z, w}
}
```

**调用示例（Python）**:
```python
from std_srvs.srv import *
import rospy

rospy.wait_for_service('/spadingPose')
service = rospy.ServiceProxy('/spadingPose', SpadingPose)
response = service.call(pileID=1)
pose = response.pose_output
```

---

## 集成要求

### 1. 服务端必须实现

需要在pile包或其他模块中实现 `/spadingPose` 服务：

```cpp
// 服务端示例代码
bool spadingPoseCallback(autonomous_loader_msgs::spadingPose::Request& req,
                         autonomous_loader_msgs::spadingPose::Response& res)
{
    uint8_t pileID = req.pileID;
    
    // 根据 pileID 计算铲料点位置
    geometry_msgs::Pose scoop_pose;
    
    // TODO: 实现铲料点计算逻辑
    // 例如：
    // scoop_pose.position.x = calculate_x(pileID);
    // scoop_pose.position.y = calculate_y(pileID);
    
    res.pose_output = scoop_pose;
    
    return true;
}

// 注册服务
ros::ServiceServer service = nh.advertiseService("/spadingPose", spadingPoseCallback);
```

### 2. 编译步骤

```bash
# 1. 编译消息包
cd /home/guo/shantui/behaviorTreev4_ws
catkin build autonomous_loader_msgs

# 2. 编译行为树包
catkin build autonomous_loader_bt

# 3. Source环境
source devel/setup.bash
```

### 3. 测试

**测试服务调用**:
```bash
# 启动服务端（pile包）
rosrun pile your_node

# 测试服务调用
rosservice call /spadingPose "pileID: 1"
```

**测试完整流程**:
```bash
# 1. 启动行为树节点
roslaunch autonomous_loader_bt autonomous_loader_bt.launch

# 2. 发送任务
rostopic pub /scheduler/start_task autonomous_loader_msgs/TaskCommand "
task_id: 1
bin_id: 1
hopper_id: 1
task_type: 'scoop'"

# 3. 观察日志
# 应该看到：成功获取铲料点位置
```

---

## 故障排除

### 问题1: Service not available

**症状**:
```
ERROR: Service [/spadingPose] not available
```

**解决方案**:
- 确保pile包的服务端已启动
- 检查服务名称是否正确

### 问题2: Service调用超时

**症状**:
```
ROS_ERROR: 获取铲料点失败: pileID=1
```

**解决方案**:
- 检查网络连接
- 确认服务端正常响应
- 系统会自动使用(0,0)作为降级方案

### 问题3: 消息类型不匹配

**症状**:
```
undefined reference to 'autonomous_loader_msgs::spadingPose'
```

**解决方案**:
```bash
# 重新编译消息包
catkin build autonomous_loader_msgs
source devel/setup.bash
```

---

## 修改总结

### 文件清单

1. ✅ `src/autonomous_loader_msgs/srv/spadingPose.srv` - 新建
2. ✅ `src/autonomous_loader_msgs/CMakeLists.txt` - 修改
3. ✅ `src/autonomous_loader_bt/include/autonomous_loader_bt/ros_topic_manager.h` - 修改
4. ✅ `src/autonomous_loader_bt/src/loader_bt_nodes.cpp` - 修改

### 关键特性

- ✅ 通过ROS Service获取真实铲料点
- ✅ 失败时有降级方案（使用0,0）
- ✅ 使用料仓ID作为pileID
- ✅ 完全集成到现有行为树流程
- ✅ 线程安全（通过单例模式）

---

## 下一步

1. **实现服务端** - 在pile包中实现 `/spadingPose` 服务
2. **测试集成** - 验证服务调用是否正常
3. **优化错误处理** - 根据实际使用情况调整失败处理逻辑
4. **性能优化** - 如果需要，可以添加缓存机制

---

# 导航 Action / 话题配合说明

## 场景概述

行为树（autonomous_loader_tree.xml）通过 `ROSTopicManager` 调度导航模块，完成以下阶段：

1. 去料仓中心 / 最终铲料点
2. 去料斗卸料
3. 去停车位或下一任务

为保证流程顺利，需要导航模块配合发布若干关键状态，并支持暂停、重新规划等控制。以下文档是导航模块对接行为树的唯一信息源，开发导航 Action/节点时务必遵循。

## Action 接口（推荐沿用 `/move_base`）

- **Action 名称**：`/move_base`
- **类型**：`move_base_msgs/MoveBaseAction`
- **Goal**：行为树调用 `SendNavigationGoal*` 节点时发送（Bin 中心、铲料点、料斗或停车点 pose）
- **Result**：
  - 成功：`set_succeeded`，并在同一时刻对 `navigation/arrived` 发布 `True`
  - 失败：`set_aborted`，必要时对 `navigation/planning_success` 发布 `False`

> 若需自定义 Action，可定义 `autonomous_loader_msgs/NavigateToPose.action`，但仍需遵守下方的话题协议。

## 必要话题（导航 → 行为树）

| 话题 | 类型 | 频率/属性 | 说明 |
| --- | --- | --- | --- |
| `/autonomous_loader_bt_node/navigation/planning_success` | `std_msgs/Bool` | 触发事件，latched | 规划成功时立即发布 `True`。若规划失败，可发布 `False` |
| `/autonomous_loader_bt_node/navigation/distance_to_goal` | `std_msgs/Float64` | 5~10 Hz | 目标距离（米）。行为树利用该值触发“≤3m 放平铲斗”“≤7m 抬臂”等动作 |
| `/autonomous_loader_bt_node/navigation/arrived` | `std_msgs/Bool` | 事件，latched | 抵达目标时发布 `True`（一次即可）。行为树在下一段导航开始前会重置 |

### 发布时序要求

1. **接到 goal 后** 200ms 内：发布 `planning_success=True`
2. **规划成功后**：开始循环发布 `distance_to_goal`，值需单调递减到 0
3. **抵达判定**：当机器人实际到达目标（或 remaining_distance ≤ 0.1m），发布 `arrived=True` 并 `set_succeeded`

### 距离计算建议

- 基于定位数据（如 amcl/odom）与目标 pose 的欧氏距离
- 若暂时无法获得真实距离，至少提供一个线性递减（参照行驶速度）以驱动行为树流程

## 控制话题（行为树 → 导航）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/autonomous_loader_bt_node/navigation/pause` | `std_msgs/Bool` | `True`：暂停/减速；`False`：恢复。需保留当前 goal |
| `/autonomous_loader_bt_node/navigation/cancel` *(可选)* | `std_msgs/Bool` | `True`：取消当前 goal，与 Action preempt 等效 |

行为树在每段导航前还会调用：

- `ResetNavigationArrived`：表明新一次导航开始，可把内部状态重置为“尚未到达”
- `ResetDistanceToGoal`：建议导航模块在收到新 goal 时立即发布第一帧 `distance_to_goal`

## 状态机依赖点

1. **距离阈值联动**：
   - `lower_bucket_distance`（默认 3.0m）：distance≤3m 时放平铲斗 (=3)
   - `raise_arm_distance`（默认 7.0m）：distance≤7m 时抬臂 (=4)
   - `dump_guard_distance`（默认 4.0m）：若 distance≤4m 仍未抬臂会触发保护逻辑

2. **到达判定**：
   - 行为树等待 `/navigation/arrived` 为 True 才继续下一阶段（铲料、卸料、返回等）
   - 每次新导航开始前，BT 会先调用 `ResetNavigationArrived`，导航模块无需主动拉低该话题

3. **反复请求铲料点**：
   - 如果 `/spadingPose` 返回 (0,0)，行为树会先导航到料仓中心，再重新请求；导航模块需能连续处理多个 goal

## 建议的导航 Action 流程（伪代码）

```python
def execute_cb(goal):
    publish_planning_success(True)
    start_distance_timer()
    while not arrived:
        if pause_cmd:
            hold_velocity()
            continue
        update_distance_to_goal()
        if new_goal_preempted:
            set_preempted()
            return
    publish_distance_to_goal(0.0)
    publish_arrived(True)
    set_succeeded()
```

## 联调步骤

1. 启动行为树：`roslaunch autonomous_loader_bt autonomous_loader_bt.launch`
2. 启动导航模块（实现上述接口）
3. 使用 `interactive_tester.py` 连续发送任务
4. 观察行为树日志是否顺利经过：
   - “Navigating to bin center / scoop point / hopper / parking”
   - “distance_to_goal” 触发放平/抬臂动作
   - “Wait for navigation arrival” 迅速结束

若需要 mock 示例，可将 `interactive_tester.py` 中的 `MoveBaseMockServer` 拆出单独运行，并按此文档补齐话题，即可模拟真实导航。

## 常见问题

| 症状 | 可能原因 | 解决方案 |
| --- | --- | --- |
| BT 长时间停在 “Wait for navigation arrival” | 未发布 `/navigation/arrived` | 抵达后立即发布 Bool(True) |
| 放平/抬臂没有触发 | 未发布 `/navigation/distance_to_goal` 或频率太低 | 保证 ≥5Hz，且距离递减到触发阈值 |
| 规划成功后仍提示“等待Planning Success” | 未及时发布 `planning_success` | 在接到 goal 后 200ms 内发送 latched True |
| 暂停指令无效 | 未订阅 `/navigation/pause` | 订阅该话题并在 True 时暂停动作 |

---

