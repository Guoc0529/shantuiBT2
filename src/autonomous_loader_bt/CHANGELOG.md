# 代码修改记录

## 2026-05-12: 完善工作状态控制与行为树停止逻辑

### 需求
完善 TaskCtrl 指令控制，实现：
1. 急停(4)、避障停车(5)、远程接管(6) 立即停止行为树
2. 结束任务(6) 等待当前任务完成后停止行为树
3. 根据任务类型正确设置工作状态
4. 正常任务完成后变为空闲，保持2秒后开始新任务

### 状态定义

| 值 | 含义 | 触发时机 |
|---|------|---------|
| 1 | 空闲中 | 启动时、任务完成后等待新任务 |
| 2 | 自动作业中 | 任务开始(type=0) |
| 3 | 临时作业中 | 任务开始(type=1) |
| 4 | 紧急停车中 | ctrlCmd=4，**立即 haltTree()** |
| 5 | 避障停车中 | ctrlCmd=3，**立即 haltTree()** |
| 6 | 远程接管中 | ctrlCmd=2，**立即 haltTree()** |
| 7 | 结束作业中 | ctrlCmd=6，等待当前任务完成 |
| 8 | 已结束 | 任务完成后自动设置，haltTree() |

### 正常任务流程状态变化

```
任务完成 → state=1 (空闲)，计时2秒
   ↓
[2秒内有新任务] → state=2/3，开始执行
[2秒后无新任务] → state=1 (保持空闲)
```

### 修改文件

#### 1. `include/autonomous_loader_bt/loader_bt_nodes.h`

**GlobalState 类新增方法:**
```cpp
void setEmergencyStop(bool stop);       // 设置急停标志
void setIsEnding(bool ending);         // 设置"结束中"标志
void setHaltRequested(bool halt);       // 请求停止行为树
bool isEmergencyStop() const;           // 获取急停标志
bool isEnding() const;                 // 获取"结束中"标志
bool isHaltRequested() const;          // 获取停止请求
void clearHaltRequested();              // 清除停止请求
```

**GlobalState 类新增成员变量:**
```cpp
bool emergency_stop_ = false;  // 急停标志
bool is_ending_ = false;       // "结束中"标志
bool halt_requested_ = false;  // 停止请求标志
```

#### 2. `src/loader_bt_nodes.cpp`

**新增方法实现:**
- `GlobalState::setEmergencyStop()`
- `GlobalState::setIsEnding()`
- `GlobalState::setHaltRequested()`
- `GlobalState::isEmergencyStop()`
- `GlobalState::isEnding()`
- `GlobalState::isHaltRequested()`
- `GlobalState::clearHaltRequested()`

#### 3. `src/autonomous_loader_bt_node.cpp`

**新增成员变量:**
```cpp
ros::Time idle_start_time_;        // 空闲计时开始时间
bool idle_timer_started_ = false;  // 空闲计时是否已开始
```

**executeTree 修改:**
```cpp
// 每次 tick 前检查是否需要停止
if (gs.isHaltRequested()) {
    tree_.haltTree();
    gs.clearHaltRequested();
    idle_timer_started_ = false;
}

// tick 行为树...

// 正常任务完成：设置 state=1，保持2秒
if (status == IDLE || SUCCESS) {
    if (!idle_timer_started_ && !gs.isEnding() && !gs.isPauseTask()) {
        idle_start_time_ = ros::Time::now();
        idle_timer_started_ = true;
        TaskStatusReporter::instance().setState(TaskStatusReporter::IDLE);
        ROS_INFO("[BT_ROOT] Task completed - state=1 (IDLE), 2s timer started");
    }
    // 2秒后有新任务则设为 2/3，无新任务保持 1
}

// 结束流程：is_ending_=true 时完成当前任务后 haltTree() 并设 state=8
```

### 执行流程

**急停/避障/接管流程 (4/5/6):**
```
1. 收到 ctrlCmd=2/3/4
2. 设置 halt_requested_=true
3. 下次 tick: tree_.haltTree() 停止所有动作
4. 状态上报为 6/5/4
5. 收到 ctrlCmd=5: 清除标志，树恢复执行
```

**结束任务流程 (7→8):**
```
1. 收到 ctrlCmd=6
2. 设置 end_task_=true, is_ending_=true
3. 状态上报为 7 (结束作业中)
4. 树继续执行完当前任务
5. 任务完成后: tree_.haltTree()
6. 状态上报为 8 (已结束)
```

**正常任务完成流程:**
```
1. 任务完成，tree_ 返回 IDLE
2. 设置 state=1 (空闲)，启动2秒计时
3. [2秒内有新任务] → cangdou 回调设 state=2/3
4. [2秒后无新任务] → state=1 保持不变
```

---

## 2026-05-10: 新增 TaskCtrl 指令控制模块

### 需求
支持云端通过 `/task_ctrl_command` 话题下发控制指令，实现任务启动、暂停、继续、结束等功能。

### Topic 列表

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/task_ctrl_command` | shuju/TaskCtrl | 订阅 | 云端下发控制指令 |

### `shuju/TaskCtrl` 消息格式

```bash
# 字段说明
int32 taskID          # 任务编号
int32 ctrlCmd         # 控制指令码 (1-6)
                      #   1: 开始任务
                      #   2: 远程接管
                      #   3: 避障停车
                      #   4: 紧急停车
                      #   5: 继续任务
                      #   6: 结束任务
float64 target_x      # 目标X坐标（米），仅当 ctrlCmd=3 时有效
float64 target_y      # 目标Y坐标（米），仅当 ctrlCmd=3 时有效
float64 target_yaw    # 目标航向角（弧度），仅当 ctrlCmd=3 时有效
bool has_location     # 位置信息有效性标志
float64 timestamp     # Unix 时间戳（秒）

```

### 实现内容

1. **GlobalState 新增 start_task_ 标志**
   - `setStartTask(bool start)` - 设置开始标志
   - `isStartTask() const` - 获取开始标志

2. **新增 CheckStartTask BT 节点**
   - 等待 `start_task_=true` 后返回 SUCCESS
   - 用于阻塞任务执行，直到收到 ctrlCmd=1

3. **订阅 /task_ctrl_command 话题**
   - ctrlCmd=1: 设置 `start_task_=true`
   - ctrlCmd=2: 设置 `pause_task_=true`（远程接管）
   - ctrlCmd=3: 设置 `pause_task_=true`（避障停车）
   - ctrlCmd=4: 急停（待实现）
   - ctrlCmd=5: 设置 `pause_task_=false`（继续任务）
   - ctrlCmd=6: 设置 `end_task_=true`（结束任务）

4. **行为树更新**
   - 在 `SEQ_TASK_LOOP` 中添加 `<CheckStartTask/>` 节点
   - 位于 `CheckNewTask` 之前，确保任务入队后等待开始指令

### 使用流程

```
1. shuchuan 下发任务 → /liaodou 服务 → 任务入队 (hasNewTask=true)
2. shuchuan 下发控制指令 → /task_ctrl_command (ctrlCmd=1)
3. CheckStartTask 返回 SUCCESS
4. CheckNewTask 返回 SUCCESS
5. 任务开始执行
```

---

## 2026-04-27: 添加通用人工接管模块

### 需求
将分散在行为树各处的「发现失败 → 人工接管 → 等待完成 → 继续重试」逻辑抽象为一个通用子树模块，在任意失败处只需插入一行 `<SubTree>` 标签即可复用。

### 设计目标
- 通用性：同一子树可用于导航规划失败、臂铲动作失败、卸料动作失败等多种场景
- 零代码耦合：调用方只需传 `phase`（阶段）和 `message`（提示信息）
- 上位机透明：状态机主动上报接管请求（含车辆位置），上位机负责 UI 提示和操作员交互

### Topic 列表

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/bt_override/request` | std_msgs/String (JSON) | 发布 | 人工接管请求，包含 phase、message、车辆位姿、时间戳 |
| `/bt_override/vehicle_pose` | geometry_msgs/PoseStamped | 发布 | 等待接管期间定期上报车辆位置（每 1s） |
| `/autonomous_loader/manual_intervention_required` | param (bool) | set | 人工接管进行中标志 |
| `/autonomous_loader/manual_intervention_complete` | std_msgs/Bool | 订阅 | 上位机发送 true = 接管完成；车辆解除急停，重新发起规划 |

### `/bt_override/request` 消息格式 (JSON)

```json
{
  "phase": "导航-铲料点",
  "message": "路径规划失败，请手动将车辆移动到合适位置后点击完成",
  "timestamp": 1713849600.123,
  "vehicle_pose": {
    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

### 流程说明

```
规划失败（WaitForPlanningFeedback 返回 FAILURE）
    │
    ▼
┌─────────────────────────────────────────────┐
│  ManualOverride 子树（通用接管流程）           │
│                                             │
│  1. PublishOverrideRequest                   │
│     → 发布请求 JSON 到 /bt_override/request  │
│                                             │
│  2. RequestManualIntervention               │
│     → 取消导航 + 发急停 + 设 manual_        │
│       intervention_required=true            │
│                                             │
│  3. WaitForManualOverride                   │
│     → 等待 /manual_intervention_complete    │
│     → 超时（默认 600s）返回 FAILURE          │
│     → 接管完成：清除参数 + 解除急停           │
│                                             │
│  4. AlwaysFailure                           │
│     → 返回 FAILURE                          │
└─────────────────────────────────────────────┘
    │
    ▼
外层 RetryUntilSuccessful 重试
    │
    ▼
ResetNavigationArrived / ResetPlanningFeedback 等重置
    │
    ▼
重新 SendNavigationGoal（从当前位置重新规划）
```

### 通用子树调用方式

```xml
<!-- 任意需要接管的地方，插入一行即可 -->
<SubTree ID="ManualOverride"
         phase="导航-铲料点"
         message="路径规划失败，请手动将车辆移动到合适位置后点击完成"
         override_timeout="600"/>
```

参数说明：
- `phase`：当前作业阶段，用于上位机区分场景
- `message`：显示给操作员的信息
- `override_timeout`：接管超时时间（秒），超时后返回 FAILURE 进入外层错误处理

### 修改文件

#### 1. `trees/manual_override_subtree.xml` (新增)
新建通用人工接管子树，包含 4 个节点：PublishOverrideRequest → RequestManualIntervention → WaitForManualOverride → AlwaysFailure。

#### 2. `include/autonomous_loader_bt/loader_bt_nodes.h`
新增两个类声明：
- `PublishOverrideRequest`：发布接管请求 JSON 到 /bt_override/request
- `WaitForManualOverride`：等待接管完成，支持超时，定期上报车辆位置

#### 3. `src/loader_bt_nodes.cpp`
实现新增节点，并注册到 BehaviorTreeFactory：
```cpp
factory.registerNodeType<WaitForManualOverride>("WaitForManualOverride");
factory.registerNodeType<PublishOverrideRequest>("PublishOverrideRequest");
```

#### 4. `src/autonomous_loader_bt_node.cpp`
注册通用子树文件：
```cpp
factory.registerBehaviorTreeFromFile(getPackagePath() + "/trees/manual_override_subtree.xml");
```

#### 5. `trees/autonomous_loader_tree.xml` (SafeNavigate 子树)
将规划失败处理从冗余的 Sequence/Fallback 结构替换为一行 `<SubTree>` 调用：
```xml
<!-- 替换前（约 12 行） -->
<Sequence>
    <RequestManualIntervention message="Path planning failed..."/>
    <Fallback>
        <WaitForManualIntervention/>
        <Sequence><CheckEndTask/><AlwaysFailure/></Sequence>
    </Fallback>
    <AlwaysFailure/>
</Sequence>

<!-- 替换后（1 行） -->
<SubTree ID="ManualOverride"
         phase="导航"
         message="路径规划失败，请手动将车辆移动到合适位置后点击完成"
         override_timeout="600"/>
```

### 待扩展场景
以下场景可在后续迭代中通过同样方式接入通用接管子树：
| 场景 | phase | 调用位置 |
|------|-------|---------|
| 铲料点规划失败 | 导航-铲料点 | SEQ_SCOOP_POINT_NAV |
| 卸料仓规划失败 | 导航-卸料仓 | PARALLEL_DUMP_NAV_AND_RAISE |
| 到达质量不达标 | 到达质量 | SafeNavigate Fallback |
| 臂铲动作超时 | 臂铲动作 | SEQ_SCOOP_ACTIONS_ON_APPROACH |
| 卸料动作超时 | 卸料动作 | SEQ_DUMP_ON_ARRIVAL |

---

## 2026-04-17: 添加动作状态消息发布功能

### 需求
在行为树执行过程中，当动作开始时发送"进行中"消息，动作完成时发送"完成"消息。

### Topic
`/bt_action/status` (std_msgs/String)

### 消息列表

| 触发时机 | 发送消息 |
|---------|---------|
| SetArmBucketState code=1 | 铲斗放平到地面进行中 |
| SetArmBucketState code=2 | 铲斗抬到运输位置进行中 |
| SetArmBucketState code=3 | 铲斗放平进行中 |
| SetArmBucketState code=4 | 举升大臂进行中 |
| SetArmBucketState code=5 | 降大臂进行中 |
| SetArmBucketState code=6 | 卸料进行中 |
| SetArmBucketState code=7 | 铲料进行中 |
| SetArmBucketState code=8 | 抖料进行中 |
| WaitForArmBucketCompletion 成功 | 对应动作完成消息 |
| SendNavigationGoal | 导航进行中 |
| WaitForNavigationResult 成功 | 导航到达 |
| WaitForPointAccumulationDone 成功 | 点云累积完成 |

### 修改文件

#### 1. `src/autonomous_loader_bt/include/autonomous_loader_bt/ros_topic_manager.h`

**添加内容:**
```cpp
// 在 initialize() 方法中添加:
ros::Publisher action_status_pub_ = nh.advertise<std_msgs::String>("/bt_action/status", 10, true);

// 添加新方法:
void publishActionStatus(const std::string& message)
{
    std_msgs::String msg;
    msg.data = message;
    action_status_pub_.publish(msg);
    ROS_INFO("[BT_ACTION] %s", message.c_str());
}
```

#### 2. `src/autonomous_loader_bt/include/autonomous_loader_bt/loader_bt_nodes.h`

**添加内容:**
```cpp
// 在 GlobalState 类中添加方法声明:
void setArmBucketCode(int code);
int getArmBucketCode() const;

// 添加成员变量:
int arm_bucket_code_ = 0;
```

#### 3. `src/autonomous_loader_bt/src/loader_bt_nodes.cpp`

**修改 SetArmBucketState::tick():**
- 保存 code 到 GlobalState
- 根据 code 发送对应的"进行中"消息

**修改 WaitForArmBucketCompletion::onRunning():**
- 成功时根据 GlobalState 中保存的 code 发送"完成"消息

**修改 SendNavigationGoal::tick():**
- 发送"导航进行中"消息

**修改 WaitForNavigationResult::onRunning():**
- 导航成功时发送"导航到达"消息

**修改 WaitForPointAccumulationDone::onRunning():**
- 成功时发送"点云累积完成"消息

**添加新方法:**
```cpp
void GlobalState::setArmBucketCode(int code)
{
    std::lock_guard<std::mutex> lock(mutex_);
    arm_bucket_code_ = code;
}

int GlobalState::getArmBucketCode() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_bucket_code_;
}
```

---

## 2026-04-17: 优化障碍物距离计算（基于矩形最近点）

### 需求
之前使用障碍物质心到原点计算距离不准确，现在改为计算车辆到障碍物最近点的距离。

### 修改文件

#### `src/obstacle_avoider/src/obstacle_avoider_node.cpp`

**新增结构体:**

```cpp
// 2D 点结构
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

// 旋转矩形结构（有朝向的矩形）
struct OrientedRect {
    Point2D center;   // 中心点
    double yaw;       // 朝向角（弧度）
    double length;   // 长
    double width;    // 宽
    std::vector<Point2D> corners;  // 四个角点

    OrientedRect() : yaw(0), length(0), width(0) {}
    OrientedRect(double cx, double cy, double _yaw, double _length, double _width);

    void computeCorners();  // 计算四个角点
};
```

**新增辅助函数:**

| 函数 | 用途 |
|------|------|
| `pointToSegmentDistance()` | 计算点到线段的最小距离 |
| `getMinDistanceBetweenRects()` | 计算两矩形间的最小距离 |
| `rectsOverlap()` | 使用 AABB 检测两矩形是否相交 |
| `quaternionToYaw()` | 从四元数提取 yaw 角 |

**修改的函数:**

| 函数 | 修改前 | 修改后 |
|------|--------|--------|
| `isObstacleNearBody(BoundingBox)` | 质心在车辆矩形内判断 | 矩形碰撞检测（AABB） |
| `getDistanceToObstacle(BoundingBox)` | 质心到原点距离 | 矩形间最小距离 |
| `getGlobalObstacleCoords(BoundingBox)` | 有 typo | 修复 |

**核心算法说明:**

1. **碰撞检测** (`isObstacleNearBody`):
   - 构建车辆矩形（考虑车辆朝向 `self.yaw`）
   - 构建障碍物矩形（考虑障碍物朝向 `bbox.pose.orientation`）
   - 使用 AABB 近似检测是否相交

2. **距离计算** (`getDistanceToObstacle`):
   - 如果两矩形相交，返回 `0.0`
   - 否则，计算车辆四个角点到障碍物四条边的最小距离

**行为变化:**

| 场景 | 修改前 | 修改后 |
|------|--------|--------|
| 障碍物质心在车内但角落未接触 | 触发 | 触发（正确） |
| 障碍物角落靠近但质心较远 | 不触发（漏报） | 触发（正确） |
| 障碍物靠近但未接触 | 触发（误报） | 不触发（正确） |

---

## 2026-04-27: 统一障碍物距离判断（全部使用最小距离）

### 需求
所有障碍物与车之间的距离判断，统一使用"车辆旋转矩形 ↔ 障碍物"的最小距离作为判断基准，消除判断逻辑分裂，提升安全性和准确性。

### 修改文件

#### `src/obstacle_avoider/src/obstacle_avoider_node.cpp`

**新增结构体:**
```cpp
// 安全判断阈值（统一参数管理）
struct SafetyThresholds {
    double body_proximity;       // 车身边界触发阈值
    double path_corridor;        // 路径走廊宽度
    double human_alert_distance; // 行人预警距离
};
```

**新增辅助函数:**

| 函数 | 用途 |
|------|------|
| `minDistanceBetweenRects()` | 两旋转矩形间的最小距离（角点到线段） |
| `minDistancePointToRect()` | 点到旋转矩形的最小距离（用于质心障碍物） |
| `rectsAABBOverlap()` | 轴对齐包围盒预检 |

**新增方法:**

| 方法 | 用途 |
|------|------|
| `isOnPath<T>()` | 障碍物全局坐标是否在路径走廊内 |
| `computeMinDistance<T>()` | 统一接口：计算障碍物到车辆的最小距离 |
| `isInFront<T>()` | 判断障碍物是否在车辆正前方 |

**核心变更 — `processObstacle<T>()` 统一逻辑:**

```
遍历障碍物
    ↓
跳过条件（臂举升 / 料场点云）
    ↓
计算最小距离 = minDistance(vehicle_rect, obstacle)
    ↓
Rule B: 最小距离 ≤ body_proximity (0=重叠) → 急停
    ↓
Rule A: 在路径走廊内
    ├── 行人 + 距离<20m → 鸣笛+急停
    └── 其他 → 急停
```

**计算方式统一:**

| 障碍物类型 | 判断方式 | 几何模型 |
|-----------|---------|---------|
| `PerceptInfo.obstacles` | 质心→车辆矩形最近点 | 点到旋转矩形 |
| `OriginObs` | 质心→车辆矩形最近点 | 点到旋转矩形 |
| `jsk_bboxes` | 矩形间最小距离 | 旋转矩形↔旋转矩形 |

#### `src/obstacle_avoider/config/params.yaml`

**新增参数:**
```yaml
# --- 安全判断阈值（基于最小距离）---
safety_thresholds:
  body_proximity: 0.0     # 障碍物进入车身矩形就算重叠（=0表示精确碰撞检测）
  path_corridor: 1.0      # 路径两侧走廊宽度（每侧1m）
  human_alert_distance: 20.0  # 行人预警距离
```

**参数变更:**

| 参数 | 修改前 | 修改后 |
|------|--------|--------|
| `vehicle_model.width` | 3.0 | 5.0 |
| `path_intrusion_rule.corridor_width` | 6.0 | 2.0 |
| `key_target_proximity_rule` | 独立参数组 | 移除（合并到 SafetyThresholds） |
| `corridor_pointcloud_rule` | 独立参数组 | 移除（合并到 processObstacle） |

**删除参数组:**
```yaml
# 已删除（合并到统一逻辑）:
key_target_proximity_rule:
  enabled: true
corridor_pointcloud_rule:
  enabled: true
```

### 优点

1. **一致性**：所有障碍物用同一套几何算法，消除三类障碍物判断逻辑分裂
2. **更安全**：最小距离 ≤ 质心距离，不会漏掉"突出部分侵入"的情况
3. **更准确**：旋转矩形保留障碍物朝向信息，AABB 不丢失角度
4. **可维护**：参数集中管理，阈值调整不需要改代码

### 行为对比

| 场景 | 修改前 | 修改后 |
|------|--------|--------|
| 障碍物角落靠近但质心较远 | 可能不触发（漏报） | 触发（正确） |
| 障碍物靠近但未接触 | 可能误报 | 精确判断 |
| 障碍物在车身边界 | 用 AABB 近似 | 用精确旋转矩形 |

---

## 2026-04-23: 添加任务状态上报功能

### 需求
车端每 1s 上报一次任务状态，每次更改任务状态立即上报一次。

### Topic
`/vehicle/task_status` (std_msgs/Int32MultiArray)

### 消息格式
```
data[0] = state       // 状态码 (0-8)
data[1] = timestamp    // Unix 时间戳
data[2] = task_id     // 任务编号（-1 表示无任务）
data[3] = reserved     // 预留
```

### 状态码对照表

| 状态 | 状态码 | 说明 |
|------|--------|------|
| NOT_STARTED | 0 | 未启动 |
| IDLE | 1 | 空闲中 |
| AUTO_WORKING | 2 | 自动作业中 |
| TEMP_WORKING | 3 | 临时作业中 |
| EMERGENCY_STOP | 4 | 紧急停车中 |
| OBSTACLE_STOP | 5 | 避障停车中 |
| REMOTE_CONTROL | 6 | 远程接管中 |
| ENDING | 7 | 结束作业中 |
| ENDED | 8 | 已结束 |

### 修改文件

#### 1. `src/autonomous_loader_bt/include/autonomous_loader_bt/loader_bt_nodes.h`

**新增 TaskStatusReporter 类:**
```cpp
class TaskStatusReporter
{
public:
    static TaskStatusReporter& instance();

    enum State { ... };

    void init(ros::NodeHandle& nh);
    void setState(int state, const std::string& task_id = "");
    void setTaskId(const std::string& task_id);
    int getState();
    std::string getTaskId();

private:
    // ... 成员变量和私有方法
};
```

**GlobalState 类新增方法:**
```cpp
void setTaskId(const std::string& task_id);
std::string getTaskId() const;

// 成员变量
std::string current_task_id_;
```

#### 2. `src/autonomous_loader_bt/src/loader_bt_nodes.cpp`

**修改 SetWorkState::tick():**
```cpp
BT::NodeStatus SetWorkState::tick()
{
    int value;
    if (!getInput("value", value)) { return BT::NodeStatus::FAILURE; }
    ros::param::set("/workstate", value);

    // 调用状态上报模块
    std::string task_id = autonomous_loader_bt::GlobalState::getInstance().getTaskId();
    autonomous_loader_bt::TaskStatusReporter::instance().setState(value, task_id);

    ROS_INFO("[BT] Set /workstate to %d, TaskID: %s", value, task_id.c_str());
    return BT::NodeStatus::SUCCESS;
}
```

**新增 GlobalState 方法:**
```cpp
void GlobalState::setTaskId(const std::string& task_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_task_id_ = task_id;
}

std::string GlobalState::getTaskId() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_task_id_;
}
```

#### 3. `src/autonomous_loader_bt/src/autonomous_loader_bt_node.cpp`

**在构造函数中初始化:**
```cpp
// 初始化任务状态上报器
autonomous_loader_bt::TaskStatusReporter::instance().init(nh_);
```

**在接收任务时更新 task_id:**
```cpp
// startTaskCallback 中
autonomous_loader_bt::GlobalState::getInstance().setTaskId(std::to_string(msg->task_id));
autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(std::to_string(msg->task_id));

// taskServiceCallback 中
autonomous_loader_bt::GlobalState::getInstance().setTaskId(std::to_string(req.taskID));
autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(std::to_string(req.taskID));
```

**在取消任务时清除 task_id:**
```cpp
gs.setTaskId("");
autonomous_loader_bt::TaskStatusReporter::instance().setTaskId("");
```

### 触发时机

| 时机 | 动作 |
|------|------|
| SetWorkState 执行 | 设置状态，触发立即上报 |
| 接收新任务 | 更新 task_id |
| 任务取消/完成 | 清除 task_id |
| 每秒定时器 | 定时上报当前状态 |
