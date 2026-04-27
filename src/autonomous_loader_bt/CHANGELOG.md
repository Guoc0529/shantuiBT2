# 代码修改记录

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
