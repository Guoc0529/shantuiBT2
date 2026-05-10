# 人工接管功能设计方案

## 一、需求概述

在自动装载作业流程中，当某一步骤执行失败（导航失败、臂铲超时等）时，状态机请求人工接管。人工操作员完成接管后将控制权交还给状态机，状态机继续执行后续流程。

---

## 二、现状分析

### 2.1 当前人工介入机制（已有但不完整）

**已有节点：**
- `RequestManualIntervention` — 发布急停、设置 `manual_intervention_required=true`
- `WaitForManualIntervention` — 等待 `/manual_intervention_complete=true`

**问题：**
1. **触发条件单一** — 只在导航 `SafeNavigate` 失败时触发，无法覆盖臂铲、点云累积等场景
2. **无上下文信息** — 操作员不知道失败原因和位置
3. **无超时机制** — 操作员接管时间不受限制，可能导致流程永久挂起
4. **无状态持久化** — 接管状态未记录到 GlobalState，无法区分"正在接管"和"未接管"
5. **无恢复策略** — 接管完成后直接重试，失败后再次接管，无最大次数限制
6. **无任务上下文** — 接管时任务 ID、当前阶段等关键信息未上报

### 2.2 现有 FAILURE 来源汇总

| 失败来源 | 触发条件 | 发生阶段 | 当前处理 |
|---------|---------|---------|---------|
| **导航规划失败** | `planning_feedback.success=false` 或超时 300s | 铲料点导航 / 卸料导航 | 人工介入（无限重试） |
| **导航不可恢复错误** | error_code 非 102/103/104 | 导航过程中 | 人工介入 |
| **到达质量不达标** | 距离误差或角度误差超阈值（仅卸料导航） | 卸料导航 | 人工介入 |
| **臂铲动作超时** | `WaitForArmBucketCompletion` 超时 300s | 铲料动作 | RetryUntilSuccessful 无限重试 |
| **卸料动作超时** | `WaitForArmBucketCompletion` 超时 300s | 卸料 | RetryUntilSuccessful 无限重试 |
| **点云累积超时** | `WaitForPointAccumulationDone` 超时 300s | 铲料点请求前 | RetryUntilSuccessful 无限重试 |
| **任务队列空** | `GetAndSetCurrentTask` 无任务 | 任务循环开始 | 回到 idle |
| **取消任务** | taskID=0 | 任意时刻（最高优先级） | 取消导航+臂铲，清理状态 |

---

## 三、设计方案

### 3.1 总体架构

```
┌─────────────────────────────────────────────────────────────┐
│                    状态机 (BehaviorTree)                     │
│  ReactiveFallback (优先级调度)                                │
│    ├── SEQ_CANCEL_TASK      取消任务（最高优先）             │
│    ├── SEQ_MANUAL_OVERRIDE  人工接管（新增，第二优先）       │
│    ├── SEQ_STOP             正常结束                         │
│    ├── SEQ_PAUSE            暂停                             │
│    └── SEL_WORK_IDLE        工作/空闲                        │
│                              └── SEQ_TASK_LOOP               │
│                                    └── SEQ_ONE_CYCLE        │
│                                          ├── 导航到铲料点    │
│                                          ├── 铲料动作       │
│                                          ├── 导航到仓        │
│                                          └── 卸料动作       │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌────────────────────────┐
              │  GlobalState 新增字段   │
              │  manual_override_state_ │
              └────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              上位机 / 远程操控界面                           │
│   接收接管请求 → 显示失败信息+操作指引 → 操作完成后通知状态机  │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 人工接管适用场景

| 场景 | 失败原因 | 建议处理方式 | 适合人工接管？ |
|------|---------|-------------|--------------|
| 导航规划失败 | 路径规划超时/无解 | 操作员手动驾驶到目标附近 | ✅ 适合 |
| 导航过程障碍物 | 检测到障碍物无法绕行 | 操作员观察后手动绕障 | ✅ 适合 |
| 到达质量不达标 | 停车位置偏差过大 | 操作员微调车辆位置 | ✅ 适合 |
| 臂铲动作超时 | 液压系统响应慢/传感器异常 | 操作员手动操作臂铲 | ⚠️ 需评估 |
| 卸料动作超时 | 同上 | 同上 | ⚠️ 需评估 |
| 点云累积超时 | 感知系统异常 | 操作员确认手动标记铲料点 | ⚠️ 需评估 |
| 设备急停触发 | 急停按钮被按下 | 必须人工现场处理 | ✅ 适合（强制） |
| 传感器异常 | 定位丢失/IMU 漂移 | 操作员确认后手动重新定位 | ✅ 适合 |

**不适合人工接管的场景**（应直接重试或放弃任务）：
- 通讯中断（网络超时）- 应重试网络
- 配置文件错误 - 应报错退出

### 3.3 新增话题设计

#### 3.3.1 状态机 → 上位机（发布）

**Topic:** `/bt_manual_override/request`
**Type:** `std_msgs/String`（JSON 格式）

```json
{
  "request_id": 12345,
  "timestamp": 1713849600.123,
  "task_id": 5,
  "phase": "NAVIGATE_TO_SCOOP",
  "failed_node": "SafeNavigate",
  "failure_reason": "PLANNING_TIMEOUT",
  "error_detail": "规划反馈超时 300s",
  "vehicle_pose": {"x": 10.5, "y": 20.3, "yaw": 1.57},
  "target_pose": {"x": 15.0, "y": 25.0, "yaw": 0.0},
  "retry_count": 2,
  "max_retries": 3,
  "instruction": "请手动驾驶车辆前往目标点(15.0, 25.0)，完成后点击'完成接管'"
}
```

#### 3.3.2 上位机 → 状态机（订阅）

**Topic:** `/bt_manual_override/complete`
**Type:** `std_msgs/String`（JSON 格式）

```json
{
  "request_id": 12345,
  "timestamp": 1713849700.456,
  "success": true,
  "operator_action": "MANUAL_DRIVE",
  "final_pose": {"x": 15.1, "y": 25.1, "yaw": 0.05},
  "note": "已手动驾驶到目标点附近，偏差 < 0.2m"
}
```

#### 3.3.3 状态机 → 上位机（发布，心跳）

**Topic:** `/bt_manual_override/heartbeat`
**Type:** `std_msgs/String`（JSON 格式）

```json
{
  "timestamp": 1713849610.123,
  "request_id": 12345,
  "state": "WAITING",
  "elapsed_seconds": 10.5,
  "vehicle_pose": {"x": 10.8, "y": 20.5, "yaw": 1.58}
}
```

### 3.4 GlobalState 新增字段

```cpp
// 人工接管状态机
enum class ManualOverrideState {
    IDLE,           // 无接管请求
    REQUESTED,      // 已发出接管请求，等待操作员确认
    IN_PROGRESS,    // 操作员正在接管
    COMPLETED,      // 操作员完成接管
    REJECTED,       // 操作员拒绝接管（超时或其他）
    MAX_RETRIES     // 达到最大接管次数
};

struct ManualOverrideInfo {
    int request_id;                     // 本次请求的唯一 ID
    std::string phase;                  // 失败时的作业阶段
    std::string failed_node;           // 失败的节点名
    std::string failure_reason;         // 失败原因
    int retry_count;                    // 当前重试次数
    int max_retries;                   // 最大重试次数
    std::chrono::steady_clock::time_point request_time;  // 请求时间
};

// 成员变量
ManualOverrideState manual_override_state_;
ManualOverrideInfo manual_override_info_;
int manual_override_request_id_counter_;
```

### 3.5 新增行为树节点

#### 3.5.1 RequestManualOverride（动作节点）

**功能：** 发布接管请求，等待上位机确认开始接管

```cpp
BT::NodeStatus RequestManualOverride::tick()
{
    switch (state_) {
        case IDLE:
            // 生成请求 ID
            manual_override_info_.request_id = ++request_id_counter_;
            manual_override_info_.phase = getInputOrThrow<std::string>("phase");
            manual_override_info_.failed_node = getInputOrThrow<std::string>("failed_node");
            manual_override_info_.failure_reason = getInputOrThrow<std::string>("failure_reason");
            manual_override_info_.retry_count = getInputOrThrow<int>("retry_count");
            manual_override_info_.max_retries = getInputOrThrow<int>("max_retries");
            manual_override_info_.request_time = std::chrono::steady_clock::now();

            // 发布接管请求
            publishOverrideRequest(manual_override_info_);
            // 发布急停
            publishEstop(1);

            GlobalState::getInstance().setManualOverrideState(REQUESTED);
            state_ = WAITING_CONFIRM;

        case WAITING_CONFIRM:
            if (/* 收到确认 */) {
                state_ = IN_PROGRESS;
                GlobalState::getInstance().setManualOverrideState(IN_PROGRESS);
            }
            return BT::NodeStatus::RUNNING;
    }
}
```

#### 3.5.2 WaitForManualOverride（动作节点）

**功能：** 等待接管完成，支持超时和最大次数限制

```cpp
BT::NodeStatus WaitForManualOverride::tick()
{
    // 检查是否达到最大接管次数
    if (manual_override_info_.retry_count >= manual_override_info_.max_retries) {
        ROS_ERROR("[ManualOverride] 达到最大接管次数 (%d)", manual_override_info_.max_retries);
        GlobalState::getInstance().setManualOverrideState(MAX_RETRIES);
        state_ = IDLE;
        return BT::NodeStatus::FAILURE;
    }

    // 发布心跳
    publishHeartbeat();

    // 检查超时
    auto elapsed = std::chrono::steady_clock::now() - manual_override_info_.request_time;
    if (elapsed > max_override_timeout_) {
        ROS_ERROR("[ManualOverride] 接管超时 (%.1fs)", max_override_timeout_.count());
        GlobalState::getInstance().setManualOverrideState(REJECTED);
        state_ = IDLE;
        return BT::NodeStatus::FAILURE;
    }

    // 检查是否收到完成消息
    if (received_complete_) {
        if (latest_complete_.success) {
            ROS_INFO("[ManualOverride] 接管完成，操作员: %s", latest_complete_.operator_action.c_str());
            GlobalState::getInstance().setManualOverrideState(COMPLETED);
            // 解除急停
            publishEstop(0);
            state_ = IDLE;
            return BT::NodeStatus::SUCCESS;
        } else {
            ROS_WARN("[ManualOverride] 操作员拒绝接管");
            GlobalState::getInstance().setManualOverrideState(REJECTED);
            state_ = IDLE;
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}
```

#### 3.5.3 CheckManualOverrideStatus（条件节点）

**功能：** 检查当前接管状态

```cpp
BT::NodeStatus CheckManualOverrideStatus::tick()
{
    auto state = GlobalState::getInstance().getManualOverrideState();
    std::string expected = getInputOrThrow<std::string>("expected_state");

    if (expected == "IN_PROGRESS" && state == ManualOverrideState::IN_PROGRESS) {
        return BT::NodeStatus::SUCCESS;
    }
    if (expected == "COMPLETED" && state == ManualOverrideState::COMPLETED) {
        return BT::NodeStatus::SUCCESS;
    }
    if (expected == "MAX_RETRIES" && state == ManualOverrideState::MAX_RETRIES) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
```

### 3.6 行为树集成

#### 3.6.1 新增优先级分支 SEQ_MANUAL_OVERRIDE

在 ReactiveFallback 中，SEQ_MANUAL_OVERRIDE 优先级仅次于取消任务：

```
ReactiveFallback (ROOT)
├── SEQ_CANCEL_TASK        (优先级 1: 取消任务)
├── SEQ_MANUAL_OVERRIDE    (优先级 2: 人工接管)  ← 新增
├── SEQ_STOP               (优先级 3: 正常结束)
├── SEQ_PAUSE              (优先级 4: 暂停)
└── SEL_WORK_IDLE          (优先级 5: 工作/空闲)
```

**SEQ_MANUAL_OVERRIDE 结构：**
```
SEQ_MANUAL_OVERRIDE
├── CheckManualOverrideStatus expected_state="IN_PROGRESS"
├── KeepRunningUntilFailure   ← 等待接管完成或失败
│     SEQ_WAIT_OVERRIDE
│     ├── WaitForManualOverride timeout=600s
│     └── CheckManualOverrideStatus expected_state="COMPLETED"
└── Fallback
      ├── CheckManualOverrideStatus expected_state="COMPLETED"
      │     → 清除接管状态，继续流程
      └── CheckManualOverrideStatus expected_state="MAX_RETRIES"
            → 发布错误日志，放弃任务，进入 end 流程
```

#### 3.6.2 改造 SafeNavigate 子树中的接管逻辑

**修改前：**
```
SendNavigationGoal
Fallback:
  ├─ Sequence: WaitForPlanningFeedback + WaitForNavigationResult
  └─ Sequence: RequestManualIntervention + WaitForManualIntervention  ← 简单版
```

**修改后：**
```
SendNavigationGoal
Fallback:
  ├─ Sequence: WaitForPlanningFeedback + WaitForNavigationResult → 成功
  └─ SEQ_NAV_FAILURE_RECOVERY  ← 独立的失败恢复序列
        ├─ CheckErrorCode (可恢复错误码 102/103/104)
        │     → AlwaysFailure → 外层 RetryUntilSuccessful 重试
        └─ SEQ_NAV_MANUAL_OVERRIDE  ← 不可恢复错误 → 人工接管
              ├─ RetryUntilSuccessful num_attempts=3
              │     SEQ_SINGLE_OVERRIDE
              │     ├─ RequestManualOverride
              │     │     phase="NAVIGATE"
              │     │     failed_node="SafeNavigate"
              │     │     failure_reason={error_detail}
              │     │     retry_count={循环计数}
              │     │     max_retries=3
              │     ├─ WaitForManualOverride timeout=600s
              │     └─ CheckManualOverrideStatus expected_state="COMPLETED"
              └─ AlwaysFailure  ← 3次接管都失败 → 放弃导航，发布错误
```

#### 3.6.3 臂铲/卸料/点云累积失败 → 人工接管

在对应动作的 `WaitForArmBucketCompletion` 等节点外层包裹：

```
SEQ_ARM_FAILURE_RECOVERY
├─ RetryUntilSuccessful num_attempts=1  ← 第一次失败
│     [原臂铲/卸料逻辑]
├─ SEQ_ARM_MANUAL_OVERRIDE  ← 第一次失败后进入接管
│     ├─ RequestManualOverride
│     │     phase="ARM_ACTION"   ← 臂铲或卸料
│     │     failed_node="WaitForArmBucketCompletion"
│     │     failure_reason="TIMEOUT" 或 {ArmBucketState}
│     │     max_retries=2
│     ├─ WaitForManualOverride timeout=600s
│     └─ CheckManualOverrideStatus expected_state="COMPLETED"
└─ AlwaysFailure  ← 接管失败 → 放弃该臂动作，继续流程（或其他策略）
```

### 3.7 上位机协议

#### 3.7.1 接管请求处理流程（上位机侧）

```
1. 订阅 /bt_manual_override/request
2. 收到请求后：
   a. 解析 JSON，提取失败原因、目标位置、操作指引
   b. 在界面上显示：
      - 当前作业阶段 (phase)
      - 失败原因 (failure_reason)
      - 车辆当前位置
      - 目标位置
      - 操作指引 (instruction)
   c. 显示倒计时（从 timeout 开始递减）
3. 操作员执行完手动操作
4. 点击"完成接管"按钮
5. 发布 /bt_manual_override/complete 到状态机
```

#### 3.7.2 上位机建议界面元素

| 区域 | 内容 |
|------|------|
| 告警横幅 | "人工接管请求 - {phase}" |
| 失败详情 | 节点名、失败原因、错误详情 |
| 地图标注 | 车辆当前位置 + 目标位置 |
| 操作指引 | 根据 failure_reason 动态显示操作步骤 |
| 倒计时 | 剩余可接管时间 |
| 按钮 | [完成接管] [拒绝接管] [取消任务] |

### 3.8 任务状态上报集成

在人工接管各阶段，上报状态给上位机（通过已有的 `/vehicle/task_status` 或新增话题）：

```cpp
// 请求接管时
publishTaskStatus(state=OBSTACLE_STOP, message="人工接管中 - " + failure_reason);

// 接管完成时
publishTaskStatus(state=AUTO_WORKING, message="接管完成，继续作业");
```

---

## 四、接管策略设计

### 4.1 按失败类型分层处理

```
导航失败
├── 规划无解 / 路径规划超时
│     → 操作员手动驾驶到目标附近 → 完成后从当前位置重新发起规划
├── 障碍物阻挡无法绕行
│     → 操作员观察后手动绕障 → 绕过后从新位置继续导航
└── 到达位置偏差过大
      → 操作员微调车辆 → 完成后继续卸料/铲料

臂铲/卸料动作失败
├── 液压响应超时
│     → 操作员手动完成臂铲动作 → 完成后继续流程
├── 传感器信号异常
│     → 操作员确认状态后继续 → 跳过等待传感器反馈
└── 点云累积超时
      → 操作员手动标记铲料点 → 跳过累积直接导航

急停触发
└── 必须现场处理 → 操作员确认安全后解除急停 → 继续或放弃任务
```

### 4.2 接管后的恢复策略

| 接管完成后的状态 | 后续行为 |
|----------------|---------|
| 车辆已到达目标点 | 继续下一动作（铲料/卸料） |
| 车辆已到达目标附近（偏差小） | 微调导航后继续 |
| 车辆未到达目标（操作员中断） | 评估后重试或放弃任务 |
| 操作员拒绝接管 | 放弃当前任务，进入 park 流程 |
| 达到最大接管次数 | 放弃任务，发布告警 |

---

## 五、参数配置

```yaml
# 人工接管配置
manual_override:
  enabled: true
  max_retries: 3              # 最大接管次数（导航）
  max_retries_arm: 2          # 最大接管次数（臂铲）
  timeout: 600                 # 接管超时时间（秒）
  require_estop_on_request: true   # 请求接管时是否发急停

# 各阶段最大接管次数
override_max_retries_by_phase:
  NAVIGATE: 3
  ARM_ACTION: 2
  POINT_ACCUMULATION: 1
  EMERGENCY_STOP: 0           # 急停不允许自动恢复接管
```

---

## 六、修改文件清单

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| `loader_bt_nodes.h` | 新增类 | `RequestManualOverride`、`WaitForManualOverride`、`CheckManualOverrideStatus` |
| `loader_bt_nodes.cpp` | 新增实现 | 上述三个节点类的 tick() 实现 |
| `loader_bt_nodes.h` | 修改类 | `GlobalState` 新增 `manual_override_state_` 等字段和方法 |
| `ros_topic_manager.h` | 新增方法 | `publishManualOverrideRequest`、`publishManualOverrideComplete`、`publishManualOverrideHeartbeat` |
| `autonomous_loader_bt_node.cpp` | 新增订阅 | 订阅 `/bt_manual_override/complete` |
| `autonomous_loader_bt_node.cpp` | 注册节点 | 将新节点注册到 BehaviorTreeFactory |
| `params.yaml` | 新增参数 | 接管超时、最大次数等配置 |
| `autonomous_loader_tree.xml` | 新增分支 | SEQ_MANUAL_OVERRIDE 分支 + 改造 SafeNavigate 子树 |
| `CHANGELOG.md` | 新增章节 | 记录本次改动 |

---

## 七、实现优先级

| 优先级 | 内容 | 工作量 | 说明 |
|--------|------|--------|------|
| P0 | GlobalState + 话题发布/订阅基础设施 | 中 | 先把基础设施搭好 |
| P0 | `RequestManualOverride` + `WaitForManualOverride` 节点 | 中 | 核心接管逻辑 |
| P0 | SEQ_MANUAL_OVERRIDE 分支集成 | 小 | 挂载到根节点 |
| P1 | 改造 SafeNavigate 中的接管逻辑 | 中 | 导航失败的完整接管流程 |
| P1 | 臂铲/卸料动作失败的接管 | 中 | 外层包裹接管节点 |
| P2 | 上位机协议和界面集成 | 大 | 需要对方配合 |
| P3 | 点云累积失败的接管 | 小 | 复用通用接管节点 |

---

## 八、风险与注意事项

1. **死锁风险** — 如果上位机不响应，状态机会在 `WaitForManualOverride` 永久等待。需要设置合理的超时时间，超时后走失败分支。
2. **并发接管** — 确保只有一个接管请求活跃。使用 `request_id` 区分多次请求。
3. **任务上下文丢失** — 接管期间如果收到取消任务，需要中断接管流程。`SEQ_CANCEL_TASK` 优先级高于接管，所以天然支持。
4. **状态一致性** — 接管开始时发急停，接管结束时必须解除急停。使用 RAII 或 try-finally 确保。
5. **最大次数保护** — 防止无限接管循环。达到最大次数后应放弃任务或上报告警。
