# 导航模块与行为树状态机对接指南

## 1. 概述

本文档旨在为导航模块的开发人员提供清晰的对接规范。行为树（BT）状态机作为上层调度者，通过标准的ROS Action接口 (`/move_base`) 与导航模块进行交互。导航模块需要实现一个符合 `move_base_msgs/MoveBaseAction` 规范的Action Server，并根据状态机的指令执行路径规划和移动控制。

## 2. 核心交互接口：/move_base Action

状态机通过 `/move_base` Action Server向导航模块下发所有移动指令。

-   **Action 名称**: `/move_base`
-   **Action 类型**: `move_base_msgs/MoveBaseAction`

### 2.1 Action Goal (`move_base_msgs/MoveBaseGoal`)

状态机会将目标位姿封装在 `target_pose` 字段中发送给导航模块。

```
goal.target_pose (geometry_msgs/PoseStamped)
```

导航模块接收到目标后，应立即开始路径规划和移动。

### 2.2 Action Feedback (`move_base_msgs/MoveBaseFeedback`)

**这是对接中最关键的部分**。状态机依赖导航模块在移动过程中持续发布的 `feedback` 来触发后续动作（如放平铲斗、抬大臂）。

导航模块**必须**以一定的频率（推荐 **10Hz** 或更高）发布 `feedback`，并填充 `base_position` 字段。

```
feedback.base_position (geometry_msgs/PoseStamped)
```

状态机会利用这个 `feedback` 中的当前位置，与目标位置进行比较，实时计算出 `distance_to_goal`。这个距离值是触发后续动作的唯一依据。

### 2.3 Action Result (`move_base_msgs/MoveBaseResult`)

当导航完成时，导航模块需要返回最终状态。状态机主要关心以下两种结果：

-   **成功 (`SUCCEEDED`)**: 表示车辆已成功到达目标点。状态机会据此触发“到达后”的动作（如开始铲料、卸料）。
-   **失败 (`ABORTED`, `REJECTED`, etc.)**: 表示导航失败。行为树会根据具体的失败情况执行相应的恢复逻辑或中止任务。

### 2.4 抢占 (Preemption)

在某些情况下（如收到“结束任务”指令），状态机会取消当前的导航目标。导航模块的 Action Server 必须能正确处理抢占请求 (`is_preempt_requested()`)，并及时停止车辆。

## 3. 导航在任务流程中的具体配合

### 3.1 前往铲料点 (Scooping Phase)

1.  **接收目标**: 状态机下发铲料点（或料仓中心）作为导航目标。
2.  **持续反馈**: 导航模块在接近目标的过程中，持续发布 `feedback`。
3.  **关键触发点 (由状态机处理)**:
    *   当状态机计算出的 `distance_to_goal` **≤ 3.0米** (`lower_bucket_distance`) 时，状态机会设置参数 `ArmBucketState` 为 `3`，指令工作装置放平铲斗。
4.  **到达铲料点**: 导航模块到达目标点后，返回 `SUCCEEDED`。
5.  **特殊速度要求 (铲料阶段)**:
    *   当状态机设置参数 `ArmBucketState` 为 `7` 时，表示进入最终的“铲料”动作。
    *   此时，导航/控制模块应**配合将车速保持在 5km/h 左右**，以完成铲料动作。这个状态会一直持续到工作装置模块将 `ArmBucketState` 置为 `10` (完成) 或 `-7` (失败)。

### 3.2 前往卸料点 (Dumping Phase)

1.  **接收目标**: 状态机下发料斗位置作为导航目标。
2.  **持续反馈**: 导航模块在接近目标的过程中，持续发布 `feedback`。
3.  **关键触发点 (由状态机处理)**:
    *   当 `distance_to_goal` **≤ 7.0米** (`raise_arm_distance`) 时，状态机会设置 `ArmBucketState` 为 `4`，指令工作装置抬大臂。
    *   **守护条件**: 如果车辆前进至 `distance_to_goal` **≤ 4.0米** (`dump_guard_distance`) 时，抬大臂动作仍未完成，状态机会判定任务失败。导航模块无需特殊处理，只需继续导航即可。
4.  **到达卸料点**: 导航模块到达目标点后，返回 `SUCCEEDED`。状态机随后会设置 `ArmBucketState` 为 `6`，开始卸料。

### 3.3 返回停车点

1.  **接收目标**: 在任务结束时，状态机会下发停车点位置。
2.  **到达**: 导航模块到达后返回 `SUCCEEDED`。

## 4. 对接总结

| 导航模块职责 | 状态机如何使用 |
| :--- | :--- |
| 实现 `/move_base` Action Server | 通过 Action Client 发送导航目标 |
| **持续发布 `feedback` (含`base_position`)** | **实时计算 `distance_to_goal`，用于触发中间动作** |
| 导航成功后返回 `SUCCEEDED` | 触发“到达后”的逻辑（如铲料、卸料） |
| 导航失败后返回 `ABORTED` 等 | 触发任务失败或恢复逻辑 |
| 响应抢占请求 | 确保高优先级指令（如“结束任务”）能及时中止导航 |
| **监测 `ArmBucketState` 参数** | **当 `ArmBucketState` 为 `7` 时，将车速控制在 5km/h 左右** |

请确保导航模块严格遵守以上规范，特别是 `feedback` 的持续发布和铲料阶段的特殊速度要求，这是保证整个自主装载流程成功的关键。

