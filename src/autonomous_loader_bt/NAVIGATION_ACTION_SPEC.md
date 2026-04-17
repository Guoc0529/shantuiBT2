# 导航 Action 对接说明

> 本文档面向“导航模块”开发者，描述如何与 `autonomous_loader_bt` 行为树配合，完成装载车在“去料仓/铲点/料斗/停车位”间的整套流程。

## 1. 总览

- 行为树通过 Action（推荐：`/move_base`，类型 `move_base_msgs/MoveBaseAction`）向导航模块下发目标位姿。
- 为驱动状态机内部的放平铲斗、抬臂、卸料等动作，导航模块需额外发布 3 个状态话题，并支持暂停/恢复。
- 若不想复用 move_base，可实现自定义 Action（例如 `autonomous_loader_msgs/NavigateToPose.action`），但仍需遵守话题协议。

## 2. Action 接口

| 项目 | 要求 |
| --- | --- |
| 名称 | `/move_base`（推荐） |
| 类型 | `move_base_msgs/MoveBaseAction` |
| Goal 内容 | `geometry_msgs/PoseStamped`，由行为树的节点 `SendNavigationGoalBinCenter / ScoopPoint / Hopper / Parking` 发起 |
| Result | 成功时 `set_succeeded()`；失败时 `set_aborted()` 并给出原因 |
| Preempt | 行为树可能在途中发布新任务，需支持 preempt/cancel |

> 自定义 Action 的 Goal/Feedback/Result 可扩展，但最低限度需与下文“状态话题”保持一致。

## 3. 必要话题（导航 → 行为树）

| 话题 | 类型 | 属性 | 说明 |
| --- | --- | --- | --- |
| `/autonomous_loader_bt_node/navigation/planning_success` | `std_msgs/Bool` | 触发事件，latched | 规划成功后立即发布 `True`（若失败可发布 `False`） |
| `/autonomous_loader_bt_node/navigation/distance_to_goal` | `std_msgs/Float64` | 5~10 Hz | 剩余距离，驱动放平/抬臂等动作，需单调递减，并在抵达时到 0 |
| `/autonomous_loader_bt_node/navigation/arrived` | `std_msgs/Bool` | 事件，latched | 抵达目标时发布 `True`（一次即可），行为树会在下一段导航前重置 |

### 发布时序

1. 收到新的 goal → **200ms 内**发布 `planning_success=True`
2. 规划成功后 → 连续发布 `distance_to_goal`（≥5Hz），值随时间减小
3. 判定抵达（距离 ≤0.1m 或 action 完成） → 发布 `arrived=True`，并 `set_succeeded`

> 建议在发布 arrived 的同一回调里再发送一次 `distance_to_goal=0.0`，避免竞态。

## 4. 控制话题（行为树 → 导航）

| 话题 | 类型 | 行为 |
| --- | --- | --- |
| `/autonomous_loader_bt_node/navigation/pause` | `std_msgs/Bool` | `True`：暂停/慢行；`False`：恢复。保持当前目标 |
| `/autonomous_loader_bt_node/navigation/cancel` *(可选)* | `std_msgs/Bool` | `True`：取消当前 goal，与 Action preempt 等效 |

行为树在每段导航前还会顺序调用：
- `ResetNavigationArrived`：通知“新一次导航开始”
- `ResetDistanceToGoal`：期望导航模块立即发布第一帧距离

## 5. 状态机依赖点

| 阶段 | 触发条件 | 导航信号 |
| --- | --- | --- |
| 放平铲斗 | 机器人距离铲料点 ≤ `lower_bucket_distance`（默认3.0m） | `distance_to_goal` 触发 |
| 抬臂 | 距离料斗 ≤ `raise_arm_distance`（默认7.0m） | `distance_to_goal` 触发 |
| Dump Guard | 若未抬臂且距离 ≤ `dump_guard_distance`（默认4.0m） | `distance_to_goal` 触发保护 |
| 导航到达 | 完成“料仓中心/铲点/料斗/停车位”等任一导航段 | `arrived=True` |

> 若 `/spadingPose` 返回 (0,0)，行为树会再次请求铲料点并重新发送 goal，导航模块需能连续处理多个目标。

## 6. 推荐实现流程（伪代码）

```python
def execute_move_base_goal(goal):
    publish(planning_success=True, latch=True)
    start_distance_timer(goal)

    while not arrived(goal):
        if pause_flag:
            hold_motion()
            continue
        publish(distance_to_goal=current_distance(goal))
        if preempt_requested:
            set_preempted()
            return

    publish(distance_to_goal=0.0)
    publish(arrived=True, latch=True)
    set_succeeded()
```

## 7. 联调步骤

1. 启动行为树：`roslaunch autonomous_loader_bt autonomous_loader_bt.launch`
2. 启动导航模块（实现上述接口）
3. 启动调度模拟器：`rosrun autonomous_loader_bt interactive_tester.py`，按 `s` 发送任务
4. 观察行为树日志，确认以下事件按顺序输出：
   - “Navigating to bin center / scoop point / hopper / parking”
   - distance 触发放平/抬臂
   - “Wait for navigation arrival” 快速通过

## 8. 常见问题

| 现象 | 可能原因 | 解决方式 |
| --- | --- | --- |
| 停在 “Wait for navigation arrival” | 没发布 `/navigation/arrived` | 到达时发布 Bool(True) 并 `set_succeeded` |
| 放平/抬臂没有动作 | 没有 `/navigation/distance_to_goal` 或频率过低 | 保证 ≥5Hz，且数值覆盖阈值 |
| 规划成功但 BT 报“等待规划成功” | `planning_success` 未在 200ms 内 latched True | 新 goal 触发时立即发布 |
| 暂停无效 | 未订阅 `/navigation/pause` | 实现暂停逻辑，保持 goal |

## 9. 参考

- 若想快速模拟，可复用仓库里的 `interactive_tester.py` 中 `MoveBaseMockServer` 逻辑，并补齐上述话题，即可当作导航 mock。
- 需要更多例子，可联系行为树维护人员获取。

