结构：
本功能包->Hybrid-Astar-ESDF的ROS wrapper（即run_hybrid_a_star）
  - Planner now uses a RemotePlannerBackend to call PlanHybridPath/SetPlannerParams services with timeout handling;

• 按 src/ros/planner_ros.cpp 梳理的状态机主链为 Idle → PlanningGlobalPath →
  AdjustingDirection → Moving → CheckingGoal → Completed（任意环节出错经
  setError 进入 Error）。

  | 状态 | 进入途径 | 状态内主要动作 | 转出条件 |
  | --- | --- | --- | --- |
  | Idle | 初始状态；任务解码失败时 kickoffTaskFromAction 退回 | 定时器仅打印等待 
  | 新的有效 onActionGoal 触发 PlanningGlobalPath |

  | PlanningGlobalPath | 由 kickoffTaskFromAction 进入 | 定时器调用 planning_global_path：取起点/终点
  特殊情况：卸料时，终点向外移3m，卸料到铲料时，起点向外移1.5m
  随后调用远端规划，分段并生成 nav_path_、发布全局路径，必要时把原始卸料终点补回 (src/ros/planner_ros.cpp:114-118,320-425) 
  | 规划成功 → AdjustingDirection (src/ros/planner_ros.cpp:425); 规划失败时，尝试重规划，如果重规划失败 → setError → Error (src/ros/planner_ros.cpp:338-401) |

  | AdjustingDirection | 规划成功后，或 Moving 完成一段后 (src/ros/
  planner_ros.cpp:425,795-799) | 定时器发布当前段路径，调用adjust_vehicle_direction；发布段终点给 goal_checker，按分段方向发送 is_forward
  切换命令并等待节点反馈/超时 (src/ros/planner_ros.cpp:120-128,572-630) | 所有必
  需节点反馈成功 → finalizeDirectionSwitch → Moving (src/ros/planner_ros.cpp:608-684,1051-1088); 切换超时/被拒/路径或标志缺失 → Error (src/
  ros/planner_ros.cpp:576-606,671-678) |

  | Moving | 前后向切换确认后进入 (src/ros/planner_ros.cpp:1051-1088) | 高频onMovingTimer：若无距离反馈报错；按当前姿态截取路径生成局部路径，发布局部路径
  与 action 反馈 (src/ros/planner_ros.cpp:182-215,207-215) | onReach 真：若已是最后一段则停动计时器并进入 CheckingGoal/直接 Completed；否则切换姿态源并回到
  AdjustingDirection (src/ros/planner_ros.cpp:724-800). onFinalGoalReached 在最后一段可先行把状态切到 CheckingGoal 或 Completed (src/ros/planner_ros.cpp:802-
  878). 缺距离等异常 → Error |

  | CheckingGoal | 最后一段走完或 final_goal 先到时进入 (src/ros/planner_ros.cpp:769-877) | 首次进入记录开始时间，等待 reach_goal_received_ 与
  final_goal_received_ 均成功；超时则记错 (src/ros/planner_ros.cpp:135-166) | 两个确认齐全且成功 → Completed (src/ros/planner_ros.cpp:143-147,866-870); 超时 →
  Error (src/ros/planner_ros.cpp:150-158); final_goal 到达但 reach 未到则保持等待 (src/ros/planner_ros.cpp:873-876) |

  | Completed | goal/final_goal 都 OK 后进入 (src/ros/planner_ros.cpp:143-147,786-787,866-870) | onEnterCompleted 发布成功结果并 reset 缓存/定时器 (src/
  ros/planner_ros.cpp:283-309); 定时器打印等待 (src/ros/planner_ros.cpp:168-171)| 新的 ActionGoal 可直接再次走 PlanningGlobalPath (src/ros/
  planner_ros.cpp:688-716,912-931) |
  
  | Error | 任意环节 setError 跳转 (src/ros/planner_ros.cpp:287-300) | 进入时发布失败结果（含错误码/消息），定时器仅打印错误 (src/ros/planner_ros.cpp:294-
  300,172-175) | 无自动恢复；外部下发新 ActionGoal 时 kickoffTaskFromAction 会reset 并重新转到 PlanningGlobalPath 或 Idle (src/ros/planner_ros.cpp:912-931)|

  补充事件流：onFinalGoalReached 还在卸料时下发 30% 刹车 (src/ros/planner_ros.cpp:849-860); reset 会清空路径并停止 moving 定时器，但不改state_，需新 goal 驱动下一次运行 (src/ros/planner_ros.cpp:933-965).



参数：
  - 私有参数 `~planner_config_path`（默认 `<pkg>/config/params.json`）用于装载全局规划参数与车辆形状，解析逻辑与 config/test_global_planner.cpp 一致；解析失败时退回默认参数并仅给出警告/错误日志。

Action相关改动：(已测试，未review)
切换为“手动”模式的 SimpleActionServer：构造时不传 execute 回调，改为
    registerGoalCallback 和 registerPreemptCallback，再 start()。这样 actionlib
      - 如果当前有活跃目标，先对旧目标 setPreempted（或 setAborted），并将
        has_active_goal_ 置 false。
      - 通过 acceptNewGoal() 取出 NavigateGoalConstPtr，立刻 setAccepted()（可
        选，表示已接受），然后把 goal 传给 planner_.onActionGoal(*goal)，并把
        has_active_goal_ 置 true。
      - 按需缓存当前 goal（若后续要用到）。
  - Preempt 回调（新增函数，如 onPreempt）逻辑：
      - 如果 has_active_goal_ 为真，则调用 setPreempted，has_active_goal_ =
        false，并通知状态机停止/重置（如果有对应接口）。
  - 状态机给终态：维持现在的做法，在 PlannerState::Completed 和
    PlannerState::Error 里调用 ros_io_->publishActionResult(...)。确保
    publishActionResult 能看到 has_active_goal_ 并清零标志。
  - 取消未触发终态的路径：当状态机 reset/abort 时（例如外部故障），也要走
    publishActionResult(false, false)，保证活跃 goal 总能收敛到终态。

  这样，整个流程变成“接收 goal → 交给状态机 → 状态机结束时给终态”，中间没有
  execute 回调返回时的强制终态检查，因此不会再触发 warning。

待review，参数设置方面:
  - JSON param loading is intentionally omitted per request;


使用TOPIC进入的流程：（待review）
• - 入口：调用 onGoal（planner_ros_legacy.cpp），打印 LEGACY 警告，reset() 清缓
    存/停 moving timer、姿态源设为后车架，然后把传入的 goal 存到
    current_goal_pose_，直接 transitionTo(PlanningGlobalPath, "goal received")。
  - 规划阶段：planning_global_path() 远程规划 → 切段 → nav_path_/方向标志就绪 →
    发布全局路径 → 进入 AdjustingDirection。
  - 换向阶段：adjust_vehicle_direction() 先发布当前段终点给 goal_checker；根据当
    前段的方向标志设置 desired_forward_ 并调用
    publishIsForward(desired_forward_)。随后等待 /is_forward_state 的反馈
    （onIsForwardState），所有必需节点确认后 finalizeDirectionSwitch()，延时 2
    秒后启动 moving timer，transitionTo(Moving, "direction aligned by
    feedback")。
  - 运行/段切换：onMovingTimer 发局部路径；onReach 收到下一段到达后切段，第二段
    起切到前车架姿态，进入下一次 AdjustingDirection 直至段耗尽 → Completed。
  - 与 is_forward 的联系：LEGACY入口和 action 入口在换向逻辑上相同，都会广播 /
    is_forward 命令并等待 /is_forward_state 反馈；onIsForward（订阅 /
    is_forward）仅更新内部布尔，不驱动状态机，主流程只认 /is_forward_state 的反
    馈来解锁换向并进 Moving。


TODO:在规划前，更好的确定使用前车架还是后车架
1. 可以用Astar跑一遍，根据Astar的路径方向决定起始用什么车架进行规划; 有情况不合理，比如车横着的时候，目标点让他竖着，这时候判断不出；
2. 可以用多级启发式判断，比如先用任务判断，如果上一状态是load或者unload，那车前方是没有空间的，必然要先后退（即使用后车架规划）；随后，使用±90°锥判断目标点是否正好落在前轴或者后轴前方，且目标点与前/后车架坐标系航向偏差不大，就判断使用前/后车架（但后车架坐标要转180°再进行判断）；然后，可以用地图语义信息进行判断，比如车在料仓内部且车头朝里，那么第一段肯定是后退；最后，没有任何信息（车在过道上，且人工接管过，从其他状态去load或者unload），那就默认先用前车架规划一遍，出来结果后，如果坐标系跟方向对不上，就换个坐标系再规划一遍。


# 注意：现在final_goal_reached被一个新增的标志位屏蔽了
# 注意：现在goal_reached话题的所有false信息都被忽略了
现在又改了，待review
理论上说现在两个goal topic都会进checkgoal，然后在这里timeout完去completed

# 注意：现在给规划喂的终点会往外面挪个3.0，然后再把原来action的点添加到规划结果
# 注意：现在临时增加了在抵达卸料点后，直到下个任务开始前会踩刹车

# 联调DEBUG：在联调center_articulation_planner_legacy，与du_ws下的center_articulation_controller与goal_checker功能包。目前，center_articulation_planner_legacy的planner_ros.cpp中会先向外发布is_forward给center_articulation_controller和goal_checker，随后再发布move_base_simple/goal给goal_checker，最后到moving中发布局部路径。理论上说，du_ws下的center_articulation_controller与goal_checker会确保在收到局部路径后，再移动车辆（发布cmd_vel）,但是实测中，输入一个新action给center_articulation_planner_legacy后，在is_forward发布后，局部路径发布前，cmd_vel中会有一个短促的非0速度，且该非0线速度呈两段阶梯式，并且很快就会归零（直到局部路径发出）。


待review：
• Added a private set_planner_params service server that forwards requests to
  the existing remote planner backend and validates non-empty updates.

  - include/center_articulation_planner/ros/ros_io.hpp, src/ros/ros_io.cpp:  include the service type, add setupServices() and setPlannerParamsCallback,
    advertise ~set_planner_params, and route callbacks into the planner.
  - include/center_articulation_planner/ros/planner_ros.hpp, src/ros/planner_ros.cpp: expose handleSetPlannerParams and implement basic
    validation (require at least one set_* flag) before forwarding via sendPlannerParams, returning success/error in the service response.


待review： 重新规划逻辑
1. 尝试退到铰接中心，并使用不同角度规划。如果当前使用前车架规划，沿前车架往后退1.5m，如果是后车架，沿后车架往前1.5m。随后尝试一左一右两个20°的偏航，重新进行规划，选总路径长度较短的路径进入下一状态。
2. 尝试起点、终点互换规划。
3. 尝试使用service设置最大换向次数为2，重新规划。
如果上述操作全都失败，setError报错。

 - 抽象一次规划尝试
      - 增加一个内部帮助函数 PlanAttempt runPlanAttempt(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped
        &goal, const std::string &tag)，封装当前的远程规划调用 + 错误处理 +buildSegmentsFromPlan。返回是否成功、路径总长度（可用已有
        computePathLength 或新 helper 计算 nav_msgs::Path 长度）、生成的global_path_ / nav_path_ / 方向标志等结果。
      - 失败时只返回错误信息，不 setError，由上层决定是否进入容错。
  - 生成“退到铰接中心”的起点
      - 新增 helper bool makeOffsetStart(double back_dist,geometry_msgs::PoseStamped &out_start)，使用当前的活动姿态源
        active_pose_source_：
          - 如果当前用前车架规划，则在当前 yaw 方向往后移 back_dist（1.5m）。
          - 如果当前用后车架规划，则在当前 yaw 方向往前移 back_dist。
          - 保留原始 yaw，不改时间戳/坐标系一致性。
      - reset() 时无需改动，保持已有逻辑。
  - 生成左右偏航候选
      - 新增 helper geometry_msgs::PoseStamped yawOffset(const geometry_msgs::PoseStamped &pose, double yaw_delta_rad)，对输入姿态的
        yaw 加 ±20°（常量转弧度），wrap 到 [-π,π]。位置不变。
  - 规划主流程改造
      - planning_global_path() 第一阶段：仍然用原始起点/目标尝试一次（保留卸料前移 3.0m 等现有特例）。
      - 若失败，构造退让起点 offset_start（1.5m，方向按上文规则），并构造三个候选：
          1. offset_start + 原始 yaw；
          2. offset_start + yaw +20°；
      - 成功后把最佳结果写入 global_path_/nav_path_/segment_backward_flags_/
        segment_lengths_，并 transitionTo(AdjustingDirection,
        "...fallback...")；如全部失败再按现有逻辑 setError。
      - 日志中注明使用了“退到铰接中心 + 左/右 20°”的容错，以便调试。
  - 头文件声明
      - 在 planner_ros.hpp 声明新增 helper、PlanAttempt 结构（含 success、message、path 长度、direction_flags/segments 等缓存）。
  - 微调 planner_ros_legacy.cpp
      - 如果需要复用新 helper（如 makeOffsetStart、yawOffset），在该 TU 内包含实现或改为共用。

Primary 规划
    │
    ▼ 失败
Fallback: 偏移规划（3个yaw角度）
    │
    ▼ 全部失败
【新增】Fallback: 增加换向次数（max_switches=2）
    │
    ├─ 成功 → 使用该结果，恢复 max_switches=1
    │
    └─ 失败 → 恢复 max_switches=1，报 Error



# 尝试进行一次铲斗可视化的，终点碰撞检测
写一个ROS1的python脚本，利用当前的前车架位姿/odometry_qianlun，沿其航向向前0.7m（为铲斗中心），在该位置生成一个2m长，1m宽的方框（可视化到rviz中），并判断这个方框其是否会与地图上/map(nav_msgs/OccupancyGrid)的障碍物发生碰撞，检测频率控制在10hz。如果发生碰撞，发布bool=false到/able_to_unload话题中。


# 关于终止任务
新 Goal 到达
    │
    ▼
检查是否有活动 Goal？ ──是──▶ 调用 setPreempted() 抢占旧 Goal
    │                        │
    │                        ▼
    │                    旧 Goal 被标记为 "preempted"
    │                    has_active_goal_ = false
    │                        │
    └──────────否────────────┘
                │
                ▼
        acceptNewGoal() 接受新 Goal
                │
                ▼
        调用 planner_.onActionGoal(*goal)

旧 action 会被立即抢占（preempted），不会等待完成
旧 action 的 result 会被设置为 success=false，message 为 "New goal preempted previous active goal."
新 action 会被接受并开始执行


停车功能流程

新 Action 到达
      │
      ▼
onActionGoal()
      │
      ├─ 保存 last_id, current_id
      ├─ 清除刹车 (发布 0)
      └─ 进入 Moving 时发布 controller_started = true (使能移动)
      │
      ▼
kickoffTaskFromAction()
      │
      ▼
current_id == 0 ?
   ┌──────┴──────┐
   │ 是          │ 否
   ▼             ▼
停车模式       正常任务流程
   │
   ├─ controller_started = false (停止)
   ├─ brake = 30%
   └─ Completed (success=true)

# 前后车架在moving状态下的选择（仿真测试通过）（未review）
• Aligned the active pose source with the planned segment directions so backward
  segments use rear-frame data before moving.

  - src/ros/planner_ros.cpp: adjust_vehicle_direction now sets the pose source
    from the current segment’s backward flag and refreshes the pose before
    issuing is_forward commands; finalizeDirectionSwitch re-syncs the pose
    source to the confirmed desired_forward_ before entering Moving; onReach
    picks the next segment’s pose source from the remaining backward flags
    (warns and falls back to the old toggle only if flags are missing).


# 只在第一段原地调整
• Implemented the one-time AligningInPlace check on the first segment only.

  - Added align_check_done_ flag, reset in reset().
  - Guarded the AligningInPlace precheck in finalizeDirectionSwitch() with
    segment_index_ == 0 and the new flag; mark it true on first evaluation so
    later Moving transitions skip it.


• Guarding is now wired in (debug only) and logs as requested.
  - Added guard params to config/center_articulation_planner_params.yaml
    (goal_guard_distance_m=3.0, goal_guard_angle_deg=110.0,
    goal_guard_forward_deadzone_m=0.25) and load them in src/ros/
    planner_ros.cpp; new state fields/prototype live in include/
    center_articulation_planner/ros/planner_ros.hpp.
  - Implemented nearGoalUnreachable in src/ros/planner_ros.cpp to compute goal
    position in the vehicle frame (front/rear aware), evaluate angle wedge and
    forward deadzone, and return a reason string.
  - onMovingTimer now, when in debug mode and on the last segment within the
    guard distance, logs a single "guard enabled" line, then logs each cycle's
    check (remaining/forward/lateral/yaw_err/status). If unreachable, it stops
    the moving timer, publishes controller_started=false plus a 30% brake, and
    raises an error with the guard reason; guard flags reset in reset().

## 2026-02-09: 优化 align_in_place 原地调整判断逻辑（待测试）
**问题描述：**
原有逻辑在所有段使用欧氏距离判断是否需要原地调整，对于后续段不够合理。因为车辆已经在路径上时，纵向偏差（超前或滞后）不应触发原地调整，只有横向偏离轨道时才需要。

**解决方案：**
实施基于路径坐标系（Frenet坐标系）的分段判断逻辑：

1. **首段 (segment_index_ == 0)**：
   - 保持欧氏距离判断：`dist = hypot(dx, dy)`
   - 阈值参数：`align_pos_thresh_m_`
   - 原因：车辆从任意位置启动，欧氏距离合理

2. **后续段 (segment_index_ > 0)**：
   - 改用横向误差判断（Frenet 坐标系）
   - 建立段起点切线坐标系，计算横向偏差：
     ```cpp
     yaw_start = tf2::getYaw(segment_start_orientation)
     lateral_error = -dx_map * sin(yaw_start) + dy_map * cos(yaw_start)
     position_error = abs(lateral_error)
     ```
   - 阈值参数：`align_lateral_thresh_m_`（默认与 `align_pos_thresh_m_` 相同）
   - 原因：车辆已在路径上，横向偏差更能反映是否需要原地调整

**新增参数：**
- `align_lateral_thresh_m_`（默认1.0m，与 `align_pos_thresh_m_` 相同）

**代码修改位置：**
- [planner_ros.hpp:200](include/center_articulation_planner/ros/planner_ros.hpp#L200) - 添加 `align_lateral_thresh_m_` 成员变量
- [planner_ros.cpp:147](src/ros/planner_ros.cpp#L147) - 在 `initialize()` 中加载参数
- [planner_ros.cpp:1912-2022](src/ros/planner_ros.cpp#L1912-L2022) - 重构 `maybeEnterAligningInPlace()` 判断逻辑

**优势：**
- 减少不必要的原地调整（纵向超前/滞后不触发）
- 更符合路径跟踪控制的实际需求
- 横向误差是路径跟踪的关键指标


## 2026-03-31: non-first-segment remaining-path replan for large lateral deviation

- Added a pre-move decision stage in `finalizeDirectionSwitch()`: `Proceed`, `Align`, or `ReplanRemaining`.
- First segment is never replanned. It keeps the existing direct-move / align behavior.
- For `segment_index_ > 0`, the planner now projects the current pose onto `nav_path_.front()` and computes lateral / longitudinal / heading errors from the nearest path segment.
- If `abs(lateral_error)` exceeds `large_lateral_replan_thresh_m_`, the planner replans the remaining path from the current active pose source to the task goal, then re-enters `AdjustingDirection`.
- Remaining-path replan is limited by `max_remaining_replan_count_` and does not fall back to the old path on failure.


# TODO:
