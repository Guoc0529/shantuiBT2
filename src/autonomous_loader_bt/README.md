# 自主装载机器人行为树ROS1功能包（优化版）

## 概述
这个ROS功能包实现了自主装载机器人的行为树逻辑，**状态机专注于协调各个模块而不执行具体功能**。功能包采用模块化设计，通过ROS话题进行通信，支持YAML配置文件，实现了完整的铲料工作流程。

## 主要特性

### 🎯 状态机设计理念
- **状态机只做协调**：不执行具体功能，只负责发送指令和接收状态反馈
- **模块化架构**：各功能模块独立运行，通过ROS话题通信
- **响应式优先级**：使用ReactiveSelector确保高优先级指令立即响应

### 🔧 核心功能
- ✅ **调度功能**：开始任务、暂停任务、结束任务
- ✅ **工作装置**：抬大臂、放平铲斗指令
- ✅ **铲料模块**：铲料指令和完成状态
- ✅ **导航模块**：路径规划、到位检测、距离监控
- ✅ **铲料点计算**：智能铲料点请求和响应
- ✅ **YAML配置**：料仓、料斗位置和工作参数配置

### 🌳 行为树结构
```
ReactiveSelector (ROOT)
├── 停止处理 (最高优先级)
│   ├── CheckEndTask
│   ├── SendNavigationGoalParking
│   └── UpdateTaskProgress
├── 暂停处理 (中等优先级)
│   ├── CheckPauseTask
│   ├── PauseNavigation
│   └── UpdateTaskProgress
└── 工作/空闲 (常规优先级)
    ├── 任务循环
    │   ├── 准备阶段 (抬大臂)
    │   ├── 铲料阶段 (请求铲料点 → 导航 → 铲料)
    │   ├── 卸料阶段 (导航到料斗 → 卸料)
    │   └── 检查下一循环
    ├── 停车 (无任务时)
    └── 空闲 (保持状态)
```

## 目录结构
```
autonomous_loader_bt/
├── launch/                    # 启动文件
│   └── autonomous_loader_bt.launch
├── config/                    # 配置文件
│   ├── loader_config.yaml     # 料仓料斗位置配置
│   └── autonomous_loader.rviz # RViz配置
├── include/autonomous_loader_bt/
│   └── loader_bt_nodes.h      # 行为树节点定义
├── src/                       # 源代码
│   ├── autonomous_loader_bt_node.cpp  # 主ROS节点
│   └── loader_bt_nodes.cpp    # 行为树节点实现
├── scripts/                   # 测试脚本
│   └── module_simulator.py    # 模块模拟器
├── trees/                     # 行为树定义
│   └── autonomous_loader_tree.xml
├── docs/                      # 文档
│   └── ros_topics_design.md   # ROS话题设计文档
├── CMakeLists.txt
├── package.xml
└── README.md
```

## ROS话题接口

### 调度模块话题
- **订阅**：
  - `/scheduler/pause_task` (std_msgs/Bool) - 暂停任务
  - `/scheduler/end_task` (std_msgs/Bool) - 结束任务

### 工作装置模块话题
- **发布**：
  - `/work_device/raise_arm` (std_msgs/Bool) - 抬大臂指令
  - `/work_device/lower_bucket` (std_msgs/Bool) - 放平铲斗指令
- **订阅**：
  - `/work_device/task_completed` (std_msgs/Bool) - 任务完成状态
  - `/work_device/arm_raised` (std_msgs/Bool) - 抬大臂完成回令
  - `/work_device/bucket_lowered` (std_msgs/Bool) - 放平铲斗完成回令

### 铲料模块话题
- **发布**：
  - `/scoop/start_scoop` (std_msgs/Bool) - 开始铲料指令
- **订阅**：
  - `/scoop/scoop_completed` (std_msgs/Bool) - 铲料完成回令

### 导航模块话题
- **发布**：
  - `/navigation/goal` (geometry_msgs/PoseStamped) - 导航目标
  - `/navigation/pause` (std_msgs/Bool) - 暂停导航
- **订阅**：
  - `/navigation/planning_success` (std_msgs/Bool) - 路径规划成功
  - `/navigation/arrived` (std_msgs/Bool) - 到达目标位置
  - `/navigation/distance_to_goal` (std_msgs/Float64) - 距离目标点距离
  - `/navigation/current_pose` (geometry_msgs/PoseStamped) - 当前机器人位置

### 状态机话题
- **发布**：
  - `/state_machine/status` (std_msgs/String) - 状态机当前状态
  - `/workstate` (std_msgs/Int8) - 任务状态：0空闲 1执行中 2暂停中 3结束中 4已结束

## 工作流程

### 铲料过程
1. **调度下发任务** → 状态机接收任务
2. **状态机发送抬大臂指令** → 工作装置模块
3. **工作装置完成** → 状态机请求铲料点位置
4. **铲料点处理**：
   - 如果收到(0,0)：导航到料仓中心 → 重新请求铲料点
   - 如果收到有效位置：直接导航到铲料点
5. **铲料执行**：
   - 距离6米：发送放平铲斗指令
   - 距离3米：发送铲料指令

### 卸料过程
1. **铲料完成** → 状态机发送料斗位置给导航
2. **距离6米** → 发送抬大臂指令
3. **距离0米** → 发送卸料指令
4. **卸料完成** → 检查是否有新任务

## 构建和运行

### 构建
```bash
cd ~/shantui/behaviorTreev4_ws
catkin build autonomous_loader_bt
source devel/setup.bash
```

### 运行
1. **启动主节点**（在一个终端）：
   ```bash
   roslaunch autonomous_loader_bt autonomous_loader_bt.launch
   ```

2. **启动模块模拟器**（在另一个终端）：
   ```bash
   rosrun autonomous_loader_bt module_simulator.py
   ```

3. **模拟器按键控制**：
   - `s` - 发送开始任务
   - `p` - 发送暂停任务
   - `e` - 发送结束任务
   - `r` - 模拟工作装置完成
   - `c` - 模拟铲料完成
   - `n` - 模拟导航到达
   - `q` - 退出

4. **调度通过Service下发任务**：
   - 服务端（状态机）名称：`/liaodou`（类型：`shuju/cangdou`）
   - 请求：`{taskID: int8, cang: int8, dou: int8}`，响应：`{huiying: bool}`
   - 示例调用：
   ```bash
   rosservice call /liaodou "taskID: 1
   cang: 1
   dou: 1"
   ```
   - **注意**：收到任务后，状态机会自动将料仓编号（`cang`）发布到ROS参数服务器，参数名为`canggoal`（int类型）

### 监控状态
```bash
# 查看状态机状态
rostopic echo /state_machine/status

# 查看任务状态（0空闲 1执行中 2暂停中 3结束中 4已结束）
rostopic echo /workstate

# 查看料仓编号参数（canggoal）
rosparam get /autonomous_loader_bt_node/canggoal

# 查看导航距离
rostopic echo /navigation/distance_to_goal

# 查看所有相关话题
rostopic list | grep -E '(scheduler|work_device|scoop|navigation|state_machine)'
```

## 配置文件

### loader_config.yaml（适配 shuju::cangdou）
```yaml
# 料仓（cang）与料斗（dou）键与服务请求参数一致
cang:
  1: { x: 10.0, y: 10.0, z: 0.0, yaw: 0.0 }

dou:
  1: { x: 5.0, y: 5.0, z: 0.0, yaw: 0.0 }

# 工作参数配置
work_parameters:
  distance_thresholds:
    raise_arm_distance: 6.0      # 抬大臂距离
    lower_bucket_distance: 6.0   # 放平铲斗距离
    start_scoop_distance: 3.0    # 开始铲料距离
    start_dump_distance: 0.0      # 开始卸料距离
```

## 测试验证

### 完整工作流程测试

运行完整测试脚本（自动测试模式）：
```bash
# 1. 启动状态机节点（在一个终端）
roslaunch autonomous_loader_bt autonomous_loader_bt.launch

# 2. 运行完整测试脚本（在另一个终端）
rosrun autonomous_loader_bt test_full_workflow.py
```

测试脚本会自动：
- 调用 `/liaodou` 服务下发任务
- 验证 `canggoal` 参数是否正确设置
- 验证 `workstate` 话题和参数是否正确更新
- 自动模拟所有模块的回令（工作装置、铲料、导航等）
- 完成整个铲料-卸料流程

### 交互式测试模式

```bash
rosrun autonomous_loader_bt test_full_workflow.py --interactive
```

交互式命令：
- `test <taskID> <cang> <dou>` - 发送测试任务
- `check canggoal` - 检查 canggoal 参数
- `check workstate` - 检查 workstate 参数
- `status` - 显示当前状态
- `quit` - 退出

### 其他测试脚本

运行模块模拟器（手动测试）：
```bash
rosrun autonomous_loader_bt module_simulator.py
```

## 技术特点

### 🎯 状态机设计
- **纯协调功能**：状态机不执行具体动作，只发送指令和接收反馈
- **模块解耦**：各功能模块独立运行，便于维护和扩展
- **实时响应**：ReactiveSelector确保高优先级指令立即处理

### 🔄 工作流程优化
- **智能铲料点处理**：自动处理(0,0)情况，导航到料仓中心重新请求
- **距离监控**：实时监控到目标点距离，精确控制动作时机
- **循环任务支持**：支持连续执行多个铲卸任务

### 📊 监控和调试
- **状态发布**：实时发布状态机状态和任务进度
- **日志记录**：支持控制台和文件日志记录
- **模块模拟器**：提供完整的模块功能模拟

## 依赖项
- ROS Noetic
- BehaviorTree.CPP v4
- yaml-cpp
- tf2
- geometry_msgs
- std_msgs
