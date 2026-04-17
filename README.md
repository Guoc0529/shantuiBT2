# shantuiBT2 - 自主装载行为树系统

基于 BehaviorTree.CPP 实现的自动驾驶装载机器人行为树控制系统。

## 功能概述

本系统实现了一种自动化物料装载机器人的行为树控制逻辑，主要功能包括：

- **自主导航**：基于 SafeNavigate 的安全导航控制
- **铲料作业**：自动接近铲料点并执行铲料动作
- **卸料作业**：到达料仓后自动卸料
- **状态管理**：通过 ArmBucketState 管理臂/铲状态

## 状态码说明

| 状态码 | 含义 | 说明 |
|--------|------|------|
| 0 | 停止/空闲 | 默认状态 |
| 1 | 复位/收斗 | 作业完成复位 |
| 2 | 准备 | 抬臂准备 |
| 3 | 下斗/放斗 | 到达铲料点前下放铲斗 |
| 4 | 抬臂 | 到达料仓前抬起铲斗 |
| 5 | 收臂 | 返回时收回臂 |
| 6 | 卸料 | 到达料仓执行卸料 |
| 7 | 铲料 | 到达铲料点执行铲料 |
| 8 | 后退等待 | 铲料后后退等待 |

## 项目结构

```
.
├── src/autonomous_loader_bt/          # 主功能包
│   ├── config/loader_config.yaml     # 行为树配置文件
│   ├── include/                       # 头文件
│   ├── src/                           # 源代码
│   │   ├── loader_bt_nodes.cpp       # 行为树节点实现
│   │   └── autonomous_loader_bt_node.cpp
│   └── trees/                        # 行为树 XML
│       └── autonomous_loader_tree.xml # 主行为树
├── build/                            # 编译目录（不提交）
├── devel/                            # 开发目录（不提交）
└── .catkin_tools/                    # catkin 工具配置（不提交）

```

## 编译

```bash
# 进入工作空间
cd /path/to/workspace

# 编译
catkin build

# 或使用 catkin_make
catkin_make
```

## 运行

```bash
# 加载环境
source devel/setup.bash

# 启动行为树节点
rosrun autonomous_loader_bt autonomous_loader_bt_node
```

## 配置

行为树参数在 `config/loader_config.yaml` 中配置：

- **距离阈值**：各作业阶段的距离判定阈值
- **超时时间**：各操作的超时设置
- **位置信息**：料仓、料仓、停车场位置

## 依赖

- ROS (Kinetic/Melodic/Noetic)
- BehaviorTree.CPP
- navigation_stack
- geometry_msgs
- move_base_msgs
