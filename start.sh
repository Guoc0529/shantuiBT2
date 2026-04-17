#!/bin/bash

# 1. 定义工作空间路径
WS_PATH="/home/nvidia/behaviorTreev4_ws"

# 定义要在每个 Tab 中执行的环境加载命令
SOURCE_CMD="source /opt/ros/noetic/setup.bash; source $WS_PATH/devel/setup.bash"

echo "--- 正在检查 roscore ---"
if ! pgrep -x "roscore" > /dev/null; then
    # roscore 也可以单独开一个 Tab，或者直接后台运行
    roscore &
    sleep 3
fi

echo "--- 正在打开新选项卡启动 Launch 文件 ---"

# 启动第一个 Tab: Autonomous Loader BT
# --tab: 在新选项卡打开
# --title: 设置选项卡标题
# bash -c: 执行后面引号里的命令
# exec bash: 保证程序运行完或崩溃后，Tab 窗口不会直接关闭，方便看报错
gnome-terminal --tab --title="Loader_BT" -- bash -c "$SOURCE_CMD; roslaunch autonomous_loader_bt autonomous_loader_bt.launch; exec bash"

# 等待 5 秒，确保行为树初始化
sleep 5

# 启动第二个 Tab: Obstacle Avoider
gnome-terminal --tab --title="Obstacle_Avoider" -- bash -c "$SOURCE_CMD; roslaunch obstacle_avoider avoider.launch; exec bash"

echo "任务已分配到各终端选项卡。"
