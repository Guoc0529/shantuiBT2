#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
interactive_tester.py
=====================

一个交互式测试脚本，用于手动测试自主装载机器人行为树（BT）状态机。

功能:
1.  **代替调度模块**:
    - 通过键盘按键 ('s') 手动发送开始任务指令。
    - 通过键盘按键 ('e') 手动发送结束任务指令。

2.  **代替导航模块**:
    - 启动一个假的 /navigate ActionServer（autonomous_loader_msgs/NavigateAction）。会按 10Hz 发布 distance_to_goal 反馈，直到 0 后返回成功。
    - 自动接受导航目标，模拟移动过程（发布feedback），并在到达后返回成功结果。

3.  **不模拟工作模块**:
    - 此脚本不会读写 ArmBucketState 或 PointAccumulation 参数。
    - 你需要在另一个终端使用 `rosparam set` 手动模拟工作模块的完成状态。

"""

import math
import random
import sys
import select
import termios
import tty

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from autonomous_loader_msgs.msg import NavigateAction, NavigateFeedback, NavigateResult
from std_msgs.msg import Bool
from autonomous_loader_msgs.msg import TaskCommand

STATUS_LABELS = {
    0: "铲料",
    1: "卸料",
    2: "其他"
}


def format_status(code):
    return STATUS_LABELS.get(code, f"未知({code})")


class NavigateMockServer:
    """简易的 NavigateAction ActionServer，用于模拟导航反馈与结果。"""

    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "/navigate", NavigateAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._speed = rospy.get_param("~mock_nav_speed", 1.5)  # m/s
        self._feedback_rate = rospy.Rate(10)
        self._current_x = 0.0
        self._current_y = 0.0
        self._server.start()
        rospy.loginfo("[MockNav] /navigate action server started (speed=%.2f m/s)", self._speed)

    def _execute_cb(self, goal):
        goal_x = goal.end_pose.pose.position.x
        goal_y = goal.end_pose.pose.position.y
        dx = goal_x - self._current_x
        dy = goal_y - self._current_y
        distance = max(math.hypot(dx, dy), 0.1)
        duration = distance / self._speed
        start_time = rospy.Time.now()

        last_status_code = getattr(goal, 'last_status', 2)
        current_status_code = getattr(goal, 'current_status', 2)
        rospy.loginfo("[MockNav] Received goal (%.2f, %.2f), status: %s -> %s, simulating %.2f sec travel.",
                      goal_x, goal_y,
                      format_status(last_status_code),
                      format_status(current_status_code),
                      duration)

        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                rospy.logwarn("[MockNav] Goal preempted")
                self._server.set_preempted()
                return

            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(elapsed / duration, 1.0)
            current_x = self._current_x + dx * progress
            current_y = self._current_y + dy * progress

            feedback = NavigateFeedback()
            # Publish remaining distance to goal
            dist_remaining = max(distance * (1.0 - progress), 0.0)
            feedback.distance_to_goal = dist_remaining
            self._server.publish_feedback(feedback)

            if progress >= 1.0:
                break

            self._feedback_rate.sleep()

        self._current_x = goal_x
        self._current_y = goal_y
        rospy.loginfo("[MockNav] Goal reached at (%.2f, %.2f)", self._current_x, self._current_y)
        result = NavigateResult()
        result.plan_succeeded = True
        result.success = True
        self._server.set_succeeded(result, "goal reached")


class InteractiveTester:
    """提供键盘接口来模拟调度模块。"""

    def __init__(self):
        bt_ns = rospy.get_param("~bt_ns", "/autonomous_loader_bt_node").rstrip("/")
        start_topic = f"{bt_ns}/scheduler/start_task"
        end_topic = f"{bt_ns}/scheduler/end_task"

        self._task_pub = rospy.Publisher(start_topic, TaskCommand, queue_size=10, latch=True)
        self._end_pub = rospy.Publisher(end_topic, Bool, queue_size=1)

        self.print_help()

    def send_task(self, bin_id=None, hopper_id=None):
        """发送任务。如果 bin_id 和 hopper_id 为 None，则使用随机值。"""
        task = TaskCommand()
        task.task_id = random.randint(100, 999)
        task.bin_id = bin_id if bin_id is not None else random.randint(1, 2)
        task.hopper_id = hopper_id if hopper_id is not None else random.randint(1, 5)
        task.task_type = "scoop"

        self._task_pub.publish(task)
        rospy.loginfo(f"[Tester] Sent Task #{task.task_id} (bin={task.bin_id}, hopper={task.hopper_id})")

    def send_end_task(self):
        msg = Bool()
        msg.data = True
        self._end_pub.publish(msg)
        rospy.loginfo("[Tester] Sent End Task command.")

    def print_help(self):
        print("\n" + "="*40)
        print("  Interactive Tester for Behavior Tree")
        print("="*40)
        print("  s: Send a new task (will prompt for bin_id and hopper_id)")
        print("  e: Send end task command")
        print("  h: Show this help message")
        print("  q: Quit")
        print("-"*40)
        print("  To simulate work modules, use another terminal:")
        print("  rosparam set /autonomous_loader_bt_node/ArmBucketState 10")
        print("  rosparam set /autonomous_loader_bt_node/PointAccumulation 10")
        print("="*40)


def get_user_input(prompt, old_settings):
    """临时恢复终端设置，获取用户输入，然后恢复原始模式。"""
    # 恢复终端设置以允许正常输入
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    try:
        print(f"\n{prompt}", end='', flush=True)
        user_input = input().strip()
        return user_input
    except (EOFError, KeyboardInterrupt):
        return None
    finally:
        # 恢复原始输入模式
        tty.setraw(sys.stdin.fileno())


def main():
    rospy.init_node("interactive_tester")
    
    # 启动假的导航服务
    _navigate_mock = NavigateMockServer()
    
    # 启动交互控制器
    tester = InteractiveTester()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == 's':
                    # 获取用户输入的 bin_id 和 hopper_id
                    user_input = get_user_input("请输入 bin_id 和 hopper_id (用空格分隔，直接回车使用随机值): ", old_settings)
                    if user_input is None:
                        continue
                    
                    if user_input.strip() == '':
                        # 用户直接回车，使用随机值
                        tester.send_task()
                    else:
                        try:
                            parts = user_input.split()
                            if len(parts) >= 2:
                                bin_id = int(parts[0])
                                hopper_id = int(parts[1])
                                tester.send_task(bin_id=bin_id, hopper_id=hopper_id)
                            elif len(parts) == 1:
                                # 只输入了一个数字，作为 bin_id，hopper_id 随机
                                bin_id = int(parts[0])
                                tester.send_task(bin_id=bin_id)
                            else:
                                rospy.logwarn("[Tester] 输入格式错误，使用随机值")
                                tester.send_task()
                        except ValueError:
                            rospy.logwarn("[Tester] 输入格式错误，使用随机值")
                            tester.send_task()
                elif key == 'e':
                    tester.send_end_task()
                elif key == 'h':
                    tester.print_help()
                elif key == 'q':
                    rospy.loginfo("Exiting...")
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    main()

