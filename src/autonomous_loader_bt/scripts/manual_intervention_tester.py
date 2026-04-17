#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manual_intervention_tester.py  (extended)
----------------------------------------
专门测试“人工介入”流程，并新增若干自动模拟能力：
1. 自动把 ArmBucketState 完成：检测到新的 LastABSCommand 后 1 秒，把
   ArmBucketState 置 10，省去 rosparam 手动操作。
2. 发送 Task 时自动请求 /spadingPose，保证铲料点非 (0,0)。也可按 `p`
   键手动请求。
3. 发送 `m` (manual_intervention_complete) 后自动把导航 mock 模式切回
   NORMAL，防止忘记。
4. 按 `s` 键时，可交互式选择导航模式并指定 bin/hopper ID。
"""
from __future__ import annotations
import math
import random
import sys
import select
import termios
import tty
import time
from enum import Enum
import logging

import rospy
import actionlib
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from autonomous_loader_msgs.msg import (
    NavigateAction,
    NavigateFeedback,
    NavigateResult,
    TaskCommand,
)
from autonomous_loader_msgs.srv import spadingPose

# ----------------------- 日志配置 ----------------------------
log_format = "[%(asctime)s.%(msecs)03d] [%(name)-11s] [%(levelname)-7s] %(message)s"
logging.basicConfig(level=logging.INFO, format=log_format, datefmt="%H:%M:%S")

# ----------------------- 导航模式 ----------------------------
class NavMode(Enum):
    NORMAL = 0           # 一切正常
    PLAN_FAIL = 1        # 规划失败 (致命错误)
    BAD_ARRIVAL = 2      # 到达但误差超阈值
    RECOVERABLE_FAIL = 3 # 可恢复的失败

MODE_LABEL = {
    NavMode.NORMAL: "NORMAL",
    NavMode.PLAN_FAIL: "PLAN_FAIL (FATAL)",
    NavMode.BAD_ARRIVAL: "BAD_ARRIVAL",
    NavMode.RECOVERABLE_FAIL: "RECOVERABLE_FAIL",
}

STATUS_LABELS = {0: "铲料", 1: "卸料", 2: "其他"}

# -------------------- 导航服务器模拟 --------------------------
class ManualInterventionMockNav:
    def __init__(self):
        self.log = logging.getLogger("MI-Nav")
        self._mode: NavMode = NavMode.NORMAL
        self._speed = rospy.get_param("~mock_nav_speed", 1.5)  # m/s
        self._srv = actionlib.SimpleActionServer(
            "/navigate", NavigateAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._feedback_rate = rospy.Rate(10)
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._srv.start()
        self.log.info("/navigate ActionServer started (speed=%.2f m/s)", self._speed)

    def set_mode(self, mode: NavMode):
        self._mode = mode
        self.log.warning("Switched navigation mode to %s", MODE_LABEL[mode])

    # ---------- Action 回调 ----------
    def _execute_cb(self, goal):
        mode = self._mode

        # Immediately send planning feedback based on the mode
        if mode in [NavMode.PLAN_FAIL, NavMode.RECOVERABLE_FAIL]:
            planning_feedback = NavigateFeedback(plan_succeeded=False)
            self._srv.publish_feedback(planning_feedback)
        else:
            planning_feedback = NavigateFeedback(plan_succeeded=True)
            self._srv.publish_feedback(planning_feedback)

        if mode == NavMode.PLAN_FAIL:
            self.log.warning("Simulating PLAN FAILURE for goal (%.2f, %.2f)",
                          goal.end_pose.pose.position.x, goal.end_pose.pose.position.y)
            result = NavigateResult(success=False, 
                                    error_code=101, error_msg="Fatal: Global planner failed (mock)")
            self._srv.set_aborted(result, "planning failed (mock)")
            return

        if mode == NavMode.RECOVERABLE_FAIL:
            self.log.warning("Simulating RECOVERABLE FAILURE for goal (%.2f, %.2f)",
                          goal.end_pose.pose.position.x, goal.end_pose.pose.position.y)
            result = NavigateResult(success=False, 
                                    error_code=102, error_msg="Recoverable: Robot temporarily stuck (mock)")
            self._srv.set_aborted(result, "recoverable failure (mock)")
            return

        gx, gy = goal.end_pose.pose.position.x, goal.end_pose.pose.position.y
        dx, dy = gx - self._cur_x, gy - self._cur_y
        dist = max(math.hypot(dx, dy), 0.1)
        duration = dist / self._speed
        st = rospy.Time.now()
        last_s = getattr(goal, "last_status", 2)
        cur_s  = getattr(goal, "current_status", 2)
        self.log.info("Goal (%.2f,%.2f) status %s->%s travel %.2fs mode=%s",
                      gx, gy, STATUS_LABELS.get(last_s, '?'), STATUS_LABELS.get(cur_s, '?'), duration, MODE_LABEL[mode])

        while not rospy.is_shutdown():
            if self._srv.is_preempt_requested():
                self._srv.set_preempted()
                return
            prog = min((rospy.Time.now()-st).to_sec()/duration, 1.0)
            fb = NavigateFeedback(distance_to_goal=dist*(1.0-prog), plan_succeeded=True)
            self._srv.publish_feedback(fb)
            if prog >= 1.0:
                break
            self._feedback_rate.sleep()

        self._cur_x, self._cur_y = gx, gy
        result = NavigateResult(success=True,
                                final_distance_error=0.1, final_angle_error=1.0,
                                error_code=0, error_msg="Success")
        if mode == NavMode.BAD_ARRIVAL:
            result.final_distance_error = 3.0
            result.final_angle_error    = 45.0
        self._srv.set_succeeded(result, "goal reached (mock)")
        self.log.info("Goal reached, result sent (mode %s)", MODE_LABEL[mode])

# -------------------- 交互控制器 ------------------------------
class ManualInterventionTester:
    def __init__(self, nav_mock: ManualInterventionMockNav):
        self.log = logging.getLogger("MI-Tester")
        self.bt_log = logging.getLogger("BT-Status")
        self.nav_mock = nav_mock
        bt_ns = rospy.get_param("~bt_ns", "/autonomous_loader_bt_node").rstrip("/")
        self.bt_ns = bt_ns
        self.task_pub = rospy.Publisher(f"{bt_ns}/scheduler/start_task", TaskCommand, queue_size=10, latch=True)
        self.end_pub  = rospy.Publisher(f"{bt_ns}/scheduler/end_task", Bool, queue_size=1)
        self.mi_pub   = rospy.Publisher("/autonomous_loader/manual_intervention_complete", Bool, queue_size=1)
        self.status_sub = rospy.Subscriber(f"{bt_ns}/state_machine/status", String, self._status_cb)

        # 服务 client
        self._spade_cli = rospy.ServiceProxy("/spadingPose", spadingPose)

        # 自动 ArmBucketState 完成
        self._abs_last_cmd = 0
        self._abs_timer = rospy.Timer(rospy.Duration(0.2), self._abs_watch_cb)

        self._print_help()

    # ------------ 自动 ABS 完成 -------------
    def _abs_watch_cb(self, _):
        last_cmd = rospy.get_param(f"{self.bt_ns}/LastABSCommand", 0)
        state    = rospy.get_param(f"{self.bt_ns}/ArmBucketState", 10)
        if last_cmd != 0 and last_cmd != self._abs_last_cmd:
            self._abs_last_cmd = last_cmd
            self._abs_stamp    = time.time()
        if hasattr(self, "_abs_stamp") and time.time()-self._abs_stamp > 1.0 and state != 10:
            rospy.set_param(f"{self.bt_ns}/ArmBucketState", 10)
            self.log.info("Auto-set ArmBucketState=10 (mock done)")

    # ----------- helper: 请求铲料点 -------------
    def _req_scoop(self, bin_id: int):
        try:
            resp = self._spade_cli(bin_id)
            self.log.info("scoopPoint(%.2f, %.2f)",
                          resp.pose_output.position.x, resp.pose_output.position.y)
        except rospy.ServiceException:
            self.log.warning("/spadingPose call failed")

    # ----------- 键盘触发动作 -------------
    def send_task(self, bin_id:int, hopper_id:int):
        msg = TaskCommand(task_id=random.randint(100,999), bin_id=bin_id, hopper_id=hopper_id, task_type="scoop")
        self.task_pub.publish(msg)
        self.log.info("Sent Task #%d (bin=%d hopper=%d)", msg.task_id, msg.bin_id, msg.hopper_id)
        self._req_scoop(msg.bin_id)

    def send_end(self):
        self.end_pub.publish(Bool(data=True))
        self.log.info("Sent End Task")

    def send_mi_done(self):
        self.mi_pub.publish(Bool(data=True))
        self.log.info("manual_intervention_complete=True")
        self.nav_mock.set_mode(NavMode.NORMAL)

    # ----------- 状态回调 -------------
    def _status_cb(self, msg: String):
        t = msg.data
        if "Idle" in t:
            self.task_active = False
            self.log.info("Detected Idle status. Ready for new task.")

        if t in ["RUNNING","SUCCESS","FAILURE","IDLE","SKIPPED"]:
            return
        self.bt_log.warning(t)

    # ----------- 帮助 -------------
    def _print_help(self):
        print("\n"+"="*50)
        print("Manual Intervention Tester")
        print("="*50)
        print("  s : Send new Task (interactive)")
        print("  m : Manual intervention done (auto NORMAL)")
        print("  e : EndTask      h : help      q : quit")
        print("="*50)

# -------------------- 主循环 ------------------------------

def main():
    rospy.init_node("manual_intervention_tester")
    nav = ManualInterventionMockNav()
    tester = ManualInterventionTester(nav)

    old = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key=='m': tester.send_mi_done()
                elif key=='e': tester.send_end()
                elif key=='h': tester._print_help()
                elif key=='q': break
                elif key=='s':
                    # 暂时恢复终端，读取用户输入
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
                    try:
                        print("\n--- Send New Task ---")
                        # 1. 选择导航模式
                        mode_in = input("Select nav mode [(f)atal, (r)ecoverable, (a)rrival_bad, (n)ormal]: ").lower()
                        if   mode_in == 'f': nav.set_mode(NavMode.PLAN_FAIL)
                        elif mode_in == 'r': nav.set_mode(NavMode.RECOVERABLE_FAIL)
                        elif mode_in == 'a': nav.set_mode(NavMode.BAD_ARRIVAL)
                        else: nav.set_mode(NavMode.NORMAL)
                        
                        # 2. 输入 bin/hopper ID
                        bi = int(input("Enter bin_id  (1-3): "))
                        hi = int(input("Enter hopper_id (1-5): "))
                        tester.send_task(bi, hi)

                    except (ValueError, IndexError):
                        print("Invalid input, task cancelled.")
                    finally:
                        # 重新 raw 模式
                        tty.setraw(sys.stdin.fileno())
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
