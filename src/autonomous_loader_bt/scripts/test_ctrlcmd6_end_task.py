#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_ctrlcmd6_end_task.py
-------------------------
专门测试 ctrlCmd=6（结束任务）功能的自动化测试脚本。

测试场景：
  TEST 1: 任务执行中途下发 ctrlCmd=6
  TEST 2: 当前任务结束后下发 ctrlCmd=6

测试流程（每个场景）：
  1. 等待系统进入 IDLE 状态
  2. 下发一个铲料任务（bin=1, hopper=1），自动请求 spadingPose
  3. 等待导航到达铲料点
  4. 等待 ArmBucketState 变为 10（自动 mock 完成）
  5. 等待导航到达卸料点
  6. [TEST1] 在此期间下发 ctrlCmd=6  → 期望：树halt后状态变为7
     [TEST2] 卸料完成后下发 ctrlCmd=6  → 期望：树halt后状态变为7
  7. 等待任务完成，确认状态=7
  8. 等待树回到 IDLE

用法：
  rosrun autonomous_loader_bt test_ctrlcmd6_end_task.py [test_id]
  不带参数：依次运行 TEST 1 和 TEST 2
  带参数 1：只运行 TEST 1（中途结束）
  带参数 2：只运行 TEST 2（完成后结束）
"""
from __future__ import annotations
import math
import sys
import time
import os
import logging
import threading

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
from shuju.msg import TaskCtrl

# ----------------------- 日志配置 ----------------------------
LOG_DIR = os.path.expanduser("~/.ros/log_ctrlcmd6_test")
os.makedirs(LOG_DIR, exist_ok=True)

ts = time.strftime("%Y%m%d_%H%M%S")
log_file = os.path.join(LOG_DIR, f"ctrlcmd6_test_{ts}.log")

file_handler = logging.FileHandler(log_file, encoding="utf-8")
file_handler.setFormatter(logging.Formatter(
    "[%(asctime)s.%(msecs)03d] [%(name)-12s] [%(levelname)-7s] %(message)s",
    datefmt="%H:%M:%S"))
stdout_handler = logging.StreamHandler()
stdout_handler.setFormatter(logging.Formatter(
    "\033[1m[%(name)-12s]\033[0m [%(levelname)-7s] %(message)s"))

log = logging.getLogger("TEST")
log.setLevel(logging.DEBUG)
log.addHandler(file_handler)
log.addHandler(stdout_handler)

bt_log = logging.getLogger("BT")
bt_log.setLevel(logging.DEBUG)
bt_log.addHandler(file_handler)

# ----------------------- 常量 ----------------------------
CTRL_CMD_NAMES = {
    1: "开始任务", 2: "远程接管", 3: "避障停车",
    4: "紧急停车", 5: "继续任务", 6: "结束任务",
}
WORKSTATE_NAMES = {
    1: "IDLE/Standby", 2: "Auto", 3: "Temp/Takeover",
    6: "Dump", 7: "Ending", 8: "Ended"
}

# ----------------------- 导航 Mock --------------------------
class NavMock:
    SPEED = 2.0  # m/s

    def __init__(self):
        self._srv = actionlib.SimpleActionServer(
            "/navigate", NavigateAction,
            execute_cb=self._execute_cb, auto_start=False
        )
        self._rate = rospy.Rate(20)
        self._cur_x, self._cur_y = 0.0, 0.0
        self._preempt = False
        self._preempt_lock = threading.Lock()
        self._srv.start()
        log.warning("Mock Nav ActionServer started (speed=%.1f m/s)", self.SPEED)

    def reset_pos(self):
        self._cur_x, self._cur_y = 0.0, 0.0

    def check_preempt(self):
        with self._preempt_lock:
            return self._preempt

    def trigger_preempt(self):
        with self._preempt_lock:
            self._preempt = True

    def _execute_cb(self, goal):
        self._preempt = False
        gx = goal.end_pose.pose.position.x
        gy = goal.end_pose.pose.position.y

        if abs(gx) < 0.001 and abs(gy) < 0.001:
            log.error("Goal is (0,0) — INVALID! Test should request spadingPose first!")
            result = NavigateResult(success=False, error_code=999,
                                    error_msg="Invalid goal (0,0)")
            self._srv.set_aborted(result, "invalid goal")
            return

        dx, dy = gx - self._cur_x, gy - self._cur_y
        dist = math.hypot(dx, dy)
        duration = max(dist / self.SPEED, 0.5)
        start = rospy.Time.now()

        log.info("Nav goal (%.2f, %.2f) dist=%.2fm duration=%.1fs",
                  gx, gy, dist, duration)

        feedback = NavigateFeedback(plan_succeeded=True, distance_to_goal=dist)
        self._srv.publish_feedback(feedback)

        while not rospy.is_shutdown():
            if self._srv.is_preempt_requested():
                log.info("Nav PREEMPTED — robot stopped at (%.2f, %.2f)",
                         self._cur_x, self._cur_y)
                self._srv.set_preempted()
                return
            elapsed = (rospy.Time.now() - start).to_sec()
            prog = min(elapsed / duration, 1.0)
            fb = NavigateFeedback(
                plan_succeeded=True,
                distance_to_goal=dist * (1.0 - prog)
            )
            self._srv.publish_feedback(fb)
            if prog >= 1.0:
                break
            self._rate.sleep()

        self._cur_x, self._cur_y = gx, gy
        result = NavigateResult(
            success=True,
            final_distance_error=0.1, final_angle_error=1.0,
            error_code=0, error_msg="Success"
        )
        self._srv.set_succeeded(result, "reached")
        log.info("Nav SUCCEEDED at (%.2f, %.2f)", self._cur_x, self._cur_y)


# ----------------------- 测试框架 --------------------------
class CtrlCmd6Tester:
    BT_NS = "/autonomous_loader_bt_node"

    def __init__(self, nav: NavMock):
        self.nav = nav
        self.task_pub = rospy.Publisher(
            f"{self.BT_NS}/scheduler/start_task", TaskCommand,
            queue_size=10, latch=True)
        self.ctrl_pub = rospy.Publisher(
            "/task_ctrl_command", TaskCtrl, queue_size=10)
        self.status_sub = rospy.Subscriber(
            f"{self.BT_NS}/state_machine/status", String, self._status_cb)

        self.spade_cli = rospy.ServiceProxy("/spadingPose", spadingPose)

        self.bt_state = "UNKNOWN"
        self.bt_state_lock = threading.Lock()
        self.work_state = 1
        self.work_state_lock = threading.Lock()

        self._abs_last_cmd = 0
        self._abs_timer = rospy.Timer(
            rospy.Duration(0.2), self._abs_watch_cb)

        self._ctrlcmd6_sent = threading.Event()
        self._ctrlcmd6_time = 0.0

        rospy.sleep(0.3)

    def _status_cb(self, msg: String):
        with self.bt_state_lock:
            self.bt_state = msg.data
        bt_log.info("BT status: %s", msg.data)

        if "Idle" in msg.data or "IDLE" in msg.data:
            log.info("BT entered IDLE")

        if msg.data not in ["RUNNING", "SUCCESS", "FAILURE", "IDLE",
                            "SKIPPED", "UNKNOWN", "WAITING"]:
            bt_log.warning("BT: %s", msg.data)

    def _abs_watch_cb(self, _):
        last_cmd = rospy.get_param(
            f"{self.BT_NS}/LastABSCommand", 0)
        state = rospy.get_param(
            f"{self.BT_NS}/ArmBucketState", 10)
        if last_cmd != 0 and last_cmd != self._abs_last_cmd:
            self._abs_last_cmd = last_cmd
            self._abs_stamp = time.time()
        if hasattr(self, "_abs_stamp") and \
                time.time() - self._abs_stamp > 1.0 and state != 10:
            rospy.set_param(f"{self.BT_NS}/ArmBucketState", 10)
            log.info("Auto ArmBucketState=10")

    def wait_bt_state(self, target, timeout=30.0):
        """等待 BT 状态变为包含 target 的字符串"""
        log.info("Waiting for BT state containing '%s' (timeout=%.0fs)",
                 target, timeout)
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.bt_state_lock:
                if target.upper() in self.bt_state.upper():
                    log.info("BT state matched: '%s'", self.bt_state)
                    return True
            rospy.sleep(0.2)
        log.error("Timeout waiting for BT state '%s' (got '%s')",
                  target, self.bt_state)
        return False

    def wait_workstate(self, target, timeout=30.0):
        """等待 workstate 变为目标值"""
        log.info("Waiting for workstate=%d (timeout=%.0fs)", target, timeout)
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            ws = rospy.get_param(f"{self.BT_NS}/workstate", -1)
            if ws == target:
                log.info("workstate=%d reached", target)
                return True
            rospy.sleep(0.2)
        ws = rospy.get_param(f"{self.BT_NS}/workstate", -1)
        log.error("Timeout waiting for workstate=%d (got %d)", target, ws)
        return False

    def send_task(self, task_id=1, bin_id=1, hopper_id=1):
        """发送任务（spadingPose 由行为树的 RequestScoopPoint 节点负责获取）"""
        msg = TaskCommand(
            task_id=task_id, bin_id=bin_id,
            hopper_id=hopper_id, task_type="scoop")
        self.task_pub.publish(msg)
        log.info("Sent Task #%d (bin=%d hopper=%d)", task_id, bin_id, hopper_id)
        return True

    def send_ctrl_cmd(self, ctrl_cmd: int, task_id: int = 1):
        """发送 TaskCtrl 命令"""
        msg = TaskCtrl()
        msg.taskID = task_id
        msg.ctrlCmd = ctrl_cmd
        msg.target_x = 0.0
        msg.target_y = 0.0
        msg.target_yaw = 0.0
        msg.has_location = False
        msg.timestamp = time.time()
        self.ctrl_pub.publish(msg)
        name = CTRL_CMD_NAMES.get(ctrl_cmd, f"#{ctrl_cmd}")
        log.info("★ Sent ctrlCmd=%d (%s)", ctrl_cmd, name)
        if ctrl_cmd == 6:
            self._ctrlcmd6_sent.set()
            self._ctrlcmd6_time = time.time()

    def wait_nav_arrived(self, phase: str, timeout=60.0):
        """等待导航到达（navigation_arrived=True）"""
        log.info("Waiting for nav arrival [%s] (timeout=%.0fs)", phase, timeout)
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            arrived = rospy.get_param(
                f"{self.BT_NS}/navigation_arrived", False)
            if arrived:
                dist = rospy.get_param(
                    f"{self.BT_NS}/distance_to_goal", 999.0)
                log.info("Nav arrived [%s] — dist_to_goal=%.2f", phase, dist)
                return True
            rospy.sleep(0.2)
        log.warning("Timeout waiting for nav arrival [%s]", phase)
        return False

    def wait_abs_done(self, timeout=30.0):
        """等待 ArmBucketState 变为 10"""
        log.info("Waiting for ArmBucketState=10 (timeout=%.0fs)", timeout)
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            state = rospy.get_param(f"{self.BT_NS}/ArmBucketState", 0)
            if state == 10:
                log.info("ArmBucketState=10 (dump done)")
                return True
            rospy.sleep(0.2)
        state = rospy.get_param(f"{self.BT_NS}/ArmBucketState", -1)
        log.error("Timeout waiting ArmBucketState=10 (got %d)", state)
        return False

    def wait_seconds(self, sec: float, label=""):
        """等待指定秒数（可被 ctrlCmd=6 中断）"""
        log.info("Waiting %.1fs%s...", sec, f" [{label}]" if label else "")
        deadline = time.time() + sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self._ctrlcmd6_sent.is_set():
                log.info("ctrlCmd=6 detected — interrupting wait")
                return False
            rospy.sleep(0.1)
        return True


# ----------------------- 测试场景 --------------------------
def test_ctrlcmd6_during_navigation(tester: CtrlCmd6Tester) -> bool:
    """
    TEST 1: 任务执行中途下发 ctrlCmd=6
    流程：IDLE → 发任务 → 到达铲料点 → 到达卸料点(中途发ctrl6) → 等待结束
    """
    log.info("")
    log.info("=" * 60)
    log.info("TEST 1: ctrlCmd=6 during navigation")
    log.info("=" * 60)

    tester.nav.reset_pos()
    tester._ctrlcmd6_sent.clear()

    # 1. 等待 IDLE
    if not tester.wait_bt_state("Idle", timeout=20):
        return False

    # 2. 发任务
    if not tester.send_task(task_id=1, bin_id=1, hopper_id=1):
        return False
    rospy.sleep(1.0)

    # 3. 等待到达铲料点
    if not tester.wait_nav_arrived("SCOOP", timeout=60):
        log.warning("Scoop arrival timeout — may continue anyway")
    rospy.sleep(0.5)

    # 4. 等待卸料完成（ArmBucketState=10）
    if not tester.wait_abs_done(timeout=60):
        log.warning("ABS timeout — continuing test")

    log.info("TEST1: Sending ctrlCmd=6 NOW (during DUMP phase)...")
    tester.send_ctrl_cmd(6, task_id=1)

    # 5. 等待 workstate 变为 7（Ending）
    ws7_ok = tester.wait_workstate(7, timeout=15)
    log.info("TEST1 workstate=7 result: %s", ws7_ok)

    # 6. 等待返回 IDLE
    idle_ok = tester.wait_bt_state("Idle", timeout=30)
    log.info("TEST1 IDLE result: %s", idle_ok)

    # 7. 确认最终状态
    final_ws = rospy.get_param(f"{tester.BT_NS}/workstate", -1)
    log.info("TEST1 final workstate: %d (%s)",
             final_ws, WORKSTATE_NAMES.get(final_ws, "?"))

    passed = ws7_ok and (idle_ok or final_ws == 7)
    log.info("TEST 1 RESULT: %s", "PASS" if passed else "FAIL")
    return passed


def test_ctrlcmd6_after_task_done(tester: CtrlCmd6Tester) -> bool:
    """
    TEST 2: 当前任务结束后下发 ctrlCmd=6
    流程：IDLE → 发任务 → 完成全部铲+卸 → 等待一段idle → 发ctrl6 → 等待结束
    """
    log.info("")
    log.info("=" * 60)
    log.info("TEST 2: ctrlCmd=6 after task completes")
    log.info("=" * 60)

    tester.nav.reset_pos()
    tester._ctrlcmd6_sent.clear()

    # 1. 等待 IDLE
    if not tester.wait_bt_state("Idle", timeout=20):
        return False

    # 2. 发任务
    if not tester.send_task(task_id=1, bin_id=1, hopper_id=1):
        return False
    rospy.sleep(1.0)

    # 3. 等待到达铲料点
    if not tester.wait_nav_arrived("SCOOP", timeout=60):
        log.warning("Scoop arrival timeout")
    rospy.sleep(0.5)

    # 4. 等待 ArmBucketState=10（卸料完成）
    if not tester.wait_abs_done(timeout=60):
        log.warning("ABS timeout")

    # 5. 等待到达卸料点
    if not tester.wait_nav_arrived("DUMP", timeout=60):
        log.warning("Dump arrival timeout")

    # 6. 任务已完成，等待一小段 idle 时间再发 ctrl6
    rospy.sleep(2.0)
    log.info("TEST2: Task appears complete, sending ctrlCmd=6 NOW...")
    tester.send_ctrl_cmd(6, task_id=1)

    # 7. 等待 workstate 变为 7
    ws7_ok = tester.wait_workstate(7, timeout=15)
    log.info("TEST2 workstate=7 result: %s", ws7_ok)

    # 8. 等待返回 IDLE
    idle_ok = tester.wait_bt_state("Idle", timeout=30)
    log.info("TEST2 IDLE result: %s", idle_ok)

    # 9. 确认最终状态
    final_ws = rospy.get_param(f"{tester.BT_NS}/workstate", -1)
    log.info("TEST2 final workstate: %d (%s)",
             final_ws, WORKSTATE_NAMES.get(final_ws, "?"))

    passed = ws7_ok and (idle_ok or final_ws == 7)
    log.info("TEST 2 RESULT: %s", "PASS" if passed else "FAIL")
    return passed


# ----------------------- 主入口 --------------------------
def main():
    rospy.init_node("test_ctrlcmd6_end_task")

    run_test1 = len(sys.argv) < 2 or "1" in sys.argv[1]
    run_test2 = len(sys.argv) < 2 or "2" in sys.argv[1]

    log.info("")
    log.info("╔══════════════════════════════════════════════════════╗")
    log.info("║       ctrlCmd=6 End Task Test Suite                  ║")
    log.info("╠══════════════════════════════════════════════════════╣")
    log.info("║  TEST 1: ctrlCmd=6 during navigation (中途结束)       ║")
    log.info("║  TEST 2: ctrlCmd=6 after task done (完成后结束)       ║")
    log.info("╚══════════════════════════════════════════════════════╝")
    log.info("Log file: %s", log_file)
    log.info("")

    nav = NavMock()
    tester = CtrlCmd6Tester(nav)

    # 等待 BT 节点完全启动
    log.info("Waiting 3s for BT node to fully initialize...")
    rospy.sleep(3.0)

    results = {}

    if run_test1:
        results[1] = test_ctrlcmd6_during_navigation(tester)
        rospy.sleep(3.0)  # 重置之间等待

    if run_test2:
        results[2] = test_ctrlcmd6_after_task_done(tester)

    # 汇总
    log.info("")
    log.info("=" * 60)
    log.info("TEST SUMMARY")
    log.info("=" * 60)
    for k, v in results.items():
        log.info("  TEST %d: %s", k, "PASS" if v else "FAIL")
    all_pass = all(results.values())
    log.info("Overall: %s", "ALL PASS" if all_pass else "SOME FAILED")
    log.info("Full log: %s", log_file)
    log.info("=" * 60)

    if not all_pass:
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
