#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
manual_intervention_noNav_tester.py
----------------------------------------
This script is a version of the tester that does NOT run a mock navigation server.
It allows you to send tasks and intervention signals to the behavior tree while
using your own, real navigation system.

Functionality:
1. Automatically completes ArmBucketState parameter changes to simulate work device completion.
2. Allows sending tasks, end signals, and manual intervention completion signals via keyboard.
"""
from __future__ import annotations
import random
import sys
import select
import termios
import tty
import time
import logging

import rospy
from std_msgs.msg import Bool, String
from autonomous_loader_msgs.msg import TaskCommand
from autonomous_loader_msgs.srv import spadingPose

# ----------------------- 日志配置 ----------------------------
log_format = "[%(asctime)s.%(msecs)03d] [%(name)-11s] [%(levelname)-7s] %(message)s"
logging.basicConfig(level=logging.INFO, format=log_format, datefmt="%H:%M:%S")

# -------------------- 交互控制器 ------------------------------
class ManualInterventionTester:
    def __init__(self):
        self.log = logging.getLogger("MI-Tester")
        self.bt_log = logging.getLogger("BT-Status")
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
        print("Manual Intervention Tester (No Navigation Mock)")
        print("="*50)
        print("  s : Send new Task (interactive)")
        print("  m : Manual intervention done")
        print("  e : EndTask      h : help      q : quit")
        print("="*50)

# -------------------- 主循环 ------------------------------

def main():
    rospy.init_node("manual_intervention_noNav_tester")
    tester = ManualInterventionTester()

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
                        # 输入 bin/hopper ID
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
