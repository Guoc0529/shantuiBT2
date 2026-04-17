#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bt_flow_smoke_test.py
================================

用于在纯ROS环境中自动化测试自主装载机器人行为树（BT）状态机的脚本。
功能：
    1. 启动一个假的 /move_base ActionServer，自动发送反馈并回传到达结果
    2. 启动 /spadingPose 服务，第一次返回 (0,0)，第二次返回真实铲料点
    3. 监控参数 ArmBucketState / PointAccumulation，模拟各阶段完成/失败
    4. 自动下发若干个任务，验证完整铲料-卸料流程
    5. 可选注入抬臂失败，用于测试状态机的异常处理
"""

import math
import threading

import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from std_msgs.msg import Bool

from autonomous_loader_msgs.msg import TaskCommand
from autonomous_loader_msgs.srv import spadingPose, spadingPoseResponse


class MoveBaseMockServer:
    """简易的 move_base ActionServer，用于模拟导航反馈与结果。"""

    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "/move_base", MoveBaseAction, execute_cb=self._execute_cb, auto_start=False
        )
        self._speed = rospy.get_param("~mock_nav_speed", 0.8)  # m/s
        self._feedback_rate = rospy.Rate(10)
        self._current_x = 0.0
        self._current_y = 0.0
        self._server.start()
        rospy.loginfo("[MockNav] /move_base action server started (speed=%.2f m/s)", self._speed)

    def _execute_cb(self, goal):
        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        dx = goal_x - self._current_x
        dy = goal_y - self._current_y
        distance = max(math.hypot(dx, dy), 0.1)
        duration = distance / self._speed
        start_time = rospy.Time.now()

        rospy.loginfo("[MockNav] Received goal (%.2f, %.2f), dist=%.2f m", goal_x, goal_y, distance)

        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                rospy.logwarn("[MockNav] Goal preempted")
                self._server.set_preempted()
                return

            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(elapsed / duration, 1.0)
            current_x = self._current_x + dx * progress
            current_y = self._current_y + dy * progress

            feedback = MoveBaseFeedback()
            feedback.base_position.header.stamp = rospy.Time.now()
            feedback.base_position.header.frame_id = "map"
            feedback.base_position.pose.position.x = current_x
            feedback.base_position.pose.position.y = current_y
            self._server.publish_feedback(feedback)

            if progress >= 1.0:
                break

            self._feedback_rate.sleep()

        self._current_x = goal_x
        self._current_y = goal_y
        rospy.loginfo("[MockNav] Goal reached")
        self._server.set_succeeded(MoveBaseResult(), "goal reached")


class ScoopPointProvider:
    """模拟 /spadingPose 服务：第一次返回(0,0)，第二次返回真实铲料点。"""

    BIN_POINTS = {
        1: (10.0, 10.0),
        2: (15.0, 12.0),
        3: (20.0, 8.0),
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._return_zero_once = True
        self._active_bin = 1
        self._srv = rospy.Service("/spadingPose", spadingPose, self._handle_request)
        rospy.loginfo("[MockScoopPoint] Service /spadingPose ready")

    def start_cycle(self, bin_id: int):
        with self._lock:
            self._return_zero_once = True
            self._active_bin = bin_id

    def _handle_request(self, req):
        with self._lock:
            pose = Pose()
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            if self._return_zero_once:
                self._return_zero_once = False
                pose.position.x = 0.0
                pose.position.y = 0.0
                rospy.loginfo("[MockScoopPoint] Return (0, 0) for pile %d", req.pileID)
            else:
                point = self.BIN_POINTS.get(req.pileID, (12.0, 12.0))
                pose.position.x, pose.position.y = point
                rospy.loginfo("[MockScoopPoint] Return (%.2f, %.2f) for pile %d",
                              pose.position.x, pose.position.y, req.pileID)

            return spadingPoseResponse(pose)


class ArmBucketParamSimulator:
    """监控/写入 ArmBucketState 与 PointAccumulation 参数，模拟装置完成。"""

    def __init__(self):
        bt_ns = rospy.get_param("~bt_ns", "/autonomous_loader_bt_node").rstrip("/")
        self._arm_param = f"{bt_ns}/ArmBucketState"
        self._point_param = f"{bt_ns}/PointAccumulation"
        self._arm_delay = rospy.get_param(
            "~arm_stage_delay",
            {2: 1.0, 3: 1.0, 4: 1.5, 7: 2.5, 6: 1.0, 5: 1.5, 1: 1.0},
        )
        self._point_delay = rospy.get_param("~point_accum_delay", 1.0)
        self._inject_failure_cycle = rospy.get_param("~inject_raise_arm_failure_after", -1)
        self._inject_scoop_failure_cycle = rospy.get_param("~inject_scoop_failure_after", -1)
        self._failure_used = False
        self._scoop_failure_used = False

        self._last_arm_value = None
        self._last_point_value = None
        self._completed_cycles = 0

        self.on_cycle_finished = None
        self.on_failure = None

        self._timer = rospy.Timer(rospy.Duration(0.1), self._poll)

    def _poll(self, _event):
        self._check_point_param()
        self._check_arm_param()

    def _check_point_param(self):
        try:
            value = rospy.get_param(self._point_param)
        except KeyError:
            return

        if value == self._last_point_value:
            return
        self._last_point_value = value

        if value == 1:
            rospy.loginfo("[ParamSim] PointAccumulation=1, will set to 10 after %.1fs", self._point_delay)
            threading.Timer(self._point_delay, lambda: rospy.set_param(self._point_param, 10)).start()

    def _check_arm_param(self):
        try:
            value = rospy.get_param(self._arm_param)
        except KeyError:
            return

        if value == self._last_arm_value:
            return
        self._last_arm_value = value

        if value in self._arm_delay:
            stage = value
            rospy.loginfo("[ParamSim] Detected ArmBucketState=%d", stage)

            if (self._inject_failure_cycle > 0 and
                    stage == 4 and
                    not self._failure_used and
                    self._completed_cycles >= self._inject_failure_cycle):
                self._failure_used = True
                rospy.logwarn("[ParamSim] Injecting failure for ArmBucketState=4 -> -4")
                threading.Timer(0.5, lambda: rospy.set_param(self._arm_param, -4)).start()
                if self.on_failure:
                    self.on_failure(-4)
                return

            if (self._inject_scoop_failure_cycle > 0 and
                    stage == 7 and
                    not self._scoop_failure_used and
                    self._completed_cycles >= self._inject_scoop_failure_cycle):
                self._scoop_failure_used = True
                rospy.logwarn("[ParamSim] Injecting failure for ArmBucketState=7 -> -7")
                threading.Timer(0.5, lambda: rospy.set_param(self._arm_param, -7)).start()
                if self.on_failure:
                    self.on_failure(-7)
                return

            delay = self._arm_delay.get(stage, 1.0)
            threading.Timer(delay, lambda: self._finish_stage(stage)).start()

    def _finish_stage(self, stage):
        rospy.set_param(self._arm_param, 10)
        rospy.loginfo("[ParamSim] Mark ArmBucketState stage %d as completed (10)", stage)

        if stage in (1, 6):
            self._completed_cycles += 1
            if self.on_cycle_finished:
                self.on_cycle_finished(self._completed_cycles)


class StateMachineTester:
    """协调各个模拟模块，自动执行多次任务循环。"""

    def __init__(self):
        bt_ns = rospy.get_param("~bt_ns", "/autonomous_loader_bt_node").rstrip("/")
        start_topic = f"{bt_ns}/scheduler/start_task"
        end_topic = f"{bt_ns}/scheduler/end_task"

        self._task_pub = rospy.Publisher(start_topic, TaskCommand, queue_size=10, latch=True)
        self._end_pub = rospy.Publisher(end_topic, Bool, queue_size=1)

        self._total_cycles = rospy.get_param("~cycles", 2)
        self._current_cycle = 0
        self._bin_sequence = rospy.get_param("~bin_sequence", [1, 2, 3])
        self._hopper_sequence = rospy.get_param("~hopper_sequence", [1, 1, 2])

        self._move_base_mock = MoveBaseMockServer()
        self._scoop_provider = ScoopPointProvider()
        self._param_sim = ArmBucketParamSimulator()
        self._param_sim.on_cycle_finished = self._handle_cycle_complete
        self._param_sim.on_failure = self._handle_failure

        rospy.sleep(1.0)  # 等待发布者/服务初始化
        self._schedule_next_task()

    def _schedule_next_task(self, delay: float = 1.0):
        rospy.Timer(rospy.Duration(delay), lambda event: self._send_task(), oneshot=True)

    def _send_task(self):
        if self._current_cycle >= self._total_cycles:
            rospy.loginfo("[Tester] 所有任务执行完毕")
            self._send_end_task()
            rospy.Timer(rospy.Duration(3.0), lambda e: rospy.signal_shutdown("Test finished"), oneshot=True)
            return

        task = TaskCommand()
        task.task_id = 100 + self._current_cycle + 1
        task.bin_id = self._bin_sequence[self._current_cycle % len(self._bin_sequence)]
        task.hopper_id = self._hopper_sequence[self._current_cycle % len(self._hopper_sequence)]
        task.task_type = "scoop"

        self._scoop_provider.start_cycle(task.bin_id)
        self._task_pub.publish(task)
        rospy.loginfo("[Tester] 发送任务 #%d (bin=%d, hopper=%d)", task.task_id, task.bin_id, task.hopper_id)

    def _handle_cycle_complete(self, finished_cycles: int):
        self._current_cycle = finished_cycles
        rospy.loginfo("[Tester] 第 %d 个任务循环完成", finished_cycles)
        self._schedule_next_task(delay=2.0)

    def _handle_failure(self, code: int):
        rospy.logwarn("[Tester] 检测到 ArmBucketState 失败码：%d，发送结束任务指令", code)
        self._send_end_task()
        rospy.Timer(
            rospy.Duration(3.0),
            lambda _evt: rospy.signal_shutdown("Failure injected"),
            oneshot=True,
        )

    def _send_end_task(self):
        msg = Bool()
        msg.data = True
        self._end_pub.publish(msg)
        rospy.loginfo("[Tester] 已发送结束任务指令")


def main():
    rospy.init_node("bt_flow_smoke_test")
    rospy.loginfo("行为树冒烟测试脚本启动 (cycles=%d)", rospy.get_param("~cycles", 2))
    StateMachineTester()
    rospy.spin()


if __name__ == "__main__":
    main()

