#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mock_spading_service_seq.py
===========================

按任务序列自动切换 /spadingPose 返回值的服务模拟器。

用途:
- 配合行为树节点与 interactive_tester.py，一次次按 's' 下发任务时，
  本服务会根据“第几次任务”自动决定返回 (0,0) 还是非零真实点。
- 默认策略：第1个任务返回(0,0)，第2与第3个任务返回非零点；
  超过3个任务后，保持返回非零点（可通过参数配置）。

特性:
- 订阅 /autonomous_loader_bt_node/scheduler/start_task 以统计任务序号。
- 提供 /spadingPose 服务，按照当前任务序号返回点位。
- 可通过参数配置非零点坐标与策略。

参数:
- ~real_point_x (float, 默认 10.0)
- ~real_point_y (float, 默认 10.0)
- ~sequence (string, 默认 "zero,real,real")
    - 以逗号分隔的模式序列：zero=返回(0,0)，real=返回非零点
    - 序列用完后，继续使用最后一个模式

运行:
  终端1: 行为树
    roslaunch autonomous_loader_bt autonomous_loader_bt.launch

  终端2: 调度与导航模拟（按 s 连续下发三次任务即可）
    rosrun autonomous_loader_bt interactive_tester.py

  终端3: 铲料点服务（按任务序列自动切换返回）
    rosrun autonomous_loader_bt mock_spading_service_seq.py _real_point_x:=10.5 _real_point_y:=10.5 _sequence:="zero,real,real"
"""

import threading
import rospy
from geometry_msgs.msg import Pose
from autonomous_loader_msgs.srv import spadingPose, spadingPoseResponse
from autonomous_loader_msgs.msg import TaskCommand

class SequencedSpadingService:
    def __init__(self):
        self.lock = threading.Lock()

        # 配置
        self.real_point = (
            rospy.get_param("~real_point_x", 10.0),
            rospy.get_param("~real_point_y", 10.0),
        )
        seq_str = rospy.get_param("~sequence", "zero,real,real")
        self.sequence = [s.strip().lower() for s in seq_str.split(',') if s.strip()]
        if not self.sequence:
            self.sequence = ["zero", "real", "real"]
        rospy.loginfo("[SeqSpading] sequence=%s, real_point=%s", self.sequence, self.real_point)

        # 状态：当前任务序号（从1开始计数），以及最近一次下发的模式
        self.task_count = 0
        self.current_mode = "zero"

        # 订阅调度开始任务主题，统计任务次数
        bt_ns = rospy.get_param("~bt_ns", "/autonomous_loader_bt_node").rstrip("/")
        start_topic = f"{bt_ns}/scheduler/start_task"
        rospy.Subscriber(start_topic, TaskCommand, self._on_start_task)

        # 提供服务
        self.service = rospy.Service('/spadingPose', spadingPose, self.handle_request)
        rospy.loginfo("[SeqSpading] /spadingPose service is ready. Waiting for tasks on %s", start_topic)

    def _mode_for_index(self, idx: int) -> str:
        if idx <= 0:
            return self.sequence[0]
        if idx <= len(self.sequence):
            return self.sequence[idx-1]
        return self.sequence[-1]

    def _on_start_task(self, msg: TaskCommand):
        with self.lock:
            self.task_count += 1
            self.current_mode = self._mode_for_index(self.task_count)
            rospy.loginfo(
                "[SeqSpading] Received start_task #%d -> mode=%s (bin=%s hopper=%s type=%s)",
                self.task_count, self.current_mode, getattr(msg, 'bin_id', None), getattr(msg, 'hopper_id', None), getattr(msg, 'task_type', None)
            )

    def handle_request(self, req):
        with self.lock:
            mode = self.current_mode
            resp = spadingPoseResponse()
            resp.pose_output = Pose()
            resp.pose_output.orientation.w = 1.0

            if mode == 'zero':
                resp.pose_output.position.x = 0.0
                resp.pose_output.position.y = 0.0
                rospy.loginfo("[SeqSpading] Responding ZERO (0,0) for pileID=%s [task#%d]", getattr(req, 'pileID', None), self.task_count)
            else:
                resp.pose_output.position.x = self.real_point[0]
                resp.pose_output.position.y = self.real_point[1]
                rospy.loginfo("[SeqSpading] Responding REAL %s for pileID=%s [task#%d]", self.real_point, getattr(req, 'pileID', None), self.task_count)

            return resp

def main():
    rospy.init_node('mock_spading_service_seq')
    _ = SequencedSpadingService()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

