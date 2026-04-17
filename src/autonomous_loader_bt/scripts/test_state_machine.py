#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import time
import actionlib
from geometry_msgs.msg import PoseStamped
from autonomous_loader_msgs.msg import NavigateAction, NavigateFeedback, NavigateResult
from std_msgs.msg import Float64
from autonomous_loader_msgs.msg import TaskCommand

class DummyNavigateServer:
    def __init__(self, server_name='navigate'):
        self._as = actionlib.SimpleActionServer(server_name, NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        # optional: publish a Float64 for distance_to_goal (not required by current BT)
        self._dist_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/distance_to_goal', Float64, queue_size=10, latch=True)
        rospy.loginfo("[Test] Dummy Navigate Action Server '%s' started", server_name)

    def execute_cb(self, goal):
        feedback = NavigateFeedback()
        result = NavigateResult()
        # Simulate a 10-second navigation decreasing distance to 0
        start = time.time()
        rate = rospy.Rate(5)
        total = 10.0
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn("[Test] Navigate goal preempted")
                self._as.set_preempted()
                return
            elapsed = time.time() - start
            dist = max(0.0, total - elapsed)
            feedback.distance_to_goal = dist
            feedback.state = "MOVING" if dist > 0.0 else "ARRIVED"
            self._as.publish_feedback(feedback)
            # also publish to the state machine's distance topic
            try:
                from std_msgs.msg import Float64
                self._dist_pub.publish(Float64(data=dist))
            except Exception:
                pass
            if dist <= 0.0:
                result.success = True
                result.message = "Arrived (dummy)"
                result.final_pose = goal.target_pose
                self._as.set_succeeded(result)
                return
            rate.sleep()

class ParamEmulator:
    """
    Emulate modules that consume requests via ArmBucketState / PointAccumulation and write back 10 on success.
    Request codes:
      ArmBucketState: 2(received), 3(flatten), 4(raise), 6(dump), 1(finish lower)
      PointAccumulation: 1(start)
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._last_arm = None
        self._last_acc = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        rospy.loginfo("[Test] ParamEmulator started (ArmBucketState/PointAccumulation)")

    def _schedule_set(self, name, value, delay=2.0):
        def _do():
            rospy.sleep(delay)
            rospy.set_param(name, int(value))
            rospy.loginfo("[Test] ParamEmulator: set %s=%d", name, value)
        threading.Thread(target=_do, daemon=True).start()

    def _loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # ArmBucketState
            try:
                arm = rospy.get_param('ArmBucketState')
            except KeyError:
                arm = None
            # PointAccumulation
            try:
                acc = rospy.get_param('PointAccumulation')
            except KeyError:
                acc = None

            with self._lock:
                if arm is not None and arm != self._last_arm:
                    self._last_arm = arm
                    rospy.loginfo("[Test] ParamEmulator: observed ArmBucketState=%s", str(arm))
                    if arm in (2, 3, 4, 6, 1, 7):
                        # Schedule success=10 after 2s (tweak per case if needed)
                        self._schedule_set('ArmBucketState', 10, delay=2.0)
                if acc is not None and acc != self._last_acc:
                    self._last_acc = acc
                    rospy.loginfo("[Test] ParamEmulator: observed PointAccumulation=%s", str(acc))
                    if acc == 1:
                        # Simulate calculation success after 3s
                        self._schedule_set('PointAccumulation', 10, delay=3.0)
            rate.sleep()

class TaskFeeder:
    def __init__(self, delay_sec=1.0, task_id=1, bin_id=1, hopper_id=1, task_type='scoop'):
        self._pub = rospy.Publisher('/autonomous_loader_bt_node/scheduler/start_task', TaskCommand, queue_size=1, latch=True)
        self._delay = delay_sec
        self._task_id = task_id
        self._bin_id = bin_id
        self._hopper_id = hopper_id
        self._task_type = task_type
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        rospy.sleep(self._delay)
        msg = TaskCommand()
        msg.task_id = self._task_id
        msg.bin_id = self._bin_id
        msg.hopper_id = self._hopper_id
        msg.task_type = self._task_type
        self._pub.publish(msg)
        rospy.loginfo("[Test] Published TaskCommand: id=%d bin=%d hopper=%d type=%s",
                      msg.task_id, msg.bin_id, msg.hopper_id, msg.task_type)

if __name__ == '__main__':
    rospy.init_node('bt_state_machine_test', anonymous=True)
    # Start dummy navigate action server
    DummyNavigateServer()
    # Start parameter emulator for ArmBucketState / PointAccumulation
    ParamEmulator()
    # Feed one task after 1s (adjust bin/hopper as needed)
    TaskFeeder(delay_sec=1.0, task_id=1, bin_id=1, hopper_id=1, task_type='scoop')
    rospy.loginfo('[Test] State machine test started. Ensure autonomous_loader_bt_node is running.')
    rospy.spin()

