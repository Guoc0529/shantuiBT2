#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
manual_test_simulator.py
========================

A minimal simulator for manually testing the Behavior Tree.
This script ONLY provides the necessary background services that cannot be easily
run from the command line:

1.  **Navigation Simulator**: A mock /navigate Action Server (NavigateAction) that automatically
    accepts goals, simulates travel (publishes distance_to_goal feedback), and returns success.

2.  **Scoop Point Service**: A mock /spadingPose Service that allows you to
    interactively toggle its response between (0,0) and a real point.

This script does NOT simulate the scheduler or any work devices. You must use
`rosservice call` to send tasks and `rosparam set` to update `ArmBucketState`
and `PointAccumulation` manually from another terminal.

Key Controls:
-   `t`: Toggle the /spadingPose response between Zero Mode and Real Point Mode.
-   `q`: Quit the script.
"""

import math
import threading
import sys
import select
import termios
import tty

import actionlib
import rospy
from geometry_msgs.msg import Pose
from autonomous_loader_msgs.msg import NavigateAction, NavigateFeedback, NavigateResult
from autonomous_loader_msgs.srv import spadingPose, spadingPoseResponse

STATUS_LABELS = {
    0: "铲料",
    1: "卸料",
    2: "其他"
}


def format_status(code):
    return STATUS_LABELS.get(code, f"未知({code})")

class NavigateMockServer:
    """Minimal mock /navigate NavigateAction Server."""
    def __init__(self):
        self._server = actionlib.SimpleActionServer("/navigate", NavigateAction, execute_cb=self._execute_cb, auto_start=False)
        self._speed = rospy.get_param("~mock_nav_speed", 2.0)  # m/s
        self._feedback_rate = rospy.Rate(10)
        self._current_x = 0.0
        self._current_y = 0.0
        self._server.start()
        rospy.loginfo("[MockNav] /navigate action server is ready (speed=%.2f m/s)", self._speed)

    def _execute_cb(self, goal):
        goal_x = goal.end_pose.pose.position.x
        goal_y = goal.end_pose.pose.position.y
        distance = max(math.hypot(goal_x - self._current_x, goal_y - self._current_y), 0.1)
        duration = distance / self._speed
        last_status_code = getattr(goal, 'last_status', 2)
        current_status_code = getattr(goal, 'current_status', 2)
        rospy.loginfo("[MockNav] Received goal (%.2f, %.2f), status=%s -> %s. Simulating %.2fs travel.",
                      goal_x, goal_y,
                      format_status(last_status_code),
                      format_status(current_status_code),
                      duration)
        # Publish feedback decreasing distance_to_goal to 0
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                rospy.logwarn("[MockNav] Goal preempted")
                self._server.set_preempted()
                return
            elapsed = (rospy.Time.now() - start).to_sec()
            progress = min(elapsed / duration, 1.0)
            dist_remaining = max(distance * (1.0 - progress), 0.0)
            fb = NavigateFeedback()
            fb.distance_to_goal = dist_remaining
            self._server.publish_feedback(fb)
            if progress >= 1.0:
                break
            self._feedback_rate.sleep()
        self._current_x = goal_x
        self._current_y = goal_y
        rospy.loginfo("[MockNav] Goal reached at (%.2f, %.2f)", self._current_x, self._current_y)
        res = NavigateResult()
        res.plan_succeeded = True
        res.success = True
        self._server.set_succeeded(res, "goal reached")

class MockSpadingService:
    """Interactive mock /spadingPose service."""
    def __init__(self):
        self.lock = threading.Lock()
        self.return_zero_mode = True
        self.real_point = (10.0, 10.0)
        self.service = rospy.Service('/spadingPose', spadingPose, self.handle_request)
        self.print_status()
        rospy.loginfo("[MockSpadingService] /spadingPose service is ready.")

    def handle_request(self, req):
        with self.lock:
            rospy.loginfo(f"[MockSpadingService] Received request for pileID: {req.pileID}")
            resp = spadingPoseResponse()
            resp.pose_output = Pose()
            if self.return_zero_mode:
                rospy.loginfo("[MockSpadingService] Responding in Zero Mode: (0.0, 0.0)")
            else:
                resp.pose_output.position.x, resp.pose_output.position.y = self.real_point
                rospy.loginfo(f"[MockSpadingService] Responding in Real Point Mode: {self.real_point}")
            return resp

    def toggle_mode(self):
        with self.lock:
            self.return_zero_mode = not self.return_zero_mode
        self.print_status()

    def print_status(self):
        mode = "Zero Mode (returns 0,0)" if self.return_zero_mode else f"Real Point Mode (returns {self.real_point})"
        rospy.loginfo(f"\n\033[93mCurrent Scoop Point Mode: {mode}. Press 't' to toggle, 'q' to quit.\033[0m")

def main():
    rospy.init_node('manual_test_simulator')
    _ = NavigateMockServer()
    spading_service = MockSpadingService()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.5)[0]:
                key = sys.stdin.read(1)
                if key == 't':
                    spading_service.toggle_mode()
                elif key == 'q':
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
