#!/usr/bin/env python3
"""
Align-in-place service (heading error driven).

Computes target_angle_deg from /way_point_shan_tui (PoseStamped) heading,
and drives that error toward 0 by publishing angular velocity on /cmd_vel
with simple P control, timeout, staleness, and saturation guards.

The external /articulation_angle is still monitored; if it reaches the
steering limit (+/-saturation_deg), the service exits early with success.

rosservice call /align_in_place "{use_rear_pose_source: false, target_angle: 15.0}"
rosservice call /align_in_place "{use_rear_pose_source: true, target_angle: -10.0}"

params:
  ~input_is_radians: false  # if true, /articulation_angle is interpreted as radians
  ~articulation_positive_is_left: true  # if false, /articulation_angle is right-positive
  ~waypoint_topic: /way_point_shan_tui
"""

import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

from center_articulation_planner_legacy.srv import AlignInPlace, AlignInPlaceResponse

STOP_PUBLISH_COUNT = 5


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_deg(deg):
    """Wrap degrees to [-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


class AlignInPlaceServer:
    def __init__(self):
        rospy.init_node("align_in_place_server", anonymous=True)

        # Tunables (all via rosparam to avoid changing srv)
        self.tol_deg = rospy.get_param("~tol_deg", 1.0)
        self.hold_count = int(rospy.get_param("~hold_count", 3))
        self.k_p = rospy.get_param("~k_p", 0.015)  # rad/s per degree
        self.max_ang_speed = rospy.get_param("~max_ang_speed", 0.25)  # rad/s
        # self.max_ang_speed = rospy.get_param("~max_ang_speed", 0.127)  # rad/s

        self.timeout_sec = rospy.get_param("~timeout_sec", 6.0)
        self.stale_sec = rospy.get_param("~stale_sec", 0.5)  # 数据过期保护时间
        self.pub_rate_hz = rospy.get_param("~pub_rate_hz", 10.0)
        self.saturation_deg = rospy.get_param("~saturation_deg", 30.0)
        self.cmd_vel_frame = rospy.get_param("~cmd_vel_frame", "base_link")
        self.input_is_radians = rospy.get_param("~input_is_radians", True)
        #self.input_is_radians = rospy.get_param("~input_is_radians", False)
        self.articulation_positive_is_left = rospy.get_param("~articulation_positive_is_left", False)
        # self.articulation_positive_is_left = rospy.get_param("~articulation_positive_is_left", True)

        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/way_point_shan_tui")

        self.current_angle_deg = None
        self.last_angle_stamp = rospy.Time(0)
        self.target_angle_deg = None
        self.last_target_stamp = rospy.Time(0)

        self._active = False
        self._lock = threading.Lock()

        self.cmd_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)

        #rospy.Subscriber("/vcu_turn_angle", Float32, self._angle_callback)
        rospy.Subscriber("/articulation_angle", Float32, self._angle_callback)

        rospy.Subscriber(self.waypoint_topic, PoseStamped, self._waypoint_callback)

        self.service = rospy.Service("align_in_place", AlignInPlace, self._handle_align)

        rospy.loginfo("AlignInPlaceServer initialized.")
        rospy.loginfo(
            "params: tol=%.2f deg, hold=%d, k_p=%.3f, max_w=%.2f rad/s, timeout=%.1f s, stale=%.1f s, sat=%.1f deg, rate=%.1f Hz, input_is_radians=%s, articulation_positive_is_left=%s",
            self.tol_deg,
            self.hold_count,
            self.k_p,
            self.max_ang_speed,
            self.timeout_sec,
            self.stale_sec,
            self.saturation_deg,
            self.pub_rate_hz,
            str(self.input_is_radians),
            str(self.articulation_positive_is_left),
        )

    def _angle_callback(self, msg):
        angle_deg = msg.data
        if self.input_is_radians:
            angle_deg = math.degrees(angle_deg)
        if not self.articulation_positive_is_left:
            angle_deg = -angle_deg
        self.current_angle_deg = angle_deg
        self.last_angle_stamp = rospy.Time.now()

    def _waypoint_callback(self, msg):
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.target_angle_deg = normalize_deg(math.degrees(yaw))
        self.last_target_stamp = rospy.Time.now()

    def _publish_cmd(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cmd_vel_frame
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def _stop_motion(self):
        for _ in range(STOP_PUBLISH_COUNT):
            self._publish_cmd(0.0)
            rospy.sleep(0.02)

    def _wait_for_inputs(self, timeout_sec):
        start = time.time()
        while not rospy.is_shutdown():
            if self.current_angle_deg is not None and self.target_angle_deg is not None:
                return True
            if time.time() - start > timeout_sec:
                return False
            rospy.sleep(0.01)
        return False

    def _handle_align(self, req):
        def finish(success, message, angle):
            rospy.loginfo(
                "align_in_place finished: success=%s, msg=%s, angle=%.3f",
                str(success),
                message,
                angle,
            )
            return AlignInPlaceResponse(success, message, angle)

        with self._lock:
            if self._active:
                return finish(False, "align_in_place is already running", 0.0)
            self._active = True

        try:
            rospy.loginfo(
                "align_in_place request: use_rear_pose_source=%s, target_angle=%.3f (ignored)",
                str(req.use_rear_pose_source),
                req.target_angle,
            )
            if not self._wait_for_inputs(0.5):
                missing = []
                if self.target_angle_deg is None:
                    missing.append("way_point_shan_tui")
                if self.current_angle_deg is None:
                    missing.append("articulation_angle")
                missing_txt = ", ".join(missing) if missing else "inputs"
                return finish(False, f"no {missing_txt} received", 0.0)

            start_time = rospy.Time.now()
            last_log_time = rospy.Time(0)
            rate = rospy.Rate(self.pub_rate_hz)
            success_hits = 0

            while not rospy.is_shutdown():
                now = rospy.Time.now()
                if (now - start_time).to_sec() > self.timeout_sec:
                    self._stop_motion()
                    return finish(False, "timeout", self.current_angle_deg or 0.0)

                if (now - self.last_target_stamp).to_sec() > self.stale_sec:
                    self._stop_motion()
                    return finish(False, "way_point_shan_tui stale", self.current_angle_deg or 0.0)

                if (now - self.last_angle_stamp).to_sec() > self.stale_sec:
                    self._stop_motion()
                    return finish(False, "articulation_angle stale", self.current_angle_deg or 0.0)

                target = self.target_angle_deg
                if target is None:
                    self._stop_motion()
                    return finish(False, "way_point_shan_tui missing", self.current_angle_deg or 0.0)

                angle = self.current_angle_deg
                if angle is None:
                    self._stop_motion()
                    return finish(False, "articulation_angle missing", 0.0)

                if abs(angle) >= self.saturation_deg:
                    self._stop_motion()
                    return finish(True, "articulation limit reached", angle)

                err = -target if req.use_rear_pose_source else target
                if abs(err) <= self.tol_deg:
                    success_hits += 1
                    if success_hits >= self.hold_count:
                        self._stop_motion()
                        return finish(True, "aligned", angle)
                else:
                    success_hits = 0

                ang_z = self.k_p * err
                ang_z = clamp(ang_z, -self.max_ang_speed, self.max_ang_speed)
                if (now - last_log_time).to_sec() >= 0.2:
                    rospy.loginfo(
                        "align_in_place: target=%.3f deg, err=%.3f deg, cmd_w=%.3f rad/s",
                        target,
                        err,
                        ang_z,
                    )
                    last_log_time = now
                self._publish_cmd(ang_z)
                rate.sleep()

            self._stop_motion()
            return finish(False, "node shutdown", self.current_angle_deg or 0.0)
        finally:
            with self._lock:
                self._active = False


if __name__ == "__main__":
    server = AlignInPlaceServer()
    rospy.spin()
