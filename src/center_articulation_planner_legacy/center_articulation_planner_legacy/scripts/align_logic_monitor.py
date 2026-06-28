#!/usr/bin/env python3
"""
Terminal monitor for the AligningInPlace decision logic.

Mirrors planner_ros.cpp::maybeEnterAligningInPlace():
  - Needs current segmented path start, active pose, and /way_point_shan_tui.
  - Triggers when distance to segment start > align_pos_thresh_m
    AND |lookahead heading| > align_heading_thresh_deg.

The script only prints diagnostics; it does not send any commands.
Override topics or thresholds via CLI args if your system uses different names.
"""

import argparse
import math
import time
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


def normalize_deg(deg: float) -> float:
    """Wrap degrees to [-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


class Monitor:
    def __init__(self, args):
        self.rate_hz = args.rate

        # Topics (can be overridden by params then CLI)
        self.segmented_path_topic = args.segmented_path_topic or rospy.get_param(
            "~segmented_path_topic", "/segmented_path"
        )
        self.front_pose_topic = args.front_pose_topic or rospy.get_param(
            "~front_pose_topic", "/Odometry_qianlun"
        )
        self.rear_pose_topic = args.rear_pose_topic or rospy.get_param(
            "~rear_pose_topic", "/Odometry_houlun"
        )
        self.waypoint_topic = args.waypoint_topic or rospy.get_param(
            "~way_point_shan_tui_topic", "/way_point_shan_tui"
        )
        self.is_forward_topic = args.is_forward_topic or rospy.get_param(
            "~is_forward_topic", "/is_forward"
        )

        # Thresholds (fallback to planner defaults)
        self.align_pos_thresh_m = args.align_pos_thresh_m
        self.align_heading_thresh_deg = args.align_heading_thresh_deg

        # Align service tunables (mirror align_in_place_server defaults)
        self.tol_deg = rospy.get_param("~tol_deg", 1.0)
        self.hold_count = int(rospy.get_param("~hold_count", 3))
        self.timeout_sec = rospy.get_param("~timeout_sec", 6.0)
        self.stale_sec = rospy.get_param("~stale_sec", 0.5)
        self.saturation_deg = rospy.get_param("~saturation_deg", 30.0)
        self.k_p = rospy.get_param("~k_p", 0.015)
        self.max_ang_speed = rospy.get_param("~max_ang_speed", 0.25)

        # Message caches
        self.path_msg: Optional[Path] = None
        self.front_pose: Optional[PoseStamped] = None
        self.rear_pose: Optional[PoseStamped] = None
        self.waypoint: Optional[PoseStamped] = None
        self.is_forward: Optional[bool] = None

        # Subscribers
        rospy.Subscriber(self.segmented_path_topic, Path, self._on_path, queue_size=1)
        rospy.Subscriber(self.front_pose_topic, PoseStamped, self._on_front_pose, queue_size=1)
        rospy.Subscriber(self.rear_pose_topic, PoseStamped, self._on_rear_pose, queue_size=1)
        rospy.Subscriber(self.waypoint_topic, PoseStamped, self._on_waypoint, queue_size=1)
        rospy.Subscriber(self.is_forward_topic, Bool, self._on_is_forward, queue_size=1)

    # Callbacks
    def _on_path(self, msg: Path):
        self.path_msg = msg

    def _on_front_pose(self, msg: PoseStamped):
        self.front_pose = msg

    def _on_rear_pose(self, msg: PoseStamped):
        self.rear_pose = msg

    def _on_waypoint(self, msg: PoseStamped):
        self.waypoint = msg

    def _on_is_forward(self, msg: Bool):
        self.is_forward = bool(msg.data)

    # Helpers
    def _choose_pose(self) -> Optional[PoseStamped]:
        """Pick active pose based on last is_forward; fall back to available."""
        if self.is_forward is None:
            return self.front_pose or self.rear_pose
        return self.front_pose if self.is_forward else self.rear_pose

    def _path_start(self) -> Optional[PoseStamped]:
        if self.path_msg and self.path_msg.poses:
            return self.path_msg.poses[0]
        return None

    def _heading_deg_from_waypoint(self) -> Optional[float]:
        if not self.waypoint:
            return None
        q = self.waypoint.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return normalize_deg(math.degrees(yaw))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._print_status()
            rate.sleep()

    def _print_status(self):
        ts = time.strftime("%H:%M:%S")

        path_ok = self.path_msg is not None and bool(self.path_msg.poses)
        start_pose = self._path_start()
        waypoint_heading = self._heading_deg_from_waypoint()
        pose = self._choose_pose()

        # Dist / heading computation
        dist = None
        if start_pose and pose:
            dx = start_pose.pose.position.x - pose.pose.position.x
            dy = start_pose.pose.position.y - pose.pose.position.y
            dist = math.hypot(dx, dy)

        will_align = False
        reasons = []

        if not path_ok:
            reasons.append("path missing")
        if pose is None:
            reasons.append("pose missing")
        if waypoint_heading is None:
            reasons.append("waypoint missing")

        if dist is not None and waypoint_heading is not None:
            dist_ok = dist > self.align_pos_thresh_m
            heading_ok = abs(waypoint_heading) > self.align_heading_thresh_deg
            will_align = dist_ok and heading_ok
        else:
            dist_ok = False
            heading_ok = False

        pose_src = "front" if (self.is_forward is None or self.is_forward) else "rear"
        line1 = (
            f"[{ts}] path={'ok' if path_ok else 'miss'} "
            f"waypoint={'ok' if waypoint_heading is not None else 'miss'} "
            f"is_forward={int(self.is_forward) if self.is_forward is not None else '?'}({pose_src})"
        )
        print(line1)

        dist_txt = "n/a" if dist is None else f"{dist:.2f}m"
        heading_txt = "n/a" if waypoint_heading is None else f"{waypoint_heading:.1f}°"
        target_angle_txt = "n/a" if waypoint_heading is None else f"{waypoint_heading:.1f}°"
        dist_cmp = "n/a" if dist is None else ("yes" if dist > self.align_pos_thresh_m else "no")
        heading_cmp = (
            "n/a" if waypoint_heading is None else ("yes" if abs(waypoint_heading) > self.align_heading_thresh_deg else "no")
        )

        reason_txt = " ".join(reasons) if reasons else ""
        align_txt = "YES" if will_align else f"NO{(' (' + reason_txt + ')') if reason_txt else ''}"
        print(
            f"  dist={dist_txt} > {self.align_pos_thresh_m:.2f}? {dist_cmp} | "
            f"heading={heading_txt} > {self.align_heading_thresh_deg:.1f}°? {heading_cmp} "
            f"-> AligningInPlace: {align_txt}"
        )
        print(f"  target_angle_deg(planner align_target_angle_deg_)= {target_angle_txt}")

        print(
            f"  align_srv: tol={self.tol_deg:.1f}° hold={self.hold_count} "
            f"timeout={self.timeout_sec:.1f}s stale={self.stale_sec:.1f}s "
            f"sat={self.saturation_deg:.0f}° k_p={self.k_p:.3f} max_w={self.max_ang_speed:.2f}rad/s"
        )
        print("")  # blank line for readability


def main():
    parser = argparse.ArgumentParser(description="Monitor AligningInPlace trigger logic.")
    parser.add_argument("--rate", type=float, default=1.0, help="Print rate in Hz.")
    parser.add_argument("--segmented-path-topic", dest="segmented_path_topic", default=None)
    parser.add_argument("--front-pose-topic", dest="front_pose_topic", default=None)
    parser.add_argument("--rear-pose-topic", dest="rear_pose_topic", default=None)
    parser.add_argument("--waypoint-topic", dest="waypoint_topic", default=None)
    parser.add_argument("--is-forward-topic", dest="is_forward_topic", default=None)
    parser.add_argument("--align-pos-thresh-m", dest="align_pos_thresh_m", type=float, default=1.0)
    parser.add_argument("--align-heading-thresh-deg", dest="align_heading_thresh_deg", type=float, default=30.0)
    args, _ = parser.parse_known_args()

    rospy.init_node("align_logic_monitor", anonymous=True)
    monitor = Monitor(args)
    rospy.loginfo("align_logic_monitor started. Printing at %.2f Hz", args.rate)
    monitor.spin()


if __name__ == "__main__":
    main()
