#!/usr/bin/env python3
"""
Visualize the near-goal guard window in RViz.

- Reads goal-guard parameters (distance, angle, forward deadzone) from the ROS
  parameter server; defaults match config/center_articulation_planner_params.yaml.
- Subscribes to the planner's segmented path (or planner goal) plus front/rear
  pose and is_forward flag to mimic the C++ guard logic.
- Publishes a MarkerArray on ~markers showing:
    * Guard radius (line strip circle)
    * Guard wedge edges and rear deadzone line
    * Goal point colored by reachability
    * A status text block with the computed numbers/reason
Run with rosrun/roslaunch and add a Marker (or MarkerArray) display in RViz on
the topic /goal_guard_visualizer/markers.
"""

import math
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _normalize_angle(rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def _path_remaining_length(path: Path, current_xy: Tuple[float, float]) -> float:
    """
    Approximate remaining arc-length from the nearest path point to the end.
    Falls back to straight-line distance if the path is empty.
    """
    if not path.poses:
        return 0.0

    cx, cy = current_xy
    nearest_idx = 0
    nearest_d2 = float("inf")
    for i, pose_stamped in enumerate(path.poses):
        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        if d2 < nearest_d2:
            nearest_d2 = d2
            nearest_idx = i

    remaining = 0.0
    last_x = cx
    last_y = cy
    # Distance from current pose to the nearest path point
    nearest_pose = path.poses[nearest_idx].pose.position
    remaining += math.hypot(nearest_pose.x - cx, nearest_pose.y - cy)

    # Arc length from nearest point to the path end
    last_x = nearest_pose.x
    last_y = nearest_pose.y
    for pose_stamped in path.poses[nearest_idx + 1 :]:
        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        remaining += math.hypot(px - last_x, py - last_y)
        last_x, last_y = px, py

    return remaining


class GoalGuardVisualizer:
    def __init__(self) -> None:
        # Use a fixed node name so the private namespace (and topics) are stable.
        rospy.init_node("goal_guard_visualizer", anonymous=False)

        # Guard parameters (defaults match center_articulation_planner_params.yaml)
        self.guard_distance = rospy.get_param("~goal_guard_distance_m", 3.5)
        self.guard_angle_deg = rospy.get_param("~goal_guard_angle_deg", 100.0)
        self.forward_deadzone = rospy.get_param("~goal_guard_forward_deadzone_m", 0.265)
        self.debug_mode = rospy.get_param("~debug_mode", True)

        # Topics (reuse planner defaults so the node works with the existing launch files)
        self.path_topic = rospy.get_param("~segmented_path_topic", "/segmented_path")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/global_path")
        self.goal_topic = rospy.get_param("~planner_goal_topic", "/planner_goal")
        self.front_pose_topic = rospy.get_param("~front_pose_topic", "/Odometry_qianlun")
        self.rear_pose_topic = rospy.get_param("~rear_pose_topic", "/Odometry_houlun")
        self.is_forward_topic = rospy.get_param("~is_forward_topic", "/is_forward")
        self.use_is_forward_switch = rospy.get_param("~use_is_forward_switch", True)
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 5.0)

        self.front_pose: Optional[PoseStamped] = None
        self.rear_pose: Optional[PoseStamped] = None
        self.current_pose: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None
        self.path: Optional[Path] = None
        self.is_forward: Optional[bool] = None
        self.warned_frame_mismatch = False

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)

        rospy.Subscriber(self.front_pose_topic, PoseStamped, self._front_pose_cb)
        rospy.Subscriber(self.rear_pose_topic, PoseStamped, self._rear_pose_cb)
        rospy.Subscriber(self.is_forward_topic, Bool, self._is_forward_cb)
        rospy.Subscriber(self.path_topic, Path, self._path_cb)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb)
        # Fallback: use global path if segmented path is unavailable
        rospy.Subscriber(self.global_path_topic, Path, self._global_path_cb)

        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 0.1)), self._timer_cb)

        rospy.loginfo(
            "Goal guard viz ready (distance=%.2f m, angle=%.1f deg, forward_deadzone=%.3f m, debug_mode=%s)",
            self.guard_distance,
            self.guard_angle_deg,
            self.forward_deadzone,
            self.debug_mode,
        )

    # --- Subscribers -----------------------------------------------------
    def _front_pose_cb(self, msg: PoseStamped) -> None:
        self.front_pose = msg

    def _rear_pose_cb(self, msg: PoseStamped) -> None:
        self.rear_pose = msg

    def _is_forward_cb(self, msg: Bool) -> None:
        self.is_forward = bool(msg.data)

    def _path_cb(self, msg: Path) -> None:
        self.path = msg
        if msg.poses:
            self.goal = msg.poses[-1]

    def _global_path_cb(self, msg: Path) -> None:
        # Only take it when segmented path is absent
        if self.path is None or not self.path.poses:
            self.path = msg
            if msg.poses:
                self.goal = msg.poses[-1]

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.goal = msg

    # --- Core logic ------------------------------------------------------
    def _pick_pose(self) -> Optional[PoseStamped]:
        """
        Choose front or rear pose to mirror the planner's PoseSource logic:
        use rear pose when is_forward=false; otherwise use front pose.
        """
        if not self.use_is_forward_switch or self.is_forward is None:
            return self.front_pose or self.rear_pose
        return self.front_pose if self.is_forward else self.rear_pose

    def _compute_guard(self) -> Optional[dict]:
        pose = self._pick_pose()
        if pose is None or self.goal is None:
            return None

        # Frame check (warn only once to avoid log spam)
        if (
            self.goal.header.frame_id
            and pose.header.frame_id
            and self.goal.header.frame_id != pose.header.frame_id
            and not self.warned_frame_mismatch
        ):
            rospy.logwarn(
                "Goal frame (%s) != pose frame (%s); visualization uses pose frame.",
                self.goal.header.frame_id,
                pose.header.frame_id,
            )
            self.warned_frame_mismatch = True

        yaw = _normalize_angle(self._yaw_from_pose(pose))
        if self.use_is_forward_switch and self.is_forward is False:
            # Mirror the C++ behavior: when using rear pose, flip yaw by 180 deg.
            yaw = _normalize_angle(yaw + math.pi)

        dx = self.goal.pose.position.x - pose.pose.position.x
        dy = self.goal.pose.position.y - pose.pose.position.y
        forward_m = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral_m = -math.sin(yaw) * dx + math.cos(yaw) * dy
        angle_rad = math.atan2(lateral_m, forward_m)
        distance_to_goal = math.hypot(dx, dy)

        path_remaining = (
            _path_remaining_length(self.path, (pose.pose.position.x, pose.pose.position.y))
            if self.path
            else distance_to_goal
        )
        effective_distance = min(distance_to_goal, path_remaining)

        guard_active = (
            self.debug_mode
            and self.guard_distance > 0.0
            and effective_distance <= self.guard_distance + 1e-3
        )

        unreachable = False
        reason = "guard inactive"
        if guard_active:
            guard_angle_rad = math.radians(self.guard_angle_deg)
            behind = forward_m < -self.forward_deadzone
            out_of_wedge = abs(angle_rad) > guard_angle_rad
            if not behind and not out_of_wedge:
                unreachable = False
                reason = "within guard wedge"
            else:
                unreachable = True
                if behind and out_of_wedge:
                    reason = "goal behind and outside wedge"
                elif behind:
                    reason = "goal behind deadzone"
                else:
                    reason = "goal outside wedge"

        return {
            "pose": pose,
            "yaw": yaw,
            "forward_m": forward_m,
            "lateral_m": lateral_m,
            "angle_rad": angle_rad,
            "distance_to_goal": distance_to_goal,
            "path_remaining": path_remaining,
            "effective_distance": effective_distance,
            "guard_active": guard_active,
            "unreachable": unreachable,
            "reason": reason,
        }

    def _timer_cb(self, _event) -> None:
        state = self._compute_guard()
        if state is None:
            # Clear markers until we have data
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[delete_all]))
            return

        markers = self._build_markers(state)
        self.marker_pub.publish(markers)

    # --- Marker construction --------------------------------------------
    def _build_markers(self, state: dict) -> MarkerArray:
        pose: PoseStamped = state["pose"]
        yaw = state["yaw"]
        guard_active = state["guard_active"]
        unreachable = state["unreachable"]
        reason = state["reason"]

        origin = pose.pose.position
        frame_id = pose.header.frame_id or "map"
        stamp = rospy.Time.now()

        circle = self._circle_marker(frame_id, stamp, origin, guard_active, unreachable)
        wedge = self._wedge_marker(frame_id, stamp, origin, yaw, guard_active, unreachable)
        deadzone = self._deadzone_marker(frame_id, stamp, origin, yaw, guard_active)
        goal_pt = self._goal_marker(frame_id, stamp, unreachable)
        return MarkerArray(markers=[circle, wedge, deadzone, goal_pt])

    def _circle_marker(
        self, frame_id: str, stamp: rospy.Time, origin, guard_active: bool, unreachable: bool
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "goal_guard/circle"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04

        color = (0.2, 0.8, 0.2) if guard_active and not unreachable else (0.9, 0.6, 0.2)
        if unreachable:
            color = (1.0, 0.2, 0.2)
        m.color = ColorRGBA(color[0], color[1], color[2], 0.8 if guard_active else 0.35)

        steps = 72
        for i in range(steps + 1):
            theta = 2.0 * math.pi * float(i) / float(steps)
            p = Point()
            p.x = origin.x + self.guard_distance * math.cos(theta)
            p.y = origin.y + self.guard_distance * math.sin(theta)
            p.z = origin.z
            m.points.append(p)
        return m

    def _wedge_marker(
        self, frame_id: str, stamp: rospy.Time, origin, yaw: float, guard_active: bool, unreachable: bool
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "goal_guard/wedge"
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05

        color = (0.0, 0.6, 1.0) if guard_active else (0.5, 0.5, 0.5)
        if unreachable:
            color = (1.0, 0.0, 0.0)
        m.color = ColorRGBA(color[0], color[1], color[2], 0.8 if guard_active else 0.35)

        guard_angle_rad = math.radians(self.guard_angle_deg)
        angles = [yaw - guard_angle_rad, yaw + guard_angle_rad]
        pts = [origin]
        for ang in angles:
            p = Point()
            p.x = origin.x + self.guard_distance * math.cos(ang)
            p.y = origin.y + self.guard_distance * math.sin(ang)
            p.z = origin.z
            pts.append(p)
        pts.append(origin)
        m.points = pts
        return m

    def _deadzone_marker(
        self, frame_id: str, stamp: rospy.Time, origin, yaw: float, guard_active: bool
    ) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "goal_guard/deadzone"
        m.id = 2
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color = ColorRGBA(1.0, 0.0, 0.0, 0.7 if guard_active else 0.3)

        start = Point()
        start.x = origin.x
        start.y = origin.y
        start.z = origin.z
        end = Point()
        end.x = origin.x - self.forward_deadzone * math.cos(yaw)
        end.y = origin.y - self.forward_deadzone * math.sin(yaw)
        end.z = origin.z

        m.points = [start, end]
        return m

    def _goal_marker(self, frame_id: str, stamp: rospy.Time, unreachable: bool) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "goal_guard/goal"
        m.id = 3
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.scale.x = 0.35
        m.scale.y = 0.35
        m.scale.z = 0.35

        color = (0.2, 0.9, 0.2) if not unreachable else (1.0, 0.1, 0.1)
        m.color = ColorRGBA(color[0], color[1], color[2], 0.9)

        if self.goal is not None:
            m.pose = self.goal.pose
        return m

    @staticmethod
    def _yaw_from_pose(pose_stamped: PoseStamped) -> float:
        q = pose_stamped.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


if __name__ == "__main__":
    viz = GoalGuardVisualizer()
    rospy.spin()
