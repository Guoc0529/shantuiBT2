#!/usr/bin/env python3
"""
Bucket collision checker node.

Subscribes to front frame pose and occupancy grid map, checks if a rectangular
zone in front of the bucket collides with obstacles, and publishes the result.
Publishes:
- /able_to_unload (Bool): True when no collision.
- /bucket_collision_side (String): "none", "left", "right", or "both".
  "left"/"right" follow REP-103 (x forward, y left in the local frame).

Usage:
  rosrun center_articulation_planner_legacy bucket_collision_checker.py
"""

import math

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker


# Default parameters
DEFAULT_OFFSET_DISTANCE = 2.1175   # Distance from front frame to bucket center (m)
DEFAULT_BOX_LENGTH = 3.016         # Box length perpendicular to heading (m)
DEFAULT_BOX_WIDTH = 1.563          # Box width along heading (m)
DEFAULT_OBSTACLE_THRESHOLD = 99    # OccupancyGrid threshold for obstacle
DEFAULT_CHECK_RATE = 10.0       # Hz


def quaternion_to_yaw(orientation):
    """Extract yaw angle from quaternion."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def rotate_point(x, y, yaw):
    """Rotate a 2D point by yaw angle."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_rot = x * cos_yaw - y * sin_yaw
    y_rot = x * sin_yaw + y * cos_yaw
    return x_rot, y_rot


class BucketCollisionChecker:
    def __init__(self):
        rospy.init_node("bucket_collision_checker", anonymous=True)

        # Parameters
        self.offset_distance = rospy.get_param("~offset_distance", DEFAULT_OFFSET_DISTANCE)
        self.box_length = rospy.get_param("~box_length", DEFAULT_BOX_LENGTH)
        self.box_width = rospy.get_param("~box_width", DEFAULT_BOX_WIDTH)
        self.obstacle_threshold = rospy.get_param("~obstacle_threshold", DEFAULT_OBSTACLE_THRESHOLD)
        check_rate = rospy.get_param("~check_rate", DEFAULT_CHECK_RATE)

        # Cached data
        self.current_pose = None
        self.current_map = None

        # Subscribers
        rospy.Subscriber("/Odometry_qianlun", PoseStamped, self.pose_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Publishers
        self.unload_pub = rospy.Publisher("/able_to_unload", Bool, queue_size=1)
        self.marker_pub = rospy.Publisher("/bucket_zone_marker", Marker, queue_size=1)
        self.collision_side_pub = rospy.Publisher("/bucket_collision_side", String, queue_size=1)

        # Timer for collision check
        self.timer = rospy.Timer(rospy.Duration(1.0 / check_rate), self.check_collision)

        rospy.loginfo("BucketCollisionChecker initialized.")
        rospy.loginfo(f"  offset_distance: {self.offset_distance} m")
        rospy.loginfo(f"  box_length: {self.box_length} m (perpendicular to heading)")
        rospy.loginfo(f"  box_width: {self.box_width} m (along heading)")
        rospy.loginfo(f"  obstacle_threshold: {self.obstacle_threshold}")
        rospy.loginfo(f"  check_rate: {check_rate} Hz")

    def pose_callback(self, msg):
        """Cache the latest pose."""
        self.current_pose = msg

    def map_callback(self, msg):
        """Cache the latest map."""
        self.current_map = msg

    def compute_bucket_center(self, pose):
        """Compute bucket center position from front frame pose."""
        yaw = quaternion_to_yaw(pose.pose.orientation)
        bucket_x = pose.pose.position.x + self.offset_distance * math.cos(yaw)
        bucket_y = pose.pose.position.y + self.offset_distance * math.sin(yaw)
        return bucket_x, bucket_y, yaw

    def compute_box_corners(self, center_x, center_y, yaw):
        """
        Compute 4 corners of the rectangular box.

        Box is centered at (center_x, center_y) with:
        - box_width along heading direction (front-back)
        - box_length perpendicular to heading (left-right)
        """
        half_width = self.box_width / 2.0   # along heading
        half_length = self.box_length / 2.0  # perpendicular to heading

        # Local coordinates (x: along heading, y: left is positive per REP-103)
        corners_local = [
            (+half_width, +half_length),  # front-left
            (+half_width, -half_length),  # front-right
            (-half_width, -half_length),  # back-right
            (-half_width, +half_length),  # back-left
        ]

        # Rotate and translate to world coordinates
        corners_world = []
        for lx, ly in corners_local:
            rx, ry = rotate_point(lx, ly, yaw)
            corners_world.append((center_x + rx, center_y + ry))

        return corners_world

    def world_to_map_index(self, world_x, world_y, map_info):
        """Convert world coordinates to map grid indices."""
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        resolution = map_info.resolution

        map_x = int((world_x - origin_x) / resolution)
        map_y = int((world_y - origin_y) / resolution)

        return map_x, map_y

    def is_point_obstacle(self, world_x, world_y, occupancy_grid):
        """Check if a world point is an obstacle in the map."""
        map_info = occupancy_grid.info
        map_x, map_y = self.world_to_map_index(world_x, world_y, map_info)

        # Check bounds
        if map_x < 0 or map_x >= map_info.width:
            return False
        if map_y < 0 or map_y >= map_info.height:
            return False

        # Get occupancy value
        index = map_y * map_info.width + map_x
        occupancy = occupancy_grid.data[index]

        # -1 means unknown, treat as non-obstacle
        if occupancy < 0:
            return False

        return occupancy > self.obstacle_threshold

    def world_to_local(self, world_x, world_y, center_x, center_y, yaw):
        """Transform world point into bucket-local frame (x: heading, y: left per REP-103)."""
        dx = world_x - center_x
        dy = world_y - center_y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        return local_x, local_y

    def check_box_collision(self, corners, occupancy_grid, center_x, center_y, yaw):
        """
        Check if any point inside the box collides with obstacles, and track side hits.

        Discretize the box interior based on map resolution.
        """
        map_info = occupancy_grid.info
        resolution = map_info.resolution
        left_collision = False
        right_collision = False

        # Get bounding box in world coordinates
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Sample points within bounding box
        step = resolution * 0.5  # Sample at half resolution for accuracy
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if self.point_in_polygon(x, y, corners):
                    if self.is_point_obstacle(x, y, occupancy_grid):
                        _, local_y = self.world_to_local(x, y, center_x, center_y, yaw)
                        # local_y sign reflects the two lateral edges (box_length axis):
                        # local_y > 0 is left edge, local_y < 0 is right edge (REP-103).
                        if abs(local_y) < 1e-3:
                            left_collision = True
                            right_collision = True
                        elif local_y > 0.0:
                            left_collision = True
                        else:
                            right_collision = True
                        if left_collision and right_collision:
                            return True, left_collision, right_collision
                y += step
            x += step

        any_collision = left_collision or right_collision
        return any_collision, left_collision, right_collision

    def point_in_polygon(self, x, y, corners):
        """Check if point (x, y) is inside the polygon defined by corners."""
        n = len(corners)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = corners[i]
            xj, yj = corners[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def publish_marker(self, corners, frame_id, is_collision):
        """Publish visualization marker for the box."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bucket_zone"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Add corners (close the loop)
        for cx, cy in corners:
            p = Point()
            p.x = cx
            p.y = cy
            p.z = 0.1  # Slightly above ground for visibility
            marker.points.append(p)
        # Close the rectangle
        p = Point()
        p.x = corners[0][0]
        p.y = corners[0][1]
        p.z = 0.1
        marker.points.append(p)

        # Scale (line width)
        marker.scale.x = 0.1

        # Color: green if safe, red if collision
        if is_collision:
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        else:
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green

        marker.lifetime = rospy.Duration(0.2)  # Short lifetime for continuous update

        self.marker_pub.publish(marker)

    def check_collision(self, event):
        """Timer callback to check collision and publish results."""
        if self.current_pose is None:
            rospy.logwarn_throttle(5.0, "No pose received yet.")
            return

        if self.current_map is None:
            rospy.logwarn_throttle(5.0, "No map received yet.")
            return

        # Compute bucket center
        bucket_x, bucket_y, yaw = self.compute_bucket_center(self.current_pose)

        # Compute box corners
        corners = self.compute_box_corners(bucket_x, bucket_y, yaw)

        # Check collision
        is_collision, left_collision, right_collision = self.check_box_collision(
            corners, self.current_map, bucket_x, bucket_y, yaw
        )

        if left_collision and right_collision:
            side_status = "both"
        elif left_collision:
            side_status = "left"
        elif right_collision:
            side_status = "right"
        else:
            side_status = "none"

        if is_collision:
            rospy.logwarn_throttle(
                1.0, f"Bucket collision detected on {side_status} side(s)."
            )

        # Publish able_to_unload (True if no collision, False if collision)
        msg = Bool()
        msg.data = not is_collision
        self.unload_pub.publish(msg)

        # Publish collision side info (y>0 is left in the local frame used above, REP-103)
        side_msg = String()
        side_msg.data = side_status
        self.collision_side_pub.publish(side_msg)

        # Publish visualization marker
        frame_id = self.current_pose.header.frame_id
        if not frame_id:
            frame_id = "map"
        self.publish_marker(corners, frame_id, is_collision)


def main():
    try:
        node = BucketCollisionChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
