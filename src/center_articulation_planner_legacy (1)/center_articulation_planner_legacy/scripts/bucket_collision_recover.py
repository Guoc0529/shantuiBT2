#!/usr/bin/env python3
"""
Bucket collision recovery node.

Computes bucket collisions from pose/map and, when triggered via a service call,
spins in place away from the contact until clearance is achieved.

Service:
- /bucket_collision_recover (std_srvs/Trigger): empty request; success/failure in response.

Publishes:
- /cmd_vel (geometry_msgs/TwistStamped): angular commands during recovery, zero on exit.
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, TriggerResponse

# Default parameters
DEFAULT_OFFSET_DISTANCE = 2.1175    # Distance from front frame to bucket center (m)
DEFAULT_BOX_LENGTH = 3.016          # Box length perpendicular to heading (m)
DEFAULT_BOX_WIDTH = 1.563           # Box width along heading (m)
DEFAULT_OBSTACLE_THRESHOLD = 99     # OccupancyGrid threshold for obstacle

DEFAULT_ANGULAR_SPEED = 0.178    # rad/s magnitude when spinning
DEFAULT_PUB_RATE = 10.0          # Hz for cmd_vel publishing during recovery
DEFAULT_DWELL_TIME = 1.0         # s to remain clear before declaring success防抖
DEFAULT_MAX_RECOVER_TIME = 5.0  # s timeout; <=0 disables timeout


def quaternion_to_yaw(orientation):
    """Extract yaw angle from quaternion."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
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


class BucketCollisionRecover:
    def __init__(self):
        rospy.init_node("bucket_collision_recover", anonymous=True)

        # Collision geometry parameters
        self.offset_distance = rospy.get_param("~offset_distance", DEFAULT_OFFSET_DISTANCE)
        self.box_length = rospy.get_param("~box_length", DEFAULT_BOX_LENGTH)
        self.box_width = rospy.get_param("~box_width", DEFAULT_BOX_WIDTH)
        self.obstacle_threshold = rospy.get_param("~obstacle_threshold", DEFAULT_OBSTACLE_THRESHOLD)

        # Recovery behavior parameters
        self.angular_speed = rospy.get_param("~angular_speed", DEFAULT_ANGULAR_SPEED)
        self.pub_rate = rospy.get_param("~pub_rate", DEFAULT_PUB_RATE)
        self.dwell_time = rospy.get_param("~dwell_time", DEFAULT_DWELL_TIME)
        self.max_recover_time = rospy.get_param("~max_recover_time", DEFAULT_MAX_RECOVER_TIME)
        self.cmd_vel_frame = rospy.get_param("~cmd_vel_frame", "base_link")

        # Cached data
        self.current_pose = None
        self.current_map = None

        # Subscribers
        rospy.Subscriber("/Odometry_qianlun", PoseStamped, self.pose_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=1)

        # Service server
        self.service = rospy.Service("bucket_collision_recover", Trigger, self.handle_recover)

        rospy.loginfo("BucketCollisionRecover initialized.")
        rospy.loginfo(f"  offset_distance: {self.offset_distance} m")
        rospy.loginfo(f"  box_length: {self.box_length} m (perpendicular to heading)")
        rospy.loginfo(f"  box_width: {self.box_width} m (along heading)")
        rospy.loginfo(f"  obstacle_threshold: {self.obstacle_threshold}")
        rospy.loginfo(f"  angular_speed: {self.angular_speed} rad/s")
        rospy.loginfo(f"  pub_rate: {self.pub_rate} Hz")
        rospy.loginfo(f"  dwell_time: {self.dwell_time} s")
        rospy.loginfo(f"  max_recover_time: {self.max_recover_time} s")

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

        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        step = resolution * 0.5  # Sample at half resolution for accuracy
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if self.point_in_polygon(x, y, corners):
                    if self.is_point_obstacle(x, y, occupancy_grid):
                        _, local_y = self.world_to_local(x, y, center_x, center_y, yaw)
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

    def get_collision_status(self):
        """Return (is_collision, side_status) where side_status is none/left/right/both."""
        if self.current_pose is None or self.current_map is None:
            return None, "unavailable"

        bucket_x, bucket_y, yaw = self.compute_bucket_center(self.current_pose)
        corners = self.compute_box_corners(bucket_x, bucket_y, yaw)
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

        return is_collision, side_status

    def publish_twist(self, angular_z):
        """Publish a TwistStamped with zero linear and given angular z."""
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.cmd_vel_frame
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_motion(self):
        """Publish a single zero Twist to stop motion."""
        self.publish_twist(0.0)

    def handle_recover(self, _req):
        """Service handler to perform recovery spin."""
        if self.current_pose is None:
            return TriggerResponse(success=False, message="No pose received yet.")
        if self.current_map is None:
            return TriggerResponse(success=False, message="No map received yet.")

        is_collision, side_status = self.get_collision_status()
        if is_collision is None:
            return TriggerResponse(success=False, message="Pose/map unavailable.")

        if not is_collision:
            return TriggerResponse(success=True, message="No collision; nothing to do.")

        if side_status == "both":
            return TriggerResponse(success=False, message="Collision on both sides; cannot recover.")
        if side_status not in ("left", "right"):
            return TriggerResponse(success=False, message=f"Unknown collision side: {side_status}")

        # Determine spin direction away from the contact.
        angular_z = -self.angular_speed if side_status == "left" else self.angular_speed
        rospy.loginfo(
            f"Starting collision recovery: side={side_status}, angular_z={angular_z:.3f} rad/s"
        )

        rate = rospy.Rate(self.pub_rate)
        start_time = rospy.Time.now()
        clear_since = None

        while not rospy.is_shutdown():
            is_collision, side_status = self.get_collision_status()
            now = rospy.Time.now()

            if is_collision is None:
                self.stop_motion()
                return TriggerResponse(success=False, message="Pose/map unavailable during recovery.")

            if side_status == "both":
                self.stop_motion()
                return TriggerResponse(success=False, message="Collision on both sides; aborting.")

            if not is_collision:
                # Clear condition: stay clear for dwell_time before declaring success.
                if clear_since is None:
                    clear_since = now
                elapsed_clear = (now - clear_since).to_sec()
                if elapsed_clear >= self.dwell_time:
                    self.stop_motion()
                    return TriggerResponse(success=True, message="Collision cleared.")
                # Stay stopped while waiting for dwell confirmation.
                self.publish_twist(0.0)
            else:
                clear_since = None
                if side_status not in ("left", "right"):
                    self.stop_motion()
                    return TriggerResponse(success=False, message=f"Unknown collision side: {side_status}")
                angular_z = -self.angular_speed if side_status == "left" else self.angular_speed
                self.publish_twist(angular_z)

            if self.max_recover_time > 0.0 and (now - start_time).to_sec() >= self.max_recover_time:
                self.stop_motion()
                return TriggerResponse(success=False, message="Recovery timed out.")

            rate.sleep()

        # If shutdown occurs, ensure we stop.
        self.stop_motion()
        return TriggerResponse(success=False, message="Recovery interrupted by shutdown.")


def main():
    try:
        node = BucketCollisionRecover()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
