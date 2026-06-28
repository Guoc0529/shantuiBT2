#!/usr/bin/env python3
# python final_goal_reached_test.py --hz 5 --count 10 --success
# python final_goal_reached_test.py --hz 2 --count 5 --dx 0.5 --dy 0.3 --angle 0.12 （这会失败）

import argparse
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion


def main():
    ap = argparse.ArgumentParser(description="Publish final_goal_reached test messages")
    ap.add_argument("--hz", type=float, default=5.0, help="publish rate in Hz")
    ap.add_argument("--count", type=int, default=10, help="number of messages to publish")
    ap.add_argument("--success", action="store_true", help="mark final goal as success (orientation.x=1.0)")
    ap.add_argument("--dx", type=float, default=0.2, help="pose.position.x (distance error X component)")
    ap.add_argument("--dy", type=float, default=0.1, help="pose.position.y (distance error Y component)")
    ap.add_argument("--angle", type=float, default=0.05, help="pose.position.z (angle error)")
    ap.add_argument("--topic", default="/final_goal_reached", help="topic name")
    args = ap.parse_args()

    rospy.init_node("final_goal_reached_test_pub", anonymous=True)
    pub = rospy.Publisher(args.topic, PoseStamped, queue_size=1)
    rate = rospy.Rate(args.hz)

    orientation = Quaternion()
    orientation.x = 1.0 if args.success else 0.0  # >0.5 => success
    orientation.w = math.sqrt(max(0.0, 1.0 - orientation.x ** 2))

    msg = PoseStamped()
    msg.pose.position.x = args.dx
    msg.pose.position.y = args.dy
    msg.pose.position.z = args.angle
    msg.pose.orientation = orientation

    for i in range(args.count):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo("Published final_goal_reached #%d (success=%s)", i + 1, args.success)
        rate.sleep()


if __name__ == "__main__":
    main()
