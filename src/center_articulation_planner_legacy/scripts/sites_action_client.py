#!/usr/bin/env python3
"""
Select start/goal from sites.json and send them to the navigate action server.

The point-selection CLI matches scripts/sites_publisher.py, but the goal is
delivered via the center_articulation_planner_legacy Navigate action for planner testing.
"""

import argparse
import sys

import actionlib
import rospy
from autonomous_loader_msgs.msg import NavigateAction, NavigateGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from sites_publisher import (
    MAP_FRAME,
    SITES_PATH,
    build_goal,
    build_initialpose,
    filter_entries,
    format_entry,
    load_sites,
    select_load_entry,
    select_unload_entry,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Send selected load/unload poses to navigate action server.")
    parser.add_argument("--mode", required=True, choices=["load-unload", "unload-load", "no-load", "no-unload"], help="Direction: start->goal ordering.")
    parser.add_argument("--load-id", required=True, help="ID of load point (e.g., load-02).")
    parser.add_argument("--unload-id", required=True, help="ID of unload point (e.g., unload-05).")
    parser.add_argument("--pile_percent", required=True, type=float, help="Pile percent to match (applies to load selection).")
    parser.add_argument("--y_offset", required=True, type=float, help="Desired y_offset for load; nearest match will be selected.")
    parser.add_argument("--initialpose-topic", default="/initialpose", help="Topic for start pose (latched).")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal", help="Unused (kept for CLI compatibility).")
    return parser.parse_args()


def derive_status(mode: str):
    """
    Map mode to last_status/current_status for NavigateGoal.
    load-unload  -> last=0 (load),   current=1 (unload)
    unload-load  -> last=1 (unload), current=0 (load)
    """
    if mode == "load-unload":
        return 0, 1
    elif mode == "unload-load":
        return 1, 0
    elif mode == "no-load":
        return 2, 0
    elif mode == "no-unload":
        return 2, 1
    return 2, 2


def feedback_cb(feedback):
    rospy.loginfo(
        "feedback: distance_to_goal=%.2f plan_succeeded=%s",
        feedback.distance_to_goal,
        feedback.plan_succeeded,
    )


def main():
    args = parse_args()
    entries = load_sites(SITES_PATH)

    load_candidates = filter_entries(entries, "load", args.load_id)
    unload_candidates = filter_entries(entries, "unload", args.unload_id)

    load_entry = select_load_entry(load_candidates, args.pile_percent, args.y_offset)
    unload_entry = select_unload_entry(unload_candidates)

    if args.mode == "load-unload":
        start_entry, goal_entry = load_entry, unload_entry
    elif args.mode == "unload-load":
        start_entry, goal_entry = unload_entry, load_entry
    elif args.mode == "no-load":
        start_entry, goal_entry = load_entry, load_entry
    elif args.mode == "no-unload":
        start_entry, goal_entry = unload_entry, unload_entry

    print("Selections:")
    print(f"  Start ({args.initialpose_topic}): {format_entry(start_entry)}")
    print(f"  Goal  (action navigate): {format_entry(goal_entry)}")
    print("Publishing start pose and sending action goal...")

    rospy.init_node("sites_action_client", anonymous=True)

    # Publish start pose for localization; the planner still consumes the action goal for navigation.
    init_pub = rospy.Publisher(args.initialpose_topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(0.2)
    # init_pub.publish(build_initialpose(start_entry))  # 本节点仅发goal就够测试了

    client = actionlib.SimpleActionClient("navigate", NavigateAction)
    rospy.loginfo("Waiting for navigate action server...")
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Navigate action server not available.")
        sys.exit(1)

    goal_msg = NavigateGoal()
    goal_msg.end_pose = build_goal(goal_entry)
    goal_msg.end_pose.header.frame_id = MAP_FRAME
    goal_msg.last_status, goal_msg.current_status = derive_status(args.mode)

    client.send_goal(goal_msg, feedback_cb=feedback_cb)
    rospy.loginfo("Goal sent. Waiting for result...")
    finished_before_timeout = client.wait_for_result(rospy.Duration(300.0))
    if not finished_before_timeout:
        rospy.logwarn("Timed out waiting for result; canceling goal.")
        client.cancel_goal()
        sys.exit(2)

    result = client.get_result()
    rospy.loginfo(
        "Result: success=%s dist_err=%.3f angle_err=%.3f err_code=%d msg=%s",
        result.success,
        result.final_distance_error,
        result.final_angle_error,
        result.error_code,
        result.error_msg,
    )
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # pragma: no cover - CLI utility
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
