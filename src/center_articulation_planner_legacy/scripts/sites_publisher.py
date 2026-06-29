#!/usr/bin/env python3
"""
Publish load/unload poses from sites.json to /initialpose and /move_base_simple/goal.

Usage example:
  python sites_publisher.py --mode load-unload --load-id load-01 --unload-id unload-01 --pile_percent 0.7 --y_offset 0.0
"""

import argparse
import json
import math
import sys
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


MAP_FRAME = "map"
DEFAULT_COV = 1e-3
SITES_PATH = Path(__file__).resolve().parent / "sites.json"


def parse_args():
    parser = argparse.ArgumentParser(description="Publish selected load/unload poses one-shot.")
    parser.add_argument("--mode", required=True, choices=["load-unload", "unload-load"], help="Direction: start->goal ordering.")
    parser.add_argument("--load-id", required=True, help="ID of load point (e.g., load-02).")
    parser.add_argument("--unload-id", required=True, help="ID of unload point (e.g., unload-05).")
    parser.add_argument("--pile_percent", required=True, type=float, help="Pile percent to match (applies to load selection).")
    parser.add_argument("--y_offset", required=True, type=float, help="Desired y_offset for load; nearest match will be selected.")
    parser.add_argument("--initialpose-topic", default="/initialpose", help="Topic for start pose (default: /initialpose).")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal", help="Topic for goal pose (default: /move_base_simple/goal).")
    return parser.parse_args()


def load_sites(path: Path):
    try:
        data = json.loads(path.read_text())
    except Exception as exc:  # pragma: no cover - CLI utility
        raise RuntimeError(f"Failed to read {path}: {exc}")
    if "sites" not in data or not isinstance(data["sites"], list):
        raise RuntimeError(f"{path} missing 'sites' list.")
    return data["sites"]


def filter_entries(entries, point_type, point_id):
    candidates = [e for e in entries if e.get("type") == point_type and e.get("id") == point_id]
    if not candidates:
        available_ids = sorted({e.get("id") for e in entries if e.get("type") == point_type})
        raise RuntimeError(
            f"No entries for type '{point_type}' with id '{point_id}'. "
            f"Available ids for type '{point_type}': {available_ids}"
        )
    return candidates


def select_load_entry(entries, target_pile, target_y_offset):
    pile_matches = [
        e for e in entries if e.get("pile_percent") is not None and math.isclose(e.get("pile_percent"), target_pile, abs_tol=1e-6)
    ]
    if not pile_matches:
        available = sorted({e.get("pile_percent") for e in entries if e.get("pile_percent") is not None})
        raise RuntimeError(f"No entries with pile_percent={target_pile}. Available pile_percent values: {available}")

    def y_offset_key(entry):
        y = entry.get("y_offset")
        if y is None:
            return (float("inf"), float("inf"))
        return (abs(y - target_y_offset), y)

    return min(pile_matches, key=y_offset_key)


def select_unload_entry(entries):
    if not entries:
        raise RuntimeError("No unload entries found.")
    return entries[0]


def validate_pose(entry):
    pose = entry.get("pose") or {}
    pos = pose.get("position") or {}
    ori = pose.get("orientation") or {}
    required_pos = ("x", "y")
    required_ori = ("x", "y", "z", "w")
    missing = [k for k in required_pos if k not in pos] + [k for k in required_ori if k not in ori]
    if missing:
        raise RuntimeError(f"Entry {entry} missing pose fields: {missing}")
    return pos, ori


def build_initialpose(entry):
    pos, ori = validate_pose(entry)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = MAP_FRAME
    msg.pose.pose.position.x = pos["x"]
    msg.pose.pose.position.y = pos["y"]
    msg.pose.pose.position.z = pos.get("z", 0.0)
    msg.pose.pose.orientation.x = ori["x"]
    msg.pose.pose.orientation.y = ori["y"]
    msg.pose.pose.orientation.z = ori["z"]
    msg.pose.pose.orientation.w = ori["w"]
    msg.pose.covariance = [0.0] * 36
    msg.pose.covariance[0] = DEFAULT_COV
    msg.pose.covariance[7] = DEFAULT_COV
    msg.pose.covariance[35] = DEFAULT_COV
    return msg


def build_goal(entry):
    pos, ori = validate_pose(entry)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = MAP_FRAME
    msg.pose.position.x = pos["x"]
    msg.pose.position.y = pos["y"]
    msg.pose.position.z = pos.get("z", 0.0)
    msg.pose.orientation.x = ori["x"]
    msg.pose.orientation.y = ori["y"]
    msg.pose.orientation.z = ori["z"]
    msg.pose.orientation.w = ori["w"]
    return msg


def format_entry(entry):
    pos = entry["pose"]["position"]
    return (
        f"id={entry.get('id')} type={entry.get('type')} pile_percent={entry.get('pile_percent')} "
        f"y_offset={entry.get('y_offset')} "
        f"position(x={pos.get('x'):.3f}, y={pos.get('y'):.3f}, z={pos.get('z', 0.0):.3f})"
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
    else:
        start_entry, goal_entry = unload_entry, load_entry

    print("Selections:")
    print(f"  Start ({args.initialpose_topic}): {format_entry(start_entry)}")
    print(f"  Goal  ({args.goal_topic}): {format_entry(goal_entry)}")
    print("Publishing...")

    rospy.init_node("sites_publisher", anonymous=True)
    init_pub = rospy.Publisher(args.initialpose_topic, PoseWithCovarianceStamped, queue_size=1, latch=True)
    goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=True)

    # Allow publishers to register with the master before sending latched messages.
    rospy.sleep(0.2)

    init_pub.publish(build_initialpose(start_entry))
    goal_pub.publish(build_goal(goal_entry))
    rospy.sleep(0.1)
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # pragma: no cover - CLI utility
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
