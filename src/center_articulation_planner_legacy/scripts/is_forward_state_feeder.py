#!/usr/bin/env python3
"""
Listen to /is_forward commands and publish /is_forward_state acks for each required node.

This is a lightweight tester to unblock the planner state machine when running only the
planning package. It mirrors the command onto the feedback topic so the planner can
leave the "waiting for direction feedback" state.
"""

import rospy
from std_msgs.msg import Bool
from center_articulation_planner_legacy.msg import IsForwardState

# Default nodes the planner expects to report success.
DEFAULT_REQUIRED_NODES = ["controller", "goal_checker"]


class IsForwardStateFeeder:
    def __init__(self):
        # Allow overriding topics/nodes via private parameters.
        self.cmd_topic = rospy.get_param("~is_forward_topic", "/is_forward")
        self.state_topic = rospy.get_param("~is_forward_state_topic", "/is_forward_state")
        self.required_nodes = rospy.get_param("~required_nodes", DEFAULT_REQUIRED_NODES)

        self.pub = rospy.Publisher(self.state_topic, IsForwardState, queue_size=10)
        rospy.Subscriber(self.cmd_topic, Bool, self.on_cmd)

        rospy.loginfo(
            "Listening on %s, publishing acks to %s for nodes: %s",
            self.cmd_topic,
            self.state_topic,
            self.required_nodes,
        )

    def on_cmd(self, msg: Bool):
        """
        When the planner publishes /is_forward (true/false), reply with two
        IsForwardState messages (controller + goal_checker by default).
        """
        forward = bool(msg.data)
        stamp = rospy.Time.now()

        for node in self.required_nodes:
            state = IsForwardState()
            state.header.stamp = stamp             # ensure freshness (planner uses ~1s window)
            state.node_name = node
            state.forward = forward                # must match the command or planner will ignore
            state.success = True                   # mark direction switch as accepted
            self.pub.publish(state)

        rospy.loginfo(
            "Sent %d is_forward_state messages (forward=%s)",
            len(self.required_nodes),
            forward,
        )


def main():
    rospy.init_node("is_forward_state_feeder")
    feeder = IsForwardStateFeeder()
    rospy.loginfo("is_forward_state_feeder started; waiting for /is_forward commands.")
    rospy.spin()


if __name__ == "__main__":
    main()
