#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
from py_trees import common
import py_trees.display
from py_trees.decorators import Decorator
import py_trees.behaviour as behaviour
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys
import operator
import py_trees_ros_interfaces.action as py_trees_actions
from nav2_msgs.action import NavigateToPose


class Repeat(Decorator):
    """
    Repeat.

    :data:`~py_trees.common.Status.SUCCESS` is
    :data:`~py_trees.common.Status.RUNNING` up to a specified number at
    which point this decorator returns :data:`~py_trees.common.Status.SUCCESS`.

    :data:`~py_trees.common.Status.FAILURE` is always
    :data:`~py_trees.common.Status.FAILURE`.

    Args:
        child: the child behaviour or subtree
        num_success: repeat this many times (-1 to repeat indefinitely)
        name: the decorator name
    """

    def __init__(self, name: str, child: behaviour.Behaviour, num_success: int):
        super().__init__(name=name, child=child)
        self.success = 0
        self.num_success = num_success

    def initialise(self) -> None:
        """Reset the currently registered number of successes."""
        self.success = 0


    def update(self) -> common.Status:
        """
        Repeat until the nth consecutive success.

        Returns:
            :data:`~py_trees.common.Status.SUCCESS` on nth success,
            :data:`~py_trees.common.Status.RUNNING` on running, or pre-nth success
            :data:`~py_trees.common.Status.FAILURE` failure.
        """
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = f"failed, aborting [status: {self.success} success from {self.num_success}]"
            return common.Status.FAILURE
        elif self.decorated.status == common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = (
                f"success [status: {self.success} success from {self.num_success}]"
            )
            if self.success == self.num_success:
                return common.Status.SUCCESS
            else:
                return common.Status.RUNNING
        else:  # RUNNING
            self.feedback_message = (
                f"running [status: {self.success} success from {self.num_success}]"
            )
            return common.Status.RUNNING


class GetWayPoint(py_trees.behaviour.Behaviour):
    """
    """
    def __init__(self, name="GetWayPoint", way_points=[]):
        super(GetWayPoint, self).__init__(name)
        self.way_points = way_points
        self.way_pointBB = py_trees.blackboard.Client(name="WayPoint")
        self.way_pointBB.register_key(key="waypoint", access=py_trees.common.Access.WRITE)

    def update(self):
        if not self.way_points:
            return py_trees.common.Status.FAILURE
        else:
            (x, y) = self.way_points.pop(0)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            self.way_pointBB.waypoint = goal_msg
        return py_trees.common.Status.SUCCESS

def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree for send navigation commands
    Returns:
        the root of the tree
    """

    way_points = [(135.0, 82.0), (235.0, 124.0), (295.0, 211.0), (345.0, 294.0),
                  (435.0, 333.0), (515.0, 294.0), (553.0, 211.0), (515.0, 124.0),
                  (435.0, 82.0), (345.0, 124.0), (235.0, 294.0), (135.0, 333.0),
                  (45.0, 294.0), (13.0, 211.0), (45.0, 124.0)
                 ]
    sequence = py_trees.composites.Sequence("GoToWayPointSequence")
    get_waypoint = GetWayPoint(way_points=way_points, name="GetWayPoint")
    robot_name = "R2D2"
    action_client = py_trees_ros.action_clients.FromBlackboard(
        action_type=NavigateToPose,
        action_name="navigate_to_pose/" + robot_name,
        name="GoToWayPoint",
        key="waypoint",
        generate_feedback_message=lambda msg: "remaining: {0}".format(msg.feedback.distance_remaining)
    )
    sequence.add_children([get_waypoint, action_client])
    root = Repeat(
        name="GoToWayPointRepeatDecorator",
        child=sequence,
        num_success=len(way_points)
    )

    return root

def checkEnd(tree: py_trees_ros.trees.BehaviourTree):
    if tree.root.status == py_trees.common.Status.SUCCESS or tree.root.status == py_trees.common.Status.FAILURE:
        tree.shutdown()


def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rclpy.init(args=None)
    root = create_root()

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0, post_tick_handler=checkEnd)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()