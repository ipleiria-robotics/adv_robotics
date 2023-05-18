#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
# Adapted by Hugo Costelha (2023)
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import rclpy
import sys

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    will become responsible for data gathering behaviours.

    Returns:
        the root of the tree
    """
    robot_name = 'robot_0'

    # Create the root node
    root = py_trees.composites.Parallel(
        name="Tutorial 01",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name=f"/{robot_name}/battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=0.30
    )
    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    flipper = py_trees.behaviours.Periodic(name="Flip Eggs", n=2)

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(priorities)
    priorities.add_child(flipper)
    priorities.add_child(idle)

    return root


def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red +
            "failed to setup the tree, aborting [{}]".format(str(e)) +
            console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
