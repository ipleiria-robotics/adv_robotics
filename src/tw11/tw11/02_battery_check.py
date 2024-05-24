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

from ament_index_python.packages import get_package_share_directory
import ar_utils.action as action

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """
    Create a basic tree with a battery to blackboard writer and a
    battery check that issues a warning sound if the battery level
    goes low.

    Returns:
        the root of the tree
    """

    root = py_trees.composites.Parallel(
        name="Tutorial 02",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=True)
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name="Battery2BB",
        topic_name="battery/state",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=0.30
    )
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)

    warn_bat_low = py_trees_ros.action_clients.FromConstant(
        action_type=action.PlaySound,
        action_name="ActionPlaySound",
        action_goal=action.PlaySound.Goal(
            sound_file=get_package_share_directory('tw10') +
            '/sounds/low_battery.wav'),
        name="WarnBatteryLow"
    )

    def check_battery_low_on_blackboard(
      blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = py_trees.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=warn_bat_low
    )
    idle = py_trees.behaviours.Running(name="Idle")

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    root.add_child(tasks)
    tasks.add_children([battery_emergency, idle])
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
        tree.setup(timeout=15)
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
