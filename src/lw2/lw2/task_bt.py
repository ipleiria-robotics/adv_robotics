#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simple behavior tree for robot task execution using py_trees and ROS 2."""
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
# Adapted by Hugo Costelha (2024)
#

##############################################################################
# Imports
##############################################################################

# Python
import operator
import sys

# ROS
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8MultiArray

# BTs
import py_trees
import py_trees.console as console
import py_trees_ros.trees


# Advanced Robotics
import ar_utils.action as action
import ar_utils.msg
import lw2.myglobals as myglobals
from lw2.behaviors import GenerateNextPose

##############################################################################
# Tutorial
##############################################################################


def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    """BT entry point.

    Create a basic tree with a battery to blackboard writer and a
    battery check that issues a warning sound if the battery level
    goes low.

    Returns:
        the root of the tree
    """
    # The BT root node
    root = py_trees.composites.Parallel(
        name='LW2-Task',
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    # The parent composite to read all relevant ROS topics to the blackboard
    topics2bb = py_trees.composites.Sequence(name='Topics2BB', memory=True)
    # Add the topics2bb as a root child
    root.add_child(topics2bb)
    # Battery topic
    battery2bb = py_trees_ros.battery.ToBlackboard(
        name='Battery2BB',
        topic_name='battery/state',
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        threshold=0.0  # We are not using this threshold
    )
    # Force recharge event topic (messages of type std_msgs.msg.Empty)
    force_recharge2bb = py_trees_ros.subscribers.EventToBlackboard(
        name='ForceRecharge2BB',
        topic_name='dashboard/force_recharge',
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name='event_force_recharge'
    )
    # Proceed to a new motion - event topic (messages of type
    # std_msgs.msg.Empty)
    start2bb = py_trees_ros.subscribers.EventToBlackboard(
        name='Start2BB',
        topic_name='dashboard/start',
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        variable_name='event_start'
    )
    # Subscribe to the forklift state
    forkstate2bb = py_trees_ros.subscribers.ToBlackboard(
        name='ForliftState2BB',
        topic_name='forklift/state',
        topic_type=ar_utils.msg.ForkliftState,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        blackboard_variables={'forklift_state': None}
    )
    # Subscribe to the parts state
    parts2bb = py_trees_ros.subscribers.ToBlackboard(
        name='PartsState2BB',
        topic_name='parts_sensor',
        topic_type=UInt8MultiArray,
        qos_profile=py_trees_ros.utilities.qos_profile_latched(),
        clearing_policy=py_trees.common.ClearingPolicy.NEVER,
        blackboard_variables={'parts_state': None}
    )
    # Add the children of the topics2bb
    topics2bb.add_children([battery2bb, force_recharge2bb, start2bb,
                            forkstate2bb, parts2bb])

    ########################################################################
    # Tasks
    ########################################################################
    # Parent composite for running the actual tasks
    tasks = py_trees.composites.Selector(name='Tasks', memory=False)
    # The tasks composite node will also be a root child
    root.add_child(tasks)

    ########################################################################
    # Tasks - Recharging/battery component
    ########################################################################
    # Build a sequence so that the robot recharges when the battery is low
    recharge_seq = py_trees.composites.Sequence(name='RechargeSeq',
                                                memory=True)

    # Recharge is possible by moving the robot to a predefined pose. Do that
    # using a sequence of actions Move2Pos, Rotate2Angle and Recharge
    # To make sure the robot charges all the way through, once started, it will
    # only stop when finished
    startedRachargeBehavior = py_trees.behaviours.SetBlackboardVariable(
        name='SetRechargeBehaviorTrueOnBB',
        variable_name='running_recharge_behavior',
        variable_value=True,
        overwrite=True
    )
    # Move2Pos action to the recharge position
    move2recharge_pos = py_trees_ros.action_clients.FromConstant(
        action_type=action.Move2Pos,
        action_name='ActionMove2Pos',
        action_goal=action.Move2Pos.Goal(target_position=Point(
            x=myglobals.recharge_targets_wpose.x,
            y=myglobals.recharge_targets_wpose.y,
            z=0.0)),
        name='Move2RechargePos'
    )
    # Rotate2Angle action to the recharge orientation
    rotate2recharge_angle = py_trees_ros.action_clients.FromConstant(
        action_type=action.Rotate2Angle,
        action_name='ActionRotate2Angle',
        action_goal=action.Rotate2Angle.Goal(
            target_orientation=myglobals.recharge_targets_wpose.theta),
        name='Rotate2RechargeAngle'
    )
    # Recharge action
    recharge = py_trees_ros.action_clients.FromConstant(
        action_type=action.Recharge,
        action_name='ActionRecharge',
        action_goal=action.Recharge.Goal(target_battery_level=1.0),
        name='Recharge'
    )
    endedRechargeBehavior = py_trees.behaviours.SetBlackboardVariable(
        name='SetRechargeBehaviorFalseOnBB',
        variable_name='running_recharge_behavior',
        variable_value=False,
        overwrite=True
    )
    # Add all these actions to the battery sequence
    recharge_seq.add_children([startedRachargeBehavior,
                               move2recharge_pos,
                               rotate2recharge_angle,
                               recharge,
                               endedRechargeBehavior])

    # Behaviour for the ActionClient SpeakText (warning sound)
    warn_bat_low = py_trees_ros.action_clients.FromConstant(
        action_type=action.SpeakText,
        action_name='ActionSpeakText',
        action_goal=action.SpeakText.Goal(
            text_to_speak='Warning, low battery!'),
        name='WarnBatteryLow'
    )

    # Build method and decorator so it returns True if a low battery warning is
    # issued.
    def check_battery_low_on_blackboard(
      blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.battery.percentage < myglobals.MIN_POWER_LEVEL
    battery_warning = py_trees.decorators.EternalGuard(
        name='Battery Low?',
        condition=check_battery_low_on_blackboard,
        blackboard_keys={'battery'},
        child=warn_bat_low
    )
    # Not having a low battery is not a failure for us
    battery_low_issuccess = py_trees.decorators.FailureIsSuccess(
        child=battery_warning,
        name='FailureIsSuccess'
    )

    # We want to warn the user while the battery is low and recharge it
    battery_control = py_trees.composites.Parallel(
        name='BatteryControl',
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            synchronise=False,
            children=[recharge_seq]
        ),
        children=[battery_low_issuccess, recharge_seq]
    )

    # Build method and decorator so it returns True if a low battery warning is
    # issued. or a force recharge happened.
    def check_battery_low_force_recharge_charging_on_blackboard(
      blackboard: py_trees.blackboard.Blackboard) -> bool:
        return ((blackboard.battery.percentage <
                 myglobals.MIN_POWER_LEVEL) or
                blackboard.event_force_recharge or
                (blackboard.exists('running_recharge_behavior') and
                 blackboard.running_recharge_behavior))
    battery_emergency = py_trees.decorators.EternalGuard(
        name='Battery Low, Forced Recharge or RechargeSeq?',
        condition=check_battery_low_force_recharge_charging_on_blackboard,
        blackboard_keys={'battery', 'event_force_recharge',
                         'running_recharge_behavior'},
        child=battery_control
    )
    # Add this decorator as a child of the battery sequence
    tasks.add_child(battery_emergency)

    ########################################################################
    # Tasks - Move to a pose component
    ########################################################################

    # Build a subtree to move the robot to a desired pose, which will be formed
    # by a sequence of Move2Pos and Rotate2Angle actions
    move2pose_seq = py_trees.composites.Sequence(name='Move2Pose', memory=True)
    # Add the sequence to the tasks
    tasks.add_child(move2pose_seq)

    # Make sure we got a start order before proceeding to the next goal.
    is_start_requested = py_trees.behaviours.CheckBlackboardVariableValue(
        name='Start?',
        check=py_trees.common.ComparisonExpression(
            variable='event_start',
            value=True,
            operator=operator.eq
        )
    )

    # Run a non action behaviour which takes the goal pose and publishes.
    # This behaviour was defined by us following the behavior template provided
    # https://py-trees.readthedocs.io/en/release-2.1.x/behaviours.html#skeleton-behaviour-include
    pose2pos_and_angle = GenerateNextPose(
        name='GenerateNextPose',
        pos_key='goal_pos',
        angle_key='goal_angle'
    )

    # Move2Pos action to the desired position
    move2pos = py_trees_ros.action_clients.FromBlackboard(
        action_type=action.Move2Pos,
        action_name='ActionMove2Pos',
        key='goal_pos',
        name='Move2Pos'
    )

    # Rotate2Angle action to the desired orientation
    rotate2angle = py_trees_ros.action_clients.FromBlackboard(
        action_type=action.Rotate2Angle,
        action_name='ActionRotate2Angle',
        key='goal_angle',
        name='Rotate2Angle'
    )
    # Add all these actions to the battery sequence
    move2pose_seq.add_children([is_start_requested,
                                pose2pos_and_angle,
                                move2pos,
                                rotate2angle])

    return root


def main():
    """Entry point for the demo script."""
    rclpy.init(args=None)
    root = tutorial_create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red + f'failed to setup the tree, aborting [{str(e)}]' +
            console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror('tree setup interrupted')
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
