#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Python packages
from math import pi

# ROS packages
import py_trees
from geometry_msgs.msg import Point, Pose2D

# Our packages
import ar_utils.action as action


class GenerateNextPose(py_trees.behaviour.Behaviour):
    """
    Generate a new goal pose and make available the corresponding position and
    orientation has two separate keys. We could just make that pose avalable
    as a single variable, if Move2Pose accepted Pose2D (or Pose) goals, but it
    is left as it is to provide a richer example.
    """
    def __init__(self, name, pos_key: str, angle_key: str):
        super(GenerateNextPose, self).__init__(name)
        self.pos_key = pos_key
        self.angle_key = angle_key

        # Waypoints list
        self.waypoints = [Pose2D(x=0.0, y=1.0, theta=0.0),
                          Pose2D(x=0.0, y=-1.0, theta=-pi)]
        self.curr_waypoint = -1

    def setup(self, **kwargs):
        """ Get blackboard client and register for the relevant keys """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        # Store the node. You can use it for any ROS-related functionality
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs " + \
                            "[{}][{}]".format(self.qualified_name,
                                              self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        # Not get setup the blackboard keys acess
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key=self.pos_key,
                                     access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=self.angle_key,
                                     access=py_trees.common.Access.WRITE)
        self.logger.debug("  %s [GenerateNextPose::setup()]" % self.name)

    def initialise(self):
        """ Nothing special to do """
        self.logger.debug("  %s [GenerateNextPose::initialise()]" % self.name)

    def update(self):
        """
        Read the goal_pose from the blackboard and store the position and
        orientation
        """
        self.logger.debug("  %s [GenerateNextPose::update()]" % self.name)

        # This behavior will always be successfull. But if it that was not the
        # case, it would return failure
        # self.feedback_message = "Some failure message!"
        # return py_trees.common.Status.FAILURE

        # If the behavior could be unning for a while, we would have to return
        # py_trees.common.Status.RUNNING, and not block its execution.

        # In this example we just need to create the position and orientation
        # keys corresponding to the next desired pose.
        self.curr_waypoint = (self.curr_waypoint + 1) % len(self.waypoints)
        self.blackboard.set(self.pos_key,
                            action.Move2Pos.Goal(target_position=Point(
                                x=self.waypoints[self.curr_waypoint].x,
                                y=self.waypoints[self.curr_waypoint].y,
                                z=0.0)))
        self.blackboard.set(
            self.angle_key,
            action.Rotate2Angle.Goal(
                target_orientation=self.waypoints[self.curr_waypoint].theta))
        self.feedback_message = "New position and orientation generated!"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting
            down
        """
        self.logger.debug(f"  {self.name} [GenerateNextPose::terminate()." +
                          f"terminate()]{self.status}->{new_status}]")
