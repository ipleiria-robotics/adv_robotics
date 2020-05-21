#!/usr/bin/env python3

# Copyright (c) 2020, Hugo Costelha
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# * Neither the name of the Player Project nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE 4 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

'''@package docstring
Implementation of the Rotate2Angle action: given a orientation goal, rotate
the robot until that orientation is reached.
'''

# ROS related modules
import rospy
import actionlib
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Our modules
import myglobals
from utils import quaternionToYaw, clipValue
import tw11.msg  # Import our (action generated) messages

# Other modules
import os
from threading import Lock
from numpy import radians

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Rotate2Angle:
    '''
        Rotate2Angle action

        Given an orientation goal, rotate the robot until that orientation is
        reached
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    feedback = tw11.msg.Rotate2AngleFeedback()
    result = tw11.msg.Rotate2AngleResult()

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            name=ACTION_NAME, ActionSpec=tw11.msg.Rotate2AngleAction,
            auto_start=False)

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        # Robot navigation/motion related constants and variables
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_ang_vel = 3.0  # Proportional gain for the angular vel. control
        self.max_angle_error = radians(5.0)  # Maximum angle error

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher(myglobals.robot_name + '/cmd_vel',
                                       Twist, queue_size=1)

        # Register callback to be called when action is preempted
        self.action_server.register_preempt_callback(self.preempt_cb)

        # Register callback to be called whenever a new goal is requested
        self.action_server.register_goal_callback(self.goal_cb)

        # Start the action server
        self.action_server.start()

    def preempt_cb(self):
        ''' Callback to call when the action is preempted '''
        rospy.loginfo(f'{ACTION_NAME} was Preempted!')
        # Stop the robotPoseCallback callback
        self.sub_pose.unregister()
        # Change the action status to preempted
        self.action_server.set_preempted()

    def goal_cb(self):
        ''' Callback to call when the action as a new goal '''
        with self.goal_lock:  # Store desired angle
            self.goal = self.action_server.accept_new_goal()
            rospy.loginfo(
                f'Executing action {ACTION_NAME}' +
                f' with goal angle {self.goal.target_orientation:.2f}.')

        # Setup subscriber for pose
        # The majority of the work will be done in the robotPoseCallback
        self.sub_pose = rospy.Subscriber(
            myglobals.robot_name + '/odom_combined',
            PoseWithCovarianceStamped,
            self.robotPoseCallback, queue_size=1)

    def robotPoseCallback(self, msg: PoseWithCovarianceStamped):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        # If the action is not active or a preemption was requested,
        # return immediately
        if (not self.action_server.is_active()) or \
           self.action_server.is_preempt_requested():
            return

        # Get the desired orientation
        with self.goal_lock:
            target_orientation = self.goal.target_orientation

        # Else, proceed...
        robot_orientation = quaternionToYaw(msg.pose.pose.orientation)
        # Use a P controller based on the desired and current angles
        error = (target_orientation - robot_orientation)
        ang_vel = self.Kp_ang_vel * error
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Publish feedback (current robot orientation)
        self.feedback.base_orientation = robot_orientation
        self.action_server.publish_feedback(self.feedback)

        # Send velocity commands
        self.vel_cmd.angular.z = ang_vel
        self.vel_cmd.linear.x = 0.0
        self.vel_pub.publish(self.vel_cmd)

        # Did we reach the goal?
        if abs(error) < self.max_angle_error:
            # Stop the robot
            self.vel_cmd.angular.z = 0.0
            self.vel_cmd.linear.x = 0.0
            self.vel_pub.publish(self.vel_cmd)
            # Stop this callback
            self.sub_pose.unregister()
            # Store final result
            self.result.final_orientation = robot_orientation
            self.action_server.set_succeeded(self.result)
            rospy.loginfo(f'{ACTION_NAME} has succeeded!')


if __name__ == '__main__':
    # Init ROS node
    rospy.init_node(ACTION_NAME)
    # Create and start the action server
    server = Rotate2Angle()
    # Spin off all callbacks
    rospy.spin()
