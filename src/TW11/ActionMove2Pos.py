#!/usr/bin/env python3

# Copyright (c) 2020, Hugo Costelha
# All rights reserved
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
Move2Pos action: given a 2D position (X and Y), move to that position.
'''

# ROS related modules
import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped

# Our modules
import myglobals
from utils import quaternionToYaw, clipValue
import tw11.msg  # Import our (action generated) messages
import LocalFrameWorldFrameTransformations as ft

# Other modules
from math import radians, atan2, sqrt
import os
from threading import Lock

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Move2Pos:
    '''
        Move2Pos action

        Given a position goal, move the robot until that position is
        reached. The orientation is not takent into account here.
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    feedback = tw11.msg.Move2PosFeedback()
    result = tw11.msg.Move2PosResult()

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            name=ACTION_NAME, ActionSpec=tw11.msg.Move2PosAction,
            auto_start=False)

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum accepted distance to target [m]
        self.max_angle_to_target = radians(30.0)
        self.velocity_at_target = 0.0

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
                f' with goal position [{self.goal.target_position.x:0.2f},' +
                f' {self.goal.target_position.y:0.2f}]. [m]')

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
            target_position = ft.Point2D(self.goal.target_position.x,
                                         self.goal.target_position.y)

        # Else, proceed...
        robot_pose = Pose2D(msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            quaternionToYaw(msg.pose.pose.orientation))

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = ft.world2Localp(robot_pose, target_position)
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = self.Kp_ang_vel * angle_to_target

        #  We will not update the linear velocity if the robot is not facing
        # the target enough. If it is, then the linear velocity will be
        # proportional to the distance, increased with the target velocity. We
        # actually use the squared distance just for performance reasons.
        distance = sqrt((robot_pose.x-target_position.x)**2 +
                        (robot_pose.y-target_position.y)**2)
        if abs(angle_to_target) < self.max_angle_to_target:
            lin_vel = self.Kp_lin_vel * distance + self.velocity_at_target
        else:
            lin_vel = 0.0

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Publish feedback (current pose)
        self.feedback.base_position = robot_pose
        self.action_server.publish_feedback(self.feedback)

        # Send velocity commands
        self.vel_cmd.angular.z = ang_vel
        self.vel_cmd.linear.x = lin_vel
        self.vel_pub.publish(self.vel_cmd)

        # Did we reach the goal?
        if distance < self.min_distance:
            # Stop the robot
            self.vel_cmd.angular.z = 0.0
            self.vel_cmd.linear.x = 0.0
            self.vel_pub.publish(self.vel_cmd)
            # Stop this callback
            self.sub_pose.unregister()
            # Store final result
            self.result.final_position = robot_pose
            self.action_server.set_succeeded(self.result)
            rospy.loginfo(f'{ACTION_NAME} has succeeded!')


if __name__ == '__main__':
    # Init ROS node
    rospy.init_node(ACTION_NAME)
    # Create and start the action server
    server = Move2Pos()
    # Spin off all callbacks
    rospy.spin()
