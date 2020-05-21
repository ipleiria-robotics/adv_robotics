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
Stop action: stop the robot.
'''

# ROS related modules
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Our modules
import myglobals
import tw11.msg  # Import our (action generated) messages

# Other modules
import os
from numpy import sqrt

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Stop:
    '''
        Stop action

        Stop the robot motion.
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    # feedback = tw11.msg.StopFeedback()
    result = tw11.msg.StopResult()

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            name=ACTION_NAME, ActionSpec=tw11.msg.StopAction,
            auto_start=False)

        self.result.is_stopped = False

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
        self.sub_odom.unregister()
        # Change the action status to preempted
        self.action_server.set_preempted(self.result)

    def goal_cb(self):
        ''' Callback to call when the action as a new goal '''
        #with self.goal_lock:  # Store desired angle
        self.goal = self.action_server.accept_new_goal()
        rospy.loginfo(
            f'Executing action {ACTION_NAME}')

        # Setup subscriber for pose
        # The majority of the work will be done in the robotPoseCallback
        self.sub_odom = rospy.Subscriber(
            myglobals.robot_name + '/odom',
            Odometry, self.robotOdomCallback, queue_size=1)

    def robotOdomCallback(self, msg: Odometry):
        '''
        Check current velocity and, if the robot is not stopped, stop it.
        '''
        # If the action is not active or a preemption was requested,
        # return immediately
        if (not self.action_server.is_active()) or \
           self.action_server.is_preempt_requested():
            return

        # Check if the robot is moving
        lin_speed = sqrt(msg.twist.twist.linear.x**2 +
                         msg.twist.twist.linear.y**2)
        if (lin_speed > 0.001) or (msg.twist.twist.angular.z > 0.002):
            # Ask the robot to stop
            self.vel_cmd.angular.z = 0.0
            self.vel_cmd.linear.x = 0.0
            self.vel_pub.publish(self.vel_cmd)
        else:
            # Stop this callback
            self.sub_odom.unregister()
            # Store final result
            self.result.is_stopped = True
            self.action_server.set_succeeded(self.result)
            rospy.loginfo(f'{ACTION_NAME} has succeeded!')


if __name__ == '__main__':
    # Init ROS node
    rospy.init_node(ACTION_NAME)
    # Create and start the action server
    server = Stop()
    # Spin off all callbacks
    rospy.spin()
