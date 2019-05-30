#!/usr/bin/env python3
'''
Copyright (c) 2019, Hugo Costelha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
* Neither the name of the Player Project nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 4
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Revision $Id$

@package docstring
Application of a State Machine for high-level robotic tasks control.
'''

# ROS related modules
import rospy
import smach
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

# Our modules
import myglobals
from utils import quaternion2yaw, clipValue


class Rotate2Angle(smach.State):
    ''' Define state Rotate2Angle '''
    def __init__(self, ignore_discharged: bool = False):
        self.ignore_discharged = ignore_discharged
        if ignore_discharged:
            smach.State.__init__(self,
                                 outcomes=['succeeded', 'aborted'],
                                 input_keys=['target_pose'],
                                 output_keys=['change_target'])
        else:
            smach.State.__init__(
                self,
                outcomes=['succeeded', 'discharged', 'aborted'],
                input_keys=['target_pose'],
                output_keys=['change_target'])
        ''' Initialize members for navigation control '''
        # Robot navigation/motion related constants and variables
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_angle = 0.1  # Minimum accepted distance to target
        self.goal_reached = False

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher(myglobals.robot_name + '/cmd_vel',
                                       Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Rotate2...')

        # Store desired angle
        self.target_angle = userdata.target_pose.theta
        self.goal_reached = False

        # Setup subscriber for pose
        self.sub_pose = rospy.Subscriber(
            myglobals.robot_name + '/odom_combined',
            PoseWithCovarianceStamped,
            self.robotPoseCallback, queue_size=1)

        # Execution loop
        rate = rospy.Rate(myglobals.execution_rate)
        while not rospy.is_shutdown():
            # Consume power
            myglobals.power_status -= 1

            # Check the power level
            if (not self.ignore_discharged) and \
               (myglobals.power_status < myglobals.MIN_POWER_LEVEL):
                self.sub_pose.unregister()  # Stop pose callback
                userdata.change_target = False
                return 'discharged'
            elif self.goal_reached:
                self.sub_pose.unregister()  # Stop pose callback
                userdata.change_target = True
                return 'succeeded'
            rate.sleep()

        self.sub_pose.unregister()  # Stop pose callback
        return 'aborted'

    def robotPoseCallback(self, msg: PoseWithCovarianceStamped):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        self.robot_angle = quaternion2yaw(msg.pose.pose.orientation)

        # Use a P controller based on the desired and current angles
        error = (self.target_angle - self.robot_angle)
        ang_vel = self.Kp_ang_vel * error
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Send velocity commands
        self.vel_cmd.angular.z = ang_vel
        self.vel_cmd.linear.x = 0.0
        self.vel_pub.publish(self.vel_cmd)

        if abs(error) < self.min_angle:
            self.goal_reached = True
