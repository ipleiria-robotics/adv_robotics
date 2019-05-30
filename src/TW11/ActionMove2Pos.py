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
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped

# Other libraries
from math import radians, atan2, sqrt

# Our modules
import myglobals
from utils import quaternion2yaw, clipValue
import LocalFrameWorldFrameTransformations as ft


class Move2Pos(smach.State):
    ''' Define state Move2Pos '''
    def __init__(self, ignore_discharged: bool = False):
        self.ignore_discharged = ignore_discharged
        if ignore_discharged:
            smach.State.__init__(self,
                                 outcomes=['succeeded', 'aborted'],
                                 input_keys=['target_pose'],
                                 output_keys=['target_pose'])
        else:
            smach.State.__init__(
                self,
                outcomes=['succeeded', 'discharged', 'aborted'],
                input_keys=['target_pose'],
                output_keys=['target_pose'])
        ''' Initialize members for navigation control '''
        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum accepted distance to target
        self.goal_reached = False

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher(myglobals.robot_name + '/cmd_vel',
                                       Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Move2...')

        # Store specified values
        self.target_pose = userdata.target_pose
        self.target_pos = ft.Point2D(self.target_pose.x,
                                     self.target_pose.y)
        self.max_angle_to_target = radians(30.0)
        self.velocity_at_target = 0.0
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
            myglobals.power_status -= 2

            # Check the power level
            if (not self.ignore_discharged) and \
               (myglobals.power_status < myglobals.MIN_POWER_LEVEL):
                self.sub_pose.unregister()  # Stop pose callback
                return 'discharged'
            elif self.goal_reached:
                self.sub_pose.unregister()  # Stop pose callback
                userdata.target_pose = self.target_pose
                return 'succeeded'
            rate.sleep()

        self.sub_pose.unregister()  # Stop pose callback
        return 'aborted'

    def robotPoseCallback(self, msg: PoseWithCovarianceStamped):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        self.robot_pose = Pose2D(msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 quaternion2yaw(msg.pose.pose.orientation))

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = ft.world2Localp(self.robot_pose, self.target_pos)
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = self.Kp_ang_vel * angle_to_target

        #  We will not update the linear velocity if the robot is not facing
        # the target enough. If it is, then the linear velocity will be
        # proportional to the distance, increased with the target velocity. We
        # actually use the squared distance just for performance reasons.
        distance = sqrt((self.robot_pose.x-self.target_pos.x)**2 +
                        (self.robot_pose.y-self.target_pos.y)**2)
        if abs(angle_to_target) < self.max_angle_to_target:
            lin_vel = self.Kp_lin_vel * distance + self.velocity_at_target
        else:
            lin_vel = 0.0

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Send velocity commands
        self.vel_cmd.angular.z = ang_vel
        self.vel_cmd.linear.x = lin_vel
        self.vel_pub.publish(self.vel_cmd)

        if distance < self.min_distance:
            self.goal_reached = True
