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
Simple navigation based on specified waypoints without obstacle avoidance.
'''

# Our libraries and functions
import LocalFrameWorldFrameTransformations as ft
from utils import clipValue, quaternion2yaw

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped

# Other libraries
from threading import Lock
from math import radians, degrees, inf, atan2, sqrt
import sys


class Navigation:
    ''' Navigation class, to be used with the state machine to make it easier
    to go to a specific pose '''

    def __init__(self, robot_name):
        ''' Initialize members '''

        self.robot_name = robot_name

        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Linear and angular velocities for the robot (initially stopped)
        self.lin_vel = 0.0  # [m/s]
        self.ang_vel = 0.0  # [°/s]

        # Navigation variables
        self.local_robot_pose = Pose2D(0, 0, 0)  # "Local copy" of robot_pose
        self.Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum acceptance distance to target
        self.goal_reached = False

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist,
                                       queue_size=1)

    def start(self, goal_position: ft.Point2D,
              max_angle_to_target: float = radians(30.0),
              velocity_at_target: float = 0.0):
        ''' goal_pose - [X, Y, Theta] with the goal pose in [m], [m] and [rad].
            max_angle_to_target - Maximum allowed angle before having linear
          speed.
            velocity_at_target - Desired velocity at next target.
        '''

        # Store specified values
        self.target_pos = goal_position
        self.max_angle_to_target = max_angle_to_target
        self.velocity_at_target = velocity_at_target
        self.goal_reached = False

        # Setup subscriber for pose
        self.sub_pose = rospy.Subscriber(self.robot_name + '/odom_combined',
                                         PoseWithCovarianceStamped,
                                         self.robotPoseCallback, queue_size=1)
        # The control of the robot will be done in the callback

    def stop(self):
        ''' Unregister (unsubscribe) the callback, so that no more velocity
        commands are sent
        '''
        self.sub_pose.unregister()

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

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Send velocity commands
        self.vel_cmd.angular.z = 1.0 * ang_vel
        self.vel_cmd.linear.x = 1.0 * lin_vel
        self.vel_pub.publish(self.vel_cmd)

        if distance < self.min_distance:
            self.goal_reached = True
