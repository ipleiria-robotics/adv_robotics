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

# Control debug messages
DEBUG_NAVIGATION_NODE = 0

# Global variables and constants
# Robot pose (computed from another node/application)
robot_pose = Pose2D(0, 0, 0)
robot_pose_updated = False
robot_pose_lock = Lock()  # Prevent simultaneous read/write to robot_pose

# Robot navigation/motion related constants and variables
MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]


def robotPoseCallback(msg: PoseWithCovarianceStamped):
    '''
     Store real, error-free pose values given by the simulator (for debugging
    puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
    '''
    global robot_pose, robot_pose_updated, robot_pose_lock

    robot_pose_lock.acquire()
    robot_pose.x = msg.pose.pose.position.x
    robot_pose.y = msg.pose.pose.position.y
    robot_pose.theta = quaternion2yaw(msg.pose.pose.orientation)
    robot_pose_updated = True
    robot_pose_lock.release()

    if DEBUG_NAVIGATION_NODE:
        # Print the received estimated pose
        print('Robot odometry posture: (X,Y,Theta) = (' +
              f'{robot_pose.x:.2f}, {robot_pose.y:.2f}, ' +
              f'{degrees(robot_pose.theta):.2f})')


if __name__ == '__main__':
    '''
    Main function
    Fixed waypoints-based navigation without obstacle avoidance
    '''
    print('Fixed waypoints-based navigation without obstacle avoidance \n' +
          '---------------------------')

    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0  # [m/s]
    ang_vel = 0.0  # [°/s]

    # Navigation variables
    local_robot_pose = Pose2D(0, 0, 0)  # "Local copy" of robot_pose
    velocity_at_target = 0.2  # Desired velocity at next target
    curr_target = 0  # Current target location index
    Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
    Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
    min_distance = 0.1  # Minimum acceptance distance to target
    max_angle_to_target = radians(30.0)

    ''' ROS related code '''
    robot_name = '/robot_0'
    vel_cmd = Twist()  # Velocity commands

    # Init ROS
    rospy.init_node('tw09_navigation_node')

    # Setup subscriber for pose
    sub_pose = rospy.Subscriber(robot_name + '/odom_combined',
                                PoseWithCovarianceStamped,
                                robotPoseCallback, queue_size=1)

    # Setup publisher for velocity commands
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # Targets positions to be followed (in world coordinates [m])
    targets_wpos = [ft.Point2D(2.0, 2.0),  # 1
                    ft.Point2D(6.0, 2.0),  # 2
                    ft.Point2D(6.0, 7.0),  # 3
                    ft.Point2D(-2.0, 7.0),  # 4
                    ft.Point2D(-7.0, 0.0),  # 5
                    ft.Point2D(-5.0, -7.0),  # 6
                    ft.Point2D(0.0, -7.0)]  # 7
    num_targets = len(targets_wpos)
    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Wait for the robot pose to be updated at least once
    while not robot_pose_updated:
        rate.sleep()

    # Determine the target closest to the robot. We will start with that one.
    # We do that by computing the distance from the robot to all desired target
    # locations, and selecting the one with the lowest distance.
    lowest_sq_distance = inf  # inf stands for infinite (very large) value
    for i in range(num_targets):
        new_sq_distance = (robot_pose.x-targets_wpos[i].x)**2 + \
                          (robot_pose.y-targets_wpos[i].y)**2
        if new_sq_distance < lowest_sq_distance:
            lowest_sq_distance = new_sq_distance
            curr_target = i

    # Infinite loop
    # Go through each point continuously using a simple P controller
    while not rospy.is_shutdown():
        # Only change velocity controls if the robot pose estimate was updated
        if not robot_pose_updated:
            continue
        # Else, proceed
        # Store a "local copy" o robot_pose
        robot_pose_lock.acquire()
        robot_pose_updated = False
        local_robot_pose.x = robot_pose.x
        local_robot_pose.y = robot_pose.y
        local_robot_pose.theta = robot_pose.theta
        robot_pose_lock.release()

        # Compute the squared distance to the target
        distance = sqrt((local_robot_pose.x-targets_wpos[curr_target].x)**2 +
                        (local_robot_pose.y-targets_wpos[curr_target].y)**2)
        # If the distance is small enough, proceed to the next target
        if distance < min_distance:
            curr_target = (curr_target + 1) % num_targets
            print(f'Going for target {curr_target+1}: (' +
                  f'{targets_wpos[curr_target].x} ' +
                  f'{targets_wpos[curr_target].y})')
            continue

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = ft.world2Localp(local_robot_pose,
                                           targets_wpos[curr_target])
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = Kp_ang_vel * angle_to_target

        #  We will not update the linear velocity if the robot is not facing
        # the target enough. If it is, then the linear velocity will be
        # proportional to the distance, increased with the target velocity. We
        # actually use the squared distance just for performance reasons.
        if abs(angle_to_target) < max_angle_to_target:
            lin_vel = Kp_lin_vel * distance + velocity_at_target

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Send velocity commands
        vel_cmd.angular.z = 1.0 * ang_vel
        vel_cmd.linear.x = 1.0 * lin_vel
        vel_pub.publish(vel_cmd)

        # Proceed at desired framerate
        rate.sleep()

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # We're done
    sys.exit(0)
