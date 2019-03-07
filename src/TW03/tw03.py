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
'''

# Library packages needed
from math import pi, atan2, radians, degrees

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# Our functions
from utils import clipValue, quaternionToYaw
from LocalFrameWorldFrameTransformations import Point2D, world2Local
from markers_msgs.msg import Markers

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Limits of the world where the robot is working
X_MAX_POS = 3.32  # [m]
Y_MAX_POS = 2.28  # [m]

# Global variables
robot_odometry_pose = Pose2D()  # Store real (error-free) robot pose
robot_estimated_pose = Pose2D()  # Store real (error-free) robot pose
true_lin_vel = 0.0  # Store real (error-free) linear velocity
true_ang_vel = 0.0  # Store real (error-free) angular velocity
odom_updated = False  # True if we got an odometry update
localization_from_markers_updated = False  # True if we markers information
'''Choose which pose to use, from odometry or estimated from the markers.
   Setting this variable to true means it will use odometry.'''
use_odometry = True


def odomCallback(data: Odometry):
    ''' Function to call whe new odometry information is available '''

    global robot_odometry_pose, true_ang_vel, true_lin_vel, odom_updated

    # Store updated values
    robot_odometry_pose.x = data.pose.pose.position.x
    robot_odometry_pose.y = data.pose.pose.position.y
    robot_odometry_pose.theta = quaternionToYaw(data.pose.pose.orientation)

    true_lin_vel = data.twist.twist.linear.x
    true_ang_vel = data.twist.twist.angular.z

    # Show pose estimated from odometry
    print(f'Robot odometry pose (X, Y, Theta)= {robot_odometry_pose.x:.2f} [m]'
          + f', {robot_odometry_pose.y:.2f} [m], '
          + f'{degrees(robot_odometry_pose.theta):.2f} [º]\r')

    odom_updated = True


def markersCallback(msg: Markers):
    ''' Process receiced markers data '''
    global localization_from_markers_updated, robot_estimated_pose
    localization_from_markers_updated = False

    # Store the world positions of the landmarks
    beacons_wpos = [Point2D(-X_MAX_POS, -Y_MAX_POS),  # 1
                    Point2D(-X_MAX_POS, Y_MAX_POS),  # 2
                    Point2D(X_MAX_POS, Y_MAX_POS),  # 3
                    Point2D(X_MAX_POS, -Y_MAX_POS)]  # 4

    # If we have less then 3 beacons, return an error
    if(msg.num_markers < 3):
        print('Unable to localize !\n')
        return

    # Display found beacons information
    print(f'Found {msg.num_markers} beacons.')
    for i in range(msg.num_markers):
        print(f'Beacon {msg.id[i]:.2f} : (range, bearing) = (' +
              f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})')

    # Compute the robot localization based on the three beacons found
    b1w = beacons_wpos[msg.id[0]-1]  # Beacon 1 world position
    b2w = beacons_wpos[msg.id[1]-1]  # Beacon 2 world position
    b3w = beacons_wpos[msg.id[2]-1]  # Beacon 3 world position

    # Fill variables
    d1s = msg.range[0]*msg.range[0]
    d2s = msg.range[1]*msg.range[1]
    d3s = msg.range[2]*msg.range[2]
    x1 = b1w.x
    y1 = b1w.y
    x2 = b2w.x
    y2 = b2w.y
    x3 = b3w.x
    y3 = b3w.y

    # Compute A
    # A = (d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)/(2*(y2-y1))
    # Compute B
    # B = (x1-x2)/(y2-y1)
    # Compute C
    # C = (d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)/(2*(y3-y2))
    # Compute D
    # D = (x2-x3)/(y3-y2)

    # Compute the robot position
    # robot_localized_pos->x = (A - C) / (D - B);
    # robot_localized_pos->y = A + B * robot_localized_pos->x;
    robot_estimated_pose.x = 0.5 * \
        ((y3-y2)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2) -
         (y2-y1)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)) / \
        ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2))
    robot_estimated_pose.y = 0.5 * \
        ((x2-x3)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2) -
         (x1-x2)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)) / \
        ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2))

    # Estimate the angle
    alpha0 = atan2(y1-robot_estimated_pose.y, x1-robot_estimated_pose.x)
    robot_estimated_pose.theta = alpha0-msg.bearing[0]
    # Limit the angle between -PI and PI
    if(robot_estimated_pose.theta > pi):
        robot_estimated_pose.theta -= 2*pi
    elif(robot_estimated_pose.theta < -pi):
        robot_estimated_pose.theta += 2*pi

    print(f'Robot estimated pose: (X,Y,Theta) = ' +
          f'{robot_estimated_pose.x:0.2f}, {robot_estimated_pose.y:0.2f}, ' +
          f'{degrees(robot_estimated_pose.theta):0.2f})', flush=True)

    localization_from_markers_updated = True


if __name__ == '__main__':
    '''
    Main function
    Controls the robot using the keyboard keys and outputs posture and velocity
    related information.
    '''

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    vel_cmd = Twist()  # For velocity commands

    # Navigation variables
    velocity_at_target = 0  # Desired velocity at next target
    curr_target = 0  # Current target location index
    Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
    Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
    min_distance = 0.1  # Minimum acceptance distance to target
    max_angle_to_target = radians(30.0)
    robot_pose = Pose2D()

    # Array of points to be followed:
    targets = [Point2D(-2.6, -1.7),  # 1
               Point2D(-2.6, 1.7),  # 2
               Point2D(0.0, 0.4),  # 3
               Point2D(2.6, 1.7),  # 4
               Point2D(2.6, -1.7),  # 5
               Point2D(0.0, -0.4)]  # 6
    num_targets = len(targets)

    try:
        # Output usage information
        print('Navigation with trilateration based localization.\n' +
              '---------------------------\n')

        # Setup subscribers
        # Odometry
        sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry,
                                    odomCallback, queue_size=1)
        sub_laser = rospy.Subscriber(robot_name + '/markers', Markers,
                                     markersCallback, queue_size=1)

        # Setup publisher
        vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

        # Init ROS
        rospy.init_node('tw03', anonymous=True)

        # Loop rate
        rate = rospy.Rate(10)  # 10 Hz

        # Wait until we have at least one localization update
        while((not rospy.is_shutdown()) and
              (not ((use_odometry and odom_updated) or
                    ((not use_odometry) and
                     localization_from_markers_updated)))):
            rate.sleep()

        if(use_odometry):
            odom_updated = False
            robot_pose = robot_odometry_pose
        else:
            localization_from_markers_updated = False
            robot_pose = robot_estimated_pose

        '''Determine the target closest to the robot. We'll start with that one.
           We do that by computing the distance from the robot to all desired
           target locations, and selecting the one with the lowest distance.'''
        lowest_sq_distance = pow(2*X_MAX_POS, 2) + pow(2*Y_MAX_POS, 2)
        for i in range(num_targets):
            new_sq_distance = pow(robot_pose.x-targets[i].x, 2) + \
                              pow(robot_pose.y-targets[i].y, 2)
            if(new_sq_distance < lowest_sq_distance):
                lowest_sq_distance = new_sq_distance
                curr_target = i

        # Infinite loop
        while not rospy.is_shutdown():

            # Only change navigation controls if markers were detected or the
            # odometry updated
            if((use_odometry and (not odom_updated)) or ((not use_odometry) and
               (not localization_from_markers_updated))):
                vel_pub.publish(vel_cmd)
                continue

            if(use_odometry):
                odom_updated = False
                robot_pose = robot_odometry_pose  # Use posture from odometry
            else:
                localization_from_markers_updated = False
                robot_pose = robot_estimated_pose  # Use pose from localization

            # Compute the squared distance to the target
            distance = pow(robot_pose.x-targets[curr_target].x, 2) + \
                pow(robot_pose.y-targets[curr_target].y, 2)
            # If the distance is small enough, proceed to the next target
            if(distance < min_distance):
                curr_target = (curr_target + 1) % num_targets
                print(f'Going for target {curr_target+1}', flush=True)
                continue

            # The angular velocity will be proportional to the angle of the
            # target as seen by the robot.
            target_local_pos = world2Local(robot_pose, targets[curr_target])
            angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
            ang_vel = Kp_ang_vel * angle_to_target

            #  We will not update the linear velocity if the robot is not
            # facing the target enough. If it is, then the linear velocity
            # will be proportional to the distance, increased with the target
            # velocity. We actually use the squared distance just for
            # performance reasons.
            if(abs(angle_to_target) < max_angle_to_target):
                lin_vel = Kp_lin_vel * distance + velocity_at_target

            # Limit maximum velocities
            lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
            ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

            # Send velocity commands
            vel_cmd.angular.z = ang_vel
            vel_cmd.linear.x = lin_vel
            vel_pub.publish(vel_cmd)

            # Sleep, if needed, to maintain the desired frequency
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        # If we are quitting, stop the robot
        vel_cmd.angular.z = 0
        vel_cmd.linear.x = 0
        vel_pub.publish(vel_cmd)
