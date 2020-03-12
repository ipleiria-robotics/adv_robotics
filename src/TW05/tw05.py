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

# Matrices and OpenCV related functions
import numpy as np
from matplotlib import pyplot as plt
import cv2

# Library packages needed
from math import radians, degrees, atan2
import time
import sys
import os

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# Our functions
from utils import clipValue, quaternionToYaw
from LocalFrameWorldFrameTransformations import Point2D, Point2Di, world2Local

# Debug related variables
DELTA_DEBUG = 1  # Show debug information ony once every DELTA_PRINT seconds

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Map related constants
MAP_RESOLUTION = 0.05  # [m]
SCALE_FACTOR = 0.25  # Amount of map scaling to apply

# Global variables
odo_robot_pose = Pose2D()
odo_lin_vel = 0.0  # Store current linear velocity
odo_ang_vel = 0.0  # Store current angular velocity
odom_updated = False  # True if we got an odometry update
end_program = False  # End infinite loop when True


def odomCallback(data: Odometry):
    ''' Function to call whe new odometry information is available '''

    global odo_robot_pose, odo_lin_vel, odo_ang_vel, odom_updated

    # Store updated values
    odo_robot_pose.x = data.pose.pose.position.x
    odo_robot_pose.y = data.pose.pose.position.y
    odo_robot_pose.theta = quaternionToYaw(data.pose.pose.orientation)

    odo_lin_vel = data.twist.twist.linear.x
    odo_ang_vel = data.twist.twist.angular.z

    odom_updated = True


def on_key(event):
    '''Quit if the q key was pressed'''
    global end_program
    if(event.key == 'q'):
        plt.close('all')  # Close all figures
        end_program = True  # In order to end the "infinite" loop


if __name__ == '__main__':
    '''
    Main function
    Build a map using a lser and random navigation with obstacle avoidance.
    '''

    # Colormap for displaying the results, allowing for a better visualization
    # Provides for different colors according to the pixel valuesl
    COLORMAP = 'plasma'  # or use 'YlOrRd'

    # Output usage information
    print('Navigation using potential fields.\n' +
          '---------------------------\n')

    # Scale to be used on the map
    scale = SCALE_FACTOR / MAP_RESOLUTION

    # Global target
    target = Point2D(2.6, 1.7)  # [m]

    # Potential repulsive factor
    kr = 0.7
    # Potential atractive factor
    ka = 1.0

    # Read map
    map_file_path = os.getenv("HOME") + "/ros/src/TW05/map.png"
    org_map_color = plt.imread(map_file_path)
    # Image is read as a color image with alpha (transparancy) channel, but we
    # just want a grayscale image, so lets keep only the 1st image plane
    org_map = org_map_color[:, :, 1]
    # In this stage, the image (map) is stored with floating point value, where
    # 0.0 is black and 1.0 is white.

    # This algorithm is very slow, so we will reduce the size of the input map
    # image
    resized_map = cv2.resize(org_map, None, fx=SCALE_FACTOR, fy=SCALE_FACTOR)

    # Show original (reduced in size) map
    fig = plt.figure("Original map (reduced scale)")
    plt.imshow(resized_map, cmap='gray')
    plt.pause(0.01)
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Compute repulsive potential map
    '''
    # Repulsive potential map will be the same size as the original map, filled
    # with zeros
    rep_pot = np.zeros_like(resized_map)

    # Go trough each map point and fill according the the distance to all
    # obstacles
    max_rep_value = 0.0
    for xr in range(0, resized_map.shape[1]):
        for yr in range(0, resized_map.shape[0]):
            rep_value = 0
            for xo in range(0, rep_pot.shape[1]):
                for yo in range(0, rep_pot.shape[0]):
                    if((xr == xo) and (yr == yo)):
                        rep_value += kr * (1.0-resized_map[yo, xo]) / \
                            (1.0 / (scale**2))
                    else:
                        #######################################################
                        # Compute the repulsive potential to add (when the
                        # position being evaluated is not part of an obstacle)
                        #######################################################
                        rep_value += 0.0  # CHANGE ME

                        #######################################################
            rep_pot[yr, xr] = rep_value
            if(rep_value > max_rep_value):
                max_rep_value = rep_value
    print(f'Max repulsive value: {max_rep_value:.2f}')

    # Show and store repulsive potential map (for debug purposes only)
    dbg_rep_pot = rep_pot/max_rep_value
    fig = plt.figure("Repulsive potential")
    pos = plt.imshow(dbg_rep_pot, cmap=COLORMAP)
    fig.colorbar(pos)
    plt.pause(0.01)
    plt.imsave('map_pot_rep.png', dbg_rep_pot, cmap='gray')
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Compute atractive potential map
    '''
    target_pxl = Point2Di(round(target.x*scale+resized_map.shape[1]/2.0),
                          round(-target.y*scale+resized_map.shape[0]/2.0))
    atractive_pot = np.zeros_like(resized_map)
    max_atr_value = 0.0
    for xr in range(0, atractive_pot.shape[1]):
        for yr in range(0, atractive_pot.shape[0]):
            ###################################################################
            # Compute here the atractive potential to add
            ###################################################################
            atractive_pot[yr, xr] = 0.0  # CHANGE ME

            ###################################################################

            if(atractive_pot[yr, xr] > max_atr_value):
                max_atr_value = atractive_pot[yr, xr]
    print(f'Max atractive value: {max_atr_value:.2f}')

    # Show and store atractive potential map (for debug purposes only)
    dbg_atr_pot = atractive_pot/max_atr_value
    fig = plt.figure("Attractive potential")
    pos = plt.imshow(dbg_atr_pot, cmap=COLORMAP)
    fig.colorbar(pos)
    plt.pause(0.01)
    plt.imsave('map_pot_atr.png', dbg_atr_pot, cmap='gray')
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Compute resulting potential
    '''
    resulting_pot = rep_pot + atractive_pot

    # Show and store resulting potential map (for debug purposes only)
    dbg_res_pot = resulting_pot/(max_atr_value+max_rep_value)
    fig = plt.figure("Resulting potential")
    pos = plt.imshow(dbg_res_pot, cmap=COLORMAP)
    fig.colorbar(pos)
    plt.pause(0.01)
    plt.imsave('map_pot_res.png', dbg_res_pot, cmap='gray')
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Navigate using the generated potential maps
    '''
    #
    # Create robot related objects
    #
    robot_name = '/robot_0'
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    vel_cmd = Twist()  # For velocity commands
    tolerance = 0.15  # [m]
    Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
    Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
    velocity_at_target = 0.2  # 0.5 # Desired velocity at next target
    max_angle_to_target = radians(5.0)

    # Setup subscribers
    # Odometry
    sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry,
                                odomCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Init ROS
    rospy.init_node('tw05')

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Wait until we get one odometry message
    while (not end_program) and (not odom_updated):
        rate.sleep()

    # Initialize target_pxl in map coordinates and current target in world
    # coordinates, from the robot current position
    target_pxl.x = round(odo_robot_pose.x*scale+resized_map.shape[1]/2.0)
    target_pxl.y = round(-odo_robot_pose.y*scale+resized_map.shape[0]/2.0)
    current_target = Point2D(
        (target_pxl.x - resized_map.shape[1]/2.0)/scale,
        (-target_pxl.y + resized_map.shape[0]/2.0)/scale)

    prev_time = time.time()
    # Infinite loop
    while (not end_program) and (not rospy.is_shutdown()):

        # Update our desired velocity only if we got a new odometry update
        if(not odom_updated):
            odom_updated = False
            continue

        '''
        Select the nearest cell with lower value
        '''
        # If we are near the current target, select the next one
        if((abs(odo_robot_pose.x - current_target.x) < tolerance) and
           (abs(odo_robot_pose.y - current_target.y) < tolerance)):
            #  Select near cell with lower potential
            current_potential = resulting_pot[target_pxl.y, target_pxl.x]
            next_target = Point2Di(target_pxl.x, target_pxl.y)
            for x in range(target_pxl.x-1, target_pxl.x+2):
                for y in range(target_pxl.y-1, target_pxl.y+2):
                    # Do not check points that are outside the map
                    if((x < 0) or (x >= resulting_pot.shape[1]) or
                       (y < 0) or (y >= resulting_pot.shape[0])):
                        continue
                    # Is this a point with lower potential?
                    if(resulting_pot[y, x] < current_potential):
                        current_potential = resulting_pot[y, x]
                        next_target.x = x
                        next_target.y = y

            # Is this the last point (if the next target is the same as the
            # previous target, then this is the last point)
            if((target_pxl.x == next_target.x) and
               (target_pxl.y == next_target.y)):
                # Print current potential value
                print(f'Final potential = {current_potential:.2f}')
                break
            else:
                target_pxl = next_target  # New target

            # Convert the target to world coordinates
            current_target.x = \
                (target_pxl.x - resized_map.shape[1]/2.0)/scale
            current_target.y = \
                (-target_pxl.y + resized_map.shape[0]/2.0)/scale

        '''
        Navigate the robot to the next target
        '''
        # The angular velocity will be proportional to the angle of the
        # target as seen by the robot.
        target_local_pos = world2Local(odo_robot_pose, current_target)
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = Kp_ang_vel * angle_to_target

        #  We will not update the linear velocity if the robot is not
        # facing target enough. If it is, then the linear velocity will be
        # proportional to the distance, increased with the target velocity.
        # We use the squared distance just for performance reasons.
        #
        # Compute the squared distance to the target
        distance = (odo_robot_pose.x-current_target.x)**2 + \
                   (odo_robot_pose.y-current_target.y)**2
        if(abs(angle_to_target) < max_angle_to_target):
            lin_vel = Kp_lin_vel * distance + velocity_at_target

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Send velocity commands
        vel_cmd.angular.z = ang_vel
        vel_cmd.linear.x = lin_vel
        vel_pub.publish(vel_cmd)

        # Debug information
        # Only show this debug information every once in a while, in order
        # to avoid consuming to much time showing information
        curr_time = time.time()
        if(curr_time - prev_time >= DELTA_DEBUG):
            prev_time = curr_time

            # Show pose estimated from odometry
            print(f'Robot estimated pose = {odo_robot_pose.x:.2f} [m], ' +
                  f'{odo_robot_pose.y:.2f} [m], ' +
                  f'{degrees(odo_robot_pose.theta):.2f} [º]\r')

            # Show estimated velocity
            print(f'Robot estimated velocity = {odo_lin_vel:.2f} [m/s], '
                  f'{degrees(odo_ang_vel):.2f} [º/s]\r')

            # Show desired velocity
            print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
                  f'{degrees(ang_vel):.2f} [º/s]\r', flush=True)

        # Sleep, if needed, to maintain the desired frequency
        rate.sleep()

    # Stop the robot
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    # Print current pose
    print(f'Final pose = {odo_robot_pose.x:.2f} [m], ' +
          f'{odo_robot_pose.y:.2f} [m],' +
          f'{degrees(odo_robot_pose.theta):.2f} [º]\r\n')
    print('Arrived at local minimum potential...press "q" to quit')
    # Keep figures open until the user quits. This is a blocking statement
    if not end_program:
        plt.show()
        plt.close('all')  # Close all figures
    sys.exit(0)
