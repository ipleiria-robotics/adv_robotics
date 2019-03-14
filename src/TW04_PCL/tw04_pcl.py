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

# Matrices and OpenCV related functions
import numpy as np
from matplotlib import pyplot as plt

# Library packages needed
from math import radians, degrees, ceil
import time
import random
import sys

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# Our functions
from utils import clipValue, quaternionToYaw, createLineIterator
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Map related constants
MAP_RESOLUTION = 0.05  # [m]
MAP_HEIGHT = 6.0  # [m]
MAP_WIDTH = 8.0  # [m]
MIN_CELL_VALUE = 0
MAX_CELL_VALUE = 255
CELL_DELTA_UP = 1  # Change value per increment update
CELL_DELTA_DOWN = 5  # Change value per decrement update
DELTA_DEBUG = 5  # Show debug information ony once every DELTA_PRINT seconds
DELTA_SAVE = 5  # Save the map to a file every DELTA_PRINT*DELTA_SAVE seconds

# Map fo the environment (to be fully created/updated)
occ_map = np.full((ceil(MAP_HEIGHT/MAP_RESOLUTION),
                   ceil(MAP_WIDTH/MAP_RESOLUTION),), 127, np.uint8)

# Control iterations between map file updates
iteration = 0

# Global variables
robot_pose = Pose2D()
robot_lin_vel = 0.0  # Store current linear velocity
robot_ang_vel = 0.0  # Store current angular velocity
closest_front_obstacle = 0.0
closest_left_obstacle = 0.0
closest_right_obstacle = 0.0
odom_updated = False  # True if we got an odometry update
laser_updated = False  # True if we got a laser data update


def odomCallback(data: Odometry):
    ''' Function to call whe new odometry information is available '''

    global robot_pose, robot_lin_vel, robot_ang_vel, odom_updated

    # Store updated values
    robot_pose.x = data.pose.pose.position.x
    robot_pose.y = data.pose.pose.position.y
    robot_pose.theta = quaternionToYaw(data.pose.pose.orientation)

    robot_lin_vel = data.twist.twist.linear.x
    robot_ang_vel = data.twist.twist.angular.z

    odom_updated = True


def laserCallback(msg: LaserScan):
    ''' Process laser data '''

    global laser_updated, closest_right_obstacle
    global closest_front_obstacle, closest_left_obstacle

    ###########################################################################
    # Update distance to closest obstacles for simple obstacle avoidance
    # navigation.
    ''' Right obstacle '''
    # Find index of -90 degrees
    min_angle_idx = round((radians(-90) - msg.angle_min)/msg.angle_increment)
    # Find index of -75 degrees
    max_angle_idx = round((radians(-75) - msg.angle_min)/msg.angle_increment)
    # Get smaller value in the given range
    closest_right_obstacle = min(msg.ranges[min_angle_idx:max_angle_idx+1])

    '''Front obstacle'''
    # Find index of -45 degrees
    min_angle_idx = round((radians(-45) - msg.angle_min)/msg.angle_increment)
    # Find index of 45 degrees
    max_angle_idx = round((radians(45) - msg.angle_min)/msg.angle_increment)
    # Get smaller value in the given range
    closest_front_obstacle = min(msg.ranges[min_angle_idx:max_angle_idx+1])

    '''Left obstacle'''
    # Find index of 75 degrees
    min_angle_idx = round((radians(75) - msg.angle_min)/msg.angle_increment)
    # Find index of 90 degrees
    max_angle_idx = round((radians(90) - msg.angle_min)/msg.angle_increment)
    # Get smaller value in the given range
    closest_left_obstacle = min(msg.ranges[min_angle_idx:max_angle_idx+1])
    ###########################################################################

    laser_updated = True


def pointCloudCallback(msg: PointCloud2):
    ''' Process point cloud data (generated from the laser) '''

    # Store robot position in map grid coordinates
    robot_map_coord = np.array(
        [round((MAP_WIDTH/2+robot_pose.x)/MAP_RESOLUTION),  # Col
         round((MAP_HEIGHT/2-robot_pose.y)/MAP_RESOLUTION)],  # Row
        dtype=int)

    for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                      skip_nans=True):

        # Get obtained value in the map grid (columm and row) - must be uint8.
        obstacle_map = np.array(
            [round((MAP_WIDTH/2+p[0])/MAP_RESOLUTION),  # Col / x
             round((MAP_HEIGHT/2-p[1])/MAP_RESOLUTION)],  # Row / y
            dtype=np.int)

        # Update map using the line iterator for free space
        it = createLineIterator(robot_map_coord,  # Start point
                                obstacle_map,  # End point
                                occ_map)
        for pt in it[:-1]:
            occ_map.itemset((pt[1], pt[0]), clipValue(pt[2]+CELL_DELTA_UP,
                                                      MIN_CELL_VALUE,
                                                      MAX_CELL_VALUE))

        # Update map using the line iterator for occupied space, if
        # applicable
        occ_map.itemset((it[-1][1], it[-1][0]),
                        clipValue(it[-1][2]-CELL_DELTA_DOWN,
                                  MIN_CELL_VALUE,
                                  MAX_CELL_VALUE))


def on_key(event):
    '''Quit if the q key was pressed'''
    if(event.key == 'q'):
        rospy.signal_shutdown('Quitting...')


if __name__ == '__main__':
    '''
    Main function
    Build a map using a lser and random navigation with obstacle avoidance.
    '''

    # Create a window to show the map
    fig = plt.figure("Map")
    # Associate the on_key callback for quitting on 'q' press
    fig.canvas.mpl_connect('key_press_event', on_key)

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    last_ang_vel = radians(10)  # 10 º/s
    vel_cmd = Twist()  # For velocity commands

    # Navigation variables
    avoid = False
    new_rotation = False
    stop_front_dist = 0.6  # [m]
    min_front_dist = 1.0  # [m]

    try:
        # Output usage information
        print('Navigation with map generation.\n' +
              '---------------------------\n')

        # Setup subscribers
        # Odometry
        sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry,
                                    odomCallback, queue_size=1)
        # Laser
        sub_laser = rospy.Subscriber(robot_name + '/base_scan', LaserScan,
                                     laserCallback, queue_size=1)
        # PointCloud
        sub_pcl = rospy.Subscriber(robot_name + '/cloud_filtered', PointCloud2,
                                   pointCloudCallback, queue_size=1)

        # Setup publisher
        vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

        # Init ROS
        rospy.init_node('tw04_pcl', anonymous=True)

        # Loop rate
        rate = rospy.Rate(10)  # 10 Hz
        prev_time = time.time()

        # Infinite loop
        while not rospy.is_shutdown():

            # Proceed only if we got a new laser update
            if(not laser_updated):
                laser_updated = False
                continue  # Use posture from odometry

            # Check for obstacles near the front of the robot
            if(closest_front_obstacle < min_front_dist):
                avoid = True
                if(closest_front_obstacle < stop_front_dist):
                    lin_vel = -0.100
                else:
                    lin_vel = 0
            else:
                avoid = False
                lin_vel = 0.5
                ang_vel = 0.0
                new_rotation = True

            # Rotate to avoid obstacles
            if(avoid):
                if(new_rotation):
                    # Swith rotation direction randomly with 0.1 probability
                    if(random.random() > 0.9):
                        last_ang_vel = -last_ang_vel
                ang_vel = last_ang_vel
                new_rotation = False

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

                # Show map
                plt.figure("Map")
                plt.cla()
                plt.imshow(occ_map, cmap='gray')
                plt.pause(0.01)

                # Save the map every DELTA_SAVE iterations
                iteration += 1
                if(iteration == DELTA_SAVE):
                    plt.imsave('map.png', occ_map, cmap='gray')
                    iteration = 0

                # Show pose estimated from odometry
                print(f'Robot estimated pose = {robot_pose.x:.2f} [m], ' +
                      f'{robot_pose.y:.2f} [m], ' +
                      f'{degrees(robot_pose.theta):.2f} [º]\r')

                # Show estimated velocity
                print(f'Robot estimated velocity = {robot_lin_vel:.2f} [m/s], '
                      f'{degrees(robot_ang_vel):.2f} [º/s]\r')

                # Show desired velocity
                print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
                      f'{degrees(ang_vel):.2f} [º/s]\r', flush=True)

            # Sleep, if needed, to maintain the desired frequency
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('Quitting on error...')

    finally:
        # If we are quitting, stop the robot
        vel_cmd.angular.z = 0
        vel_cmd.linear.x = 0
        vel_pub.publish(vel_cmd)
        print('Quitting...')
        sys.exit(0)
