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

# Matrix and math related functions
import numpy as np
from math import degrees
from matplotlib import pyplot as plt

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan

# Other libraries
import sys
import time

# Global variables
pose = Pose2D()  # Robot pose received from the localization module
end_program = False  # End infinite loop when True
# Map fo the environment (to be fully created/updated)
# CHANGE THE RESOLUTION ACCORDING TO YOUR IMPLEMENTATION
occ_map = np.full((520, 520), 127, np.uint8)


def poseCallback(msg: Pose2D):
    ''' Function to call when new pose information is available '''

    global pose

    # Store updated values
    pose.x = msg.x
    pose.y = msg.y
    pose.theta = msg.theta

    # Show estimated pose
    # NOTE: this is here only for testing purposes. You should avoid printing
    # information so often
    print(f'Robot estimated pose (X, Y, Theta)= {pose.x:.2f} [m]' +
          f', {pose.y:.2f} [m], ' +
          f'{degrees(pose.theta):.2f} [º]\r')


def laserCallback(msg: LaserScan):
    ''' Process laser data '''
    print('laserCallback not yet implemented!')


def on_key(event):
    global end_program
    '''Quit if the q key was pressed'''
    if(event.key == 'q'):
        end_program = True


if __name__ == '__main__':
    '''
    Main function
    Build a map using a laser and the estimated pose.
    '''
    # Create a window to show the map
    fig = plt.figure("Map")
    # Associate the on_key callback for quitting on 'q' press
    fig.canvas.mpl_connect('key_press_event', on_key)

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'
    vel_cmd = Twist()  # For velocity commands

    # Setup subscribers
    # Pose subscriber
    sub_pose = rospy.Subscriber(robot_name + '/pose', Pose2D,
                                poseCallback, queue_size=1)
    # Laser subscriber
    sub_laser = rospy.Subscriber(robot_name + '/base_scan', LaserScan,
                                 laserCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Init ROS
    rospy.init_node('lw01_map')

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz
    prev_time = time.time()

    # Infinite loop
    while (not end_program) and (not rospy.is_shutdown()):
        # Debug information
        # Only show this debug information every once in a while, in order
        # to avoid consuming to much time debugging
        curr_time = time.time()
        if(curr_time - prev_time >= 5):
            prev_time = curr_time

            # Show map
            plt.figure("Map")
            plt.cla()
            plt.imshow(occ_map, cmap='gray')
            plt.pause(0.01)

        # Sleep, if needed, to maintain the desired frequency
        rate.sleep()

    print('Quitting...')
    # Stop the robot before quitting
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    sys.exit(0)
