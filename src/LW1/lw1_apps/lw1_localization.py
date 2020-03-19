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

# ROS API
import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

# Other libraries
import sys
import termios  # Terminal
import atexit

# Our functions
from utils import quaternionToYaw, getKey
from markers_msgs.msg import Markers

# Global variables
robot_real_pose = Pose2D()  # Store real (error-free) robot pose
robot_estimated_pose = Pose2D()  # Store estimated robot pose
real_pose_updated = False  # True if the real pose was updated
# Terminal related
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


def poseGndTruthCallback(data: Odometry):
    ''' Function which is called when new (real) pose information is
    available. This should only be used for debugging purposes, in
    order to verify if your estimated robot pose is correct.
    '''

    global robot_real_pose, real_pose_updated

    # Store updated values
    robot_real_pose.x = data.pose.pose.position.x
    robot_real_pose.y = data.pose.pose.position.y
    robot_real_pose.theta = quaternionToYaw(data.pose.pose.orientation)

    # Print real pose
    print(f'Real robot pose (X, Y, Theta)= {robot_real_pose.x:.2f} [m]' +
          f', {robot_real_pose.y:.2f} [m], ' +
          f'{degrees(robot_real_pose.theta):.2f} [º]\r')

    real_pose_updated = True


def markersCallback(msg: Markers, pose_pub: rospy.Publisher):
    ''' Process received markers data '''

    global robot_estimated_pose

    # Display found beacons information
    print(f'Found {msg.num_markers} beacons.')
    for i in range(msg.num_markers):
        print(f'Beacon {msg.id[i]:.2f} : (range, bearing) = (' +
              f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})')

    ###
    # CHANGE THE CODE BELOW
    ###

    #
    # Change/use this code to your needs
    #

    # A matrix, 4 x 2
    A = np.zeros((4, 2), np.float)

    # b matrix, 4 x 1 (columm vector)
    b = np.zeros((4, 1), np.float)

    # Store some test values in A
    A[0, 0] = 1
    A[0, 1] = 2
    A[1, 0] = -1
    A[1, 1] = 2
    A[2, 0] = 1
    A[2, 1] = -2
    A[3, 0] = 2
    A[3, 1] = 2.5

    # Store some test values in b
    b[0] = 1
    b[1] = 2
    b[2] = -1
    b[3] = 1.5

    # Compute r = inv(A'*A)*A'*b, where inv(X) = X^(-1) and @ is used for
    # matrix multiplication (using '*' would result in element-wise
    # multiplication, which is not what we want)
    # r = np.linalg.inv(np.T(A) @ A) @ np.T(A) @ b
    r = np.linalg.pinv(A) @ b

    # Debug code
    print('Matrix A:')
    print(A)
    print('Vector b:')
    print(b)
    print('Vector r:')
    print(r)

    # ^-- The code above is here just to show the minimization procedure
    # implementation described in the lab work guide.

    # Currently, we are publishing the ground-truth pose for testing purposes.
    # You need to replace this by your algorithm to compute the robot pose.

    # Fill pose values using the localization results
    robot_estimated_pose = Pose2D(robot_real_pose.x, robot_real_pose.y,
                                  robot_real_pose.theta)

    # Publish estimated pose
    pose_pub.publish(robot_estimated_pose)


def setNormalTerm():
    ''' Restore the terminal back to its original settings '''
    global old_settings
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)


if __name__ == '__main__':
    '''
    Main function
    Compute the robot pose and publish it for the navastar to use it.
    '''

    # Change terminal settings so that we can query the keyboard with no echo
    new_settings = termios.tcgetattr(fd)
    new_settings[3] = (new_settings[3] & ~termios.ICANON & ~termios.ECHO)
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_settings)
    # Make sure the setNormalTerm function is called on exit, in order to
    # restore the terminal settings
    atexit.register(setNormalTerm)

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('lw01_localization')

    # Setup pose publisher
    # 2D robot estimated pose
    pose_pub = rospy.Publisher(robot_name + '/pose', Pose2D, queue_size=1)

    # Subscribe to the real robot pose (Ground truth), for debugging purposes
    rospy.Subscriber(robot_name + '/base_pose_ground_truth', Odometry,
                     poseGndTruthCallback, queue_size=1)
    # Markers
    # Here we are passing the pose publisher (pose_pub) as an argument, so
    # that it can be used in the markersCallback
    rospy.Subscriber(robot_name + '/markers', Markers, markersCallback,
                     pose_pub, queue_size=1)

    # Loop rate
    rate = rospy.Rate(1)  # 1 Hz

    # Infinite loop
    while not rospy.is_shutdown():
        # Check if the 'q' key was pressed
        nChar = getKey()
        if nChar == 'q':
            break  # Exit the infinite loop, and end the application

        # Sleep, if needed, to maintain the desired frequency
        rate.sleep()

    # Quitting...
    print('Quitting...')
    sys.exit(0)
