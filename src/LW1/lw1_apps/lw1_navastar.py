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
from math import degrees

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist

# Our functions
from utils import getKey

# Other libraries
import sys
import termios  # Terminal
import atexit

# Global variables
pose = Pose2D()  # Robot pose received from the localization module
end_program = False  # End infinite loop when True
# Terminal related
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


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


def on_key(event):
    global end_program
    '''Quit if the q key was pressed'''
    if(event.key == 'q'):
        end_program = True


def setNormalTerm():
    ''' Restore the terminal back to its original settings '''
    global old_settings
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)


if __name__ == '__main__':
    '''
    Main function
    Generate a path plan and execute it, given the robot estimated pose and the
    generated map
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
    vel_cmd = Twist()  # For velocity commands

    # Init ROS
    rospy.init_node('lw01_navastar')

    # Setup subscribers
    # Pose subscriber
    sub_pose = rospy.Subscriber(robot_name + '/pose', Pose2D,
                                poseCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Infinite loop
    while (not end_program) and (not rospy.is_shutdown()):
        # Check if the 'q' key was pressed
        nChar = getKey()
        if nChar == 'q':
            break  # Exit the infinite loop, and end the application

        # Sleep, if needed, to maintain the desired frequency
        rate.sleep()

    # If we are quitting, stop the robot
    print('Quitting...')
    # Stop the robot before quitting
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    sys.exit(0)
