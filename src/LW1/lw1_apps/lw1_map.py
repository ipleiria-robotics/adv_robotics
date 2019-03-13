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

# ROS API
import rospy
from geometry_msgs.msg import Pose2D

# Other libraries
from math import degrees
import sys

# Global variables
pose = Pose2D()


def poseCallback(msg: Pose2D):
    ''' Function to call whe new odometry information is available '''

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


if __name__ == '__main__':
    '''
    Main function
    Generate a map using the laser and the estimated robot pose.
    '''

    # Put your code here

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'

    try:
        # Pose subscriber
        sub_pose = rospy.Subscriber(robot_name + '/pose', Pose2D,
                                    poseCallback, queue_size=1)

        # Init ROS
        rospy.init_node('lw01_map', anonymous=True)

        # Loop rate
        rate = rospy.Rate(10)  # 10 Hz

        # Infinite loop
        while not rospy.is_shutdown():
            # Sleep, if needed, to maintain the desired frequency
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        print('Quitting...')
        sys.exit(0)
