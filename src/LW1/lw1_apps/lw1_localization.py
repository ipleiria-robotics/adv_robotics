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

# Matrix related functions
import numpy as np

# ROS API
import rospy
from geometry_msgs.msg import Pose2D

# Other libraries
import sys

if __name__ == '__main__':
    '''
    Main function
    Compute the robot pose and publish it for the navastar to use it.
    '''

    # Delete this and put your code here

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
    # r = np.linalg.inv(np.transpose(A) @ A) @np.transpose(A) @ b
    r = np.linalg.pinv(A) @ b

    # Debug code
    print('Matrix A:')
    print(A)
    print('Vector b:')
    print(b)
    print('Vector r:')
    print(r)

    # Main Code

    #
    # Create robot related objects
    #
    robot_name = '/robot_0'

    try:

        # Setup pose publisher
        pose_pub = rospy.Publisher(robot_name + '/pose', Pose2D, queue_size=1)

        # Init ROS
        rospy.init_node('lw01_localization', anonymous=True)

        # Loop rate
        rate = rospy.Rate(1)  # 1 Hz

        # Infinite loop
        while not rospy.is_shutdown():

            # Fill pose values using the localization results
            pose = Pose2D(1.0, 2.0, 3.0)

            # Publish estimateed pose
            pose_pub.publish(pose)

            # Sleep, if needed, to maintain the desired frequency
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    finally:
        # If we are quitting, stop the robot
        print('Quitting...')
        sys.exit(0)
