#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
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

# ROS
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Python
from math import pi, atan2
import sys


def printxy(x, y, text):
    '''Print in a specified screen position'''
    sys.stdout.write('\x1b[%d;%df%s' % (x, y, text))


def quaternionToYaw(q):
    '''Returns the yaw in radians of a quaternion.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return atan2(t0, t1)


def odomCallback(msg):
    '''Function to call whe new odometry information is available'''

    # Clear the screen. Check these and other key sequences in:
    #   http://ascii-table.com/ansi-escape-sequences.php
    sys.stdout.write('\x1b[2J')
    printxy(
        1, 0,
        f'Robot estimated pose = {msg.pose.pose.position.x:.2f} [m], ' +
        f'{msg.pose.pose.position.y:.2f} [m], ' +
        f'{quaternionToYaw(msg.pose.pose.orientation)*180.0/pi:.2f} [°]')

    printxy(
        2, 0,
        f'Robot estimated velocity = {msg.twist.twist.linear.x:.2f} [m/s], '
        f'{msg.twist.twist.angular.z*180.0/pi:.2f} [°/s]')
    sys.stdout.flush()  # Make sure the information is printed now


def main(args=None):
    '''Main function. Associate a callback to a topic. Print the root position
       and orientation in that callback.
    '''
    # Initiate python node
    rclpy.init(args=args)
    # Output some information to the screen top left
    printxy(
        0, 0,
        'Running tw01 - print robot pose and velocity...')

    node = Node('tw01_subscriber')
    node.create_subscription(Odometry, '/robot_0/odom',
                             odomCallback, 5)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # We use the curses wrapper to make it easier to control the screen output
    main()
