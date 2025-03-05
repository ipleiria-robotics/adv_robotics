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

# Library packages needed
from math import radians, degrees
import sys

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Advanced Robotics class utilities
import ar_py_utils.utils as utils

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = radians(90)  # [rad/s]


class BasicNavigation(Node):
    '''
    Random navigation avoiding laser-based detected obstacles.
    '''

    def __init__(self):
        '''
        Initializes the class instance.
        '''

        # Internal navigation variables
        self.rotating_right = False
        self.rotating_left = True
        self.stop_front_dist = 0.6
        self.min_front_dist = 0.8

        # Initialize the node itself
        super().__init__('tw02_navigation')

        # Setup subscriber for the laser scan
        self.create_subscription(LaserScan, 'base_scan',
                                 self.laserCallback, 1)

        # Setup publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def laserCallback(self, msg: LaserScan):
        ''' Update distance to closest obstacles and controll the robot
        bsed on those detected obstacles '''

        #######################################################################
        # STAR YOUR CODE HERE - OBSTACLE DETECTION

        ''' Right obstacle '''
        # closest_right_obstacle = ...

        '''Front obstacle'''
        angle = -0.785  # radians(-45)
        i = round((angle - msg.angle_min)/msg.angle_increment)
        closest_front_obstacle = msg.range_max
        while angle < 0.785:  # radians(45)
            if((msg.ranges[i] < msg.range_max) and
               (msg.ranges[i] > msg.range_min) and
               (msg.ranges[i] < closest_front_obstacle)):
                closest_front_obstacle = msg.ranges[i]
            i += 1
            angle += msg.angle_increment

        '''Left obstacle'''
        # closest_left_obstacle = ...

        # END YOUR CODE HERE
        #######################################################################

        #######################################################################
        # START YOUR CODE HERE, CHANGE THIS - NAVIGATION CONTROL
    
        lin_vel = 0.4  # Reference base linear velocity
        ang_vel = 0.   # Reference base angular velocity

        if closest_front_obstacle < self.stop_front_dist:
            lin_vel = 0.

        # END YOUR CODE HERE
        #######################################################################

        # Limit maximum velocities (should not be needed)
        lin_vel = utils.clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = utils.clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Show desired velocity
        utils.clearLine()
        utils.printxy(
            3, 0,
            f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
            f'{degrees(ang_vel):.2f} [°/s]\r')
        sys.stdout.flush()  # Force screen update

        # Send linear and angular velocity commands
        vel_cmd = Twist()
        vel_cmd.angular.z = ang_vel
        vel_cmd.linear.x = lin_vel
        self.vel_pub.publish(vel_cmd)


def main(args=None):
    '''
    Main function for tw02.
    '''

    # Clear screen
    utils.clearTerminal()

    # Output usage information
    utils.printxy(0, 0,
                  'Random navigation with obstacle avoidance.\n' +
                  '------------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    nav_node = BasicNavigation()

    # Get the node executing
    rclpy.spin(nav_node)

    # Cleanup memory and shutdown
    nav_node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
