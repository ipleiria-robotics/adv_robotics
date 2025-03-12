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
import numpy as np
import time
import random

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

# Our utility functions (using the ones from tw02)
import ar_py_utils.utils as utils

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
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
        super().__init__('tw03_navigation')

        # Parameters
        # Maximum linear velocity
        ref_lin_vel_param_desc = \
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='Reference linear velocity!')
        self.declare_parameter('ref_lin_vel', 0.2,  ref_lin_vel_param_desc)
        # Maximum angular velocity
        ref_ang_vel_param_desc = \
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='reference angular velocity!')
        self.declare_parameter('ref_ang_vel', radians(30),
                               ref_ang_vel_param_desc)

        # Setup subscriber for the laser scan
        self.create_subscription(LaserScan, 'base_scan',
                                 self.laser_cb, 1)

        # Setup publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def laser_cb(self, msg: LaserScan):
        ''' Update distance to closest obstacles and controll the robot
        bsed on those detected obstacles '''

        #######################################################################
        # OBSTACLE DETECTION

        ''' Right obstacle '''
        # Find index of -90 degrees
        min_angle_idx = \
            round((radians(-90) - msg.angle_min)/msg.angle_increment)
        # Find index of -75 degrees
        max_angle_idx = \
            round((radians(-75) - msg.angle_min)/msg.angle_increment)
        # Get smaller value in the given range
        closest_right_obstacle = \
            np.min(msg.ranges[min_angle_idx:max_angle_idx+1])

        '''Front obstacle'''
        # Find index of -45 degrees
        min_angle_idx = \
            round((radians(-45) - msg.angle_min)/msg.angle_increment)
        # Find index of 45 degrees
        max_angle_idx = \
            round((radians(45) - msg.angle_min)/msg.angle_increment)
        # Get smaller value in the given range
        closest_front_obstacle = \
            np.min(msg.ranges[min_angle_idx:max_angle_idx+1])

        '''Left obstacle'''
        # Find index of 75 degrees
        min_angle_idx = \
            round((radians(75) - msg.angle_min)/msg.angle_increment)
        # Find index of 90 degrees
        max_angle_idx = \
            round((radians(90) - msg.angle_min)/msg.angle_increment)
        # Get smaller value in the given range
        closest_left_obstacle = \
            np.min(msg.ranges[min_angle_idx:max_angle_idx+1])

        #######################################################################

        #######################################################################
        # NAVIGATION CONTROL

        # Get maximum linear and angular parameters
        ref_lin_vel = self.get_parameter('ref_lin_vel'
                                         ).get_parameter_value().double_value
        ref_ang_vel = self.get_parameter('ref_ang_vel'
                                         ).get_parameter_value().double_value

        avoid = False
        lin_vel = ref_lin_vel  # Reference base linear velocity
        ang_vel = 0.   # Reference base angular velocity

        if closest_front_obstacle < self.stop_front_dist:
            avoid = True
            lin_vel = -ref_lin_vel/4.0
        elif closest_front_obstacle < self.min_front_dist:
            avoid = True
            lin_vel = 0.

        if avoid:

            if self.rotating_left:
                ang_vel = ref_ang_vel
            elif self.rotating_right:
                ang_vel = -ref_ang_vel
            else:
                # To add some variability to the process, sometimes we will
                # rotate towards the closest obstacle (10% of the times). We
                # will do that by generating a random number between 0 and 1
                # and, if the number is <= 0.1, we will switch direction.
                x = random.random()  # Random number between 0 and 1
                if x <= 0.1:
                    use_other_rot_direction = True
                else:
                    use_other_rot_direction = False
                # Check where is the closest obstacle and decide.
                if ((closest_left_obstacle < closest_right_obstacle) and
                        (use_other_rot_direction is False)) or \
                   ((closest_left_obstacle >= closest_right_obstacle) and
                        (use_other_rot_direction is True)):
                    self.rotating_left = False
                    self.rotating_right = True
                    ang_vel = -ref_ang_vel
                else:
                    self.rotating_left = True
                    self.rotating_right = False
                    ang_vel = ref_ang_vel
        else:
            self.rotating_left = False
            self.rotating_right = False

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
    time.sleep(5)  # Wait 5 seconds before starting to move
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
