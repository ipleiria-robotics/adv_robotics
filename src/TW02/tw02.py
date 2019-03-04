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

# Library packages needed
from math import pi, atan2, radians, degrees

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 

# Our functions
import utils

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Global variables
true_pose = Pose2D() # Store real (error-free) robot pose
true_lin_vel = 0.0 # Store real (error-free) linear velocity
true_ang_vel = 0.0 # Store real (error-free) angular velocity
odom_updated = False # True if we got an odometry update
laser_updated = False # True if we got a laser update
closest_front_obstacle = 0.0 # Distance to closest front obstacle (m)
closest_left_obstacle = 0.0 # Distance to closest left obstacle (m)
closest_right_obstacle = 0.0 # Distance to closest right obstacle (m)


def odomCallback(data: Odometry):
    ''' Function to call whe new odometry information is available '''

    global true_pose, true_ang_vel, true_lin_vel, odom_updated

    # Store updated values
    true_pose.x = data.pose.pose.position.x
    true_pose.y = data.pose.pose.position.y
    true_pose.theta = utils.quaternionToYaw(data.pose.pose.orientation)

    true_lin_vel = data.twist.twist.linear.x
    true_ang_vel = data.twist.twist.angular.z

    odom_updated = True


def laserCallback(msg: LaserScan):
  ''' Update distance to closest obstacles '''

  global laser_updated

  ###########################################################################
  # STAR YOUR CODE HERE
  # closest_right_obstacle = ...
  # closest_left_obstacle = ...
  # closest_left_obstacle = ...


  # END YOUR CODE HERE
  ###########################################################################

  laser_updated = True


'''
Main function
Controls the robot using the keyboard keys and outputs posture and velocity
related information.
'''
if __name__ == '__main__':


    #
    # Create robot related objects
    #
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    vel_cmd = Twist()


    try:
        # Output usage information
        print('Random navigation with obstacle avoidance.\n' +
              '---------------------------\n')

        # Setup subscribers
        # Odometry
        sub_odom = rospy.Subscriber('/robot_0/odom', Odometry, odomCallback,
                                    queue_size=1)
        sub_laser = rospy.Subscriber('/robot_0/base_scan', LaserScan, laserCallback,
                                     queue_size=1)

        # Setup publisher
        vel_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

        # Init ROS
        rospy.init_node('tw02', anonymous=True)

        # Infinite loop
        rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
        while not rospy.is_shutdown():
            # If there are not new values, sleep
            if laser_updated is False:
                continue
            laser_updated = False

            # Show pose estimated from odometry
            print(f'Robot estimated pose = {true_pose.x:.2f} [m], ' +
                  f'{true_pose.y:.2f} [m], ' +
                  f'{degrees(true_pose.theta):.2f} [º]\r')

            # Show estimated velocity
            print(f'Robot estimated velocity = {true_lin_vel:.2f} [m/s], '
                  f'{degrees(true_ang_vel):.2f} [º/s]\r')

            ################################################################
            # START YOUR CODE HERE
            # Change lin_vel and ang_vel so as to avoid obstacles
            # lin_vel = ...
            # ang_vel = ...


            # END YOUR CODE HERE
            ################################################################

            # Limit maximum velocities (should not be needed)
            #lin_vel = utils.clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
            #ang_vel = utils.clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

            # Show desired velocity
            print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
                  f'{degrees(ang_vel):.2f} [º/s]\r', flush=True)

            # Send velocity commands
            vel_cmd.angular.z = ang_vel
            vel_cmd.linear.x = lin_vel
            vel_pub.publish(vel_cmd)

            # Sleep, if needed, to maintain the dseired frequency
            rate.sleep()

            # Move cursor back up n lines (and erase them)
            for n in range(0, 3):
                print('\033[1A', end="")
                print('\033[2K', end="")

    except rospy.ROSInterruptException:
        pass

    finally:
        # If we are quitting, stop the robot
        vel_cmd.angular.z = 0
        vel_cmd.linear.x = 0
        vel_pub.publish(vel_cmd)
