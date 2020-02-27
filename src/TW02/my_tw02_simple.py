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
from math import radians, degrees
import curses
import sys

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
true_pose = Pose2D()  # Store real (error-free) robot pose
true_lin_vel = 0.0  # Store real (error-free) linear velocity
true_ang_vel = 0.0  # Store real (error-free) angular velocity
odom_updated = False  # True if we got an odometry update
laser_updated = False  # True if we got a laser update
closest_front_obstacle = 0.0  # Distance to closest front obstacle (m)
closest_left_obstacle = 0.0  # Distance to closest left obstacle (m)
closest_right_obstacle = 0.0  # Distance to closest right obstacle (m)


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

    global laser_updated, closest_right_obstacle, closest_front_obstacle,\
        closest_left_obstacle

    ###########################################################################
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


def main(stdscr):
    '''
    Main function
    Controls the robot using the keyboard keys and outputs posture and velocity
    related information.
    '''
    global laser_updated, MAX_LIN_VEL, MAX_ANG_VEL, true_pose, true_lin_vel,\
        true_ang_vel, closest_left_obstacle, closest_front_obstacle,\
        closest_right_obstacle

    # Terminal settings
    # Check https://docs.python.org/3.6/howto/curses.html for more info
    stdscr.clear()  # Clear screen
    stdscr.nodelay(True)  # Do not block when getting a key

    #
    # Create robot related objects
    #
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    vel_cmd = Twist()

    # Navigation variables
    rotating_right = False
    rotating_left = True
    stop_front_dist = 0.6
    min_front_dist = 0.8

    # Output usage information
    stdscr.addstr(0, 0,
                  'Random navigation with obstacle avoidance.\n' +
                  'Press "q" to quit.\n' +
                  '---------------------------\n')

    # Setup subscribers
    # Odometry
    rospy.Subscriber('/robot_0/odom', Odometry, odomCallback, queue_size=1)
    rospy.Subscriber('/robot_0/base_scan', LaserScan, laserCallback,
                     queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

    # Init ROS
    rospy.init_node('tw02', anonymous=True)

    # Infinite loop
    rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
    while not rospy.is_shutdown():
        # If the 'q' key is pressed, end the application
        if stdscr.getch() == ord('q'):
            break

        # If there are not new values, sleep
        if laser_updated is False:
            rate.sleep()
            continue
        laser_updated = False

        # Show pose estimated from odometry
        stdscr.addstr(
            3, 0,
            f'Robot estimated pose = {true_pose.x:.2f} [m], ' +
            f'{true_pose.y:.2f} [m], ' +
            f'{degrees(true_pose.theta):.2f} [º]\r')

        # Show estimated velocity
        stdscr.addstr(
            4, 0,
            f'Robot estimated velocity = {true_lin_vel:.2f} [m/s], '
            f'{degrees(true_ang_vel):.2f} [º/s]\r')

        ################################################################
        avoid = False
        lin_vel = 0.4
        ang_vel = 0

        if closest_front_obstacle < stop_front_dist:
            avoid = True
            lin_vel = -0.100
        elif closest_front_obstacle < min_front_dist:
            avoid = True
            lin_vel = 0

        if avoid:
            if rotating_left:
                ang_vel = radians(30)
            elif rotating_right:
                ang_vel = radians(-30)
            elif closest_left_obstacle < closest_right_obstacle:
                rotating_left = False
                rotating_right = True
                ang_vel = radians(-30)
            else:
                rotating_left = True
                rotating_right = False
                ang_vel = radians(30)
        else:
            rotating_left = False
            rotating_right = False
        ################################################################

        # Limit maximum velocities (should not be needed)
        lin_vel = utils.clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = utils.clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Show desired velocity
        stdscr.addstr(
            5, 0,
            f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
            f'{degrees(ang_vel):.2f} [º/s]\r')
        stdscr.refresh()  # Update screen

        # Send velocity commands
        vel_cmd.angular.z = ang_vel
        vel_cmd.linear.x = lin_vel
        vel_pub.publish(vel_cmd)

        # Sleep, if needed, to maintain the dseired frequency
        rate.sleep()

    # Ask the robot to stop before quitting
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)


'''
This is what is actually called when we run this python script. It then calls
the main function defined above, but using the curses wrapper in order to
control the terminal
'''
if __name__ == '__main__':
    curses.wrapper(main)
    print('Quitting...')
    sys.exit(0)
