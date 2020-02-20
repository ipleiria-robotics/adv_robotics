#!/usr/bin/env python3
'''
Copyright (c) 2020, Hugo Costelha
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
from math import pi, atan2
import curses

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Global variables
true_pose = Pose2D()
true_lin_vel = 0.0
true_ang_vel = 0.0
odom_updated = False


def clipValue(value: float, min: float, max: float) -> float:
    '''Clip a given value to the interval [min, max]'''
    if value > max:
        return max
    elif value < min:
        return min
    else:
        return value


def quaternionToYaw(q):
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf
    doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return atan2(t0, t1)


def odomCallback(data):
    '''Function to call whe new odometry information is available'''
    global true_pose, true_ang_vel, true_lin_vel, odom_updated
    # Store updated values
    true_pose.x = data.pose.pose.position.x
    true_pose.y = data.pose.pose.position.y
    true_pose.theta = quaternionToYaw(data.pose.pose.orientation)

    true_lin_vel = data.twist.twist.linear.x
    true_ang_vel = data.twist.twist.angular.z

    odom_updated = True


def main(stdscr):
    '''
    Main function
    Controls the robot using the keyboard keys and outputs posture and velocity
    related information.
    '''
    global MAX_LIN_VEL, MAX_ANG_VEL, true_pose, true_lin_vel, true_ang_vel,\
        odom_updated

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
    l_scale = 0.0
    a_scale = 0.0
    # Velocity increase for each keystroke
    delta_lin_vel = 0.1
    delta_ang_vel = 5.0/180.0*3.14
    vel_cmd = Twist()

    # Output usage information
    stdscr.addstr(
        0, 0,
        'Reading from keyboard\n' +
        'Use UP, LEFT, DOWN, RIGHT and space to move front, left, back,' +
        ' right and stop, respectively.\nPress q to quit.\n' +
        '---------------------------')

    # Get parameters
    a_scale = rospy.get_param("~scale_angular", 1.0)
    l_scale = rospy.get_param("~scale_linear", 1.0)

    # Setup subscriber
    rospy.Subscriber('/robot_0/odom', Odometry, odomCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=1)

    # Init ROS
    rospy.init_node('robot_keyboard_teleop', anonymous=True)

    # Infinite loop
    rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
    while not rospy.is_shutdown():
        # If there are no new values, sleep
        if odom_updated is False:
            rate.sleep()
            continue

        odom_updated = False

        # Get data from the robot and print it
        # We are using the curses addstr function in order to write in a
        # specific screen position, otherwise we could use "print"
        stdscr.addstr(
            4, 0,
            f'Robot estimated pose = {true_pose.x:.2f} [m], ' +
            f'{true_pose.y:.2f} [m], ' +
            f'{true_pose.theta*180.0/pi:.2f} [º]')

        stdscr.addstr(
            5, 0,
            f'Robot estimated velocity = {true_lin_vel:.2f} [m/s], '    
            f'{true_ang_vel*180.0/pi:.2f} [º/s]')

        # Get char, and act accordingly
        nChar = stdscr.getch()
        if nChar == curses.ERR:
            # Decelerate automatically if no key was pressed
            lin_vel *= 0.9
            ang_vel *= 0.9
        elif nChar == ord('q'):
            break
        elif nChar == curses.KEY_UP:
            # Increase linear velocity
            lin_vel += delta_lin_vel
        elif nChar == curses.KEY_DOWN:
            # Decrease linear velocity
            lin_vel -= delta_lin_vel
        elif nChar == curses.KEY_LEFT:
            # Increase angular velocity
            ang_vel += delta_ang_vel
        elif nChar == curses.KEY_RIGHT:
            # Decrease angular velocity
            ang_vel -= delta_ang_vel
        elif nChar == ord(' '):
            # Stop robot
            lin_vel = 0
            ang_vel = 0

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Show desired velocity
        stdscr.addstr(
            6, 0,
            f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
            f'{ang_vel*180.0/pi:.2f} [º/s]')
        stdscr.refresh()  # Update screen

        # Send velocity commands
        vel_cmd.angular.z = a_scale*ang_vel
        vel_cmd.linear.x = l_scale*lin_vel
        vel_pub.publish(vel_cmd)

'''
This is what is actually called when we run this python script. It then calls
the main function defined above, but using the curses wrapper in order to
control the terminal
'''
if __name__ == '__main__':
    curses.wrapper(main)