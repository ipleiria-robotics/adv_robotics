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
import os
import tty
import termios
import sys
import fcntl
from math import pi, atan2
from threading import Lock

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64, UInt8MultiArray

# Our ROS-related modules
from markers_msgs.msg import Markers

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Global variables
true_pose = Pose2D()
true_lin_vel = 0.0
true_ang_vel = 0.0
odom_updated = False

# Forklift information
forklift_position = -1.0
forklift_up = False
forklift_down = False
FORKLIFT_DOWN = 0.0  # Down position
FORKLIFT_UP = 0.07  # Up position

# Parts information
parts_status = []

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


def forkliftCallback(msg):
    ''' Process forklift related messages '''
    global forklift_position, forklift_up, forklift_down
    global FORKLIFT_DOWN, FORKLIFT_UP
    forklift_position = msg.process_value
    # If the forklift is moving, then it is neither down or up
    if abs(msg.error) > 0.02:
        forklift_up = False
        forklift_down = False
    else:
        # The forklift is almost stopped, lets check in which position
        if abs(forklift_position-FORKLIFT_DOWN) < 0.01:
            # It's down
            if forklift_down is False:
                print('Forklift is down')
                forklift_up = False
                forklift_down = True
        elif abs(forklift_position-FORKLIFT_UP) < 0.01:
            # It's up
            if forklift_up is False:
                forklift_up = True
                forklift_down = False
                print('Forklift is up')


def partsSensorCallback(msg):
    ''' Process the parts sensor data '''
    global parts_status
    parts_status = [x for x in msg.data]


def markersCallback(msg: Markers):
    ''' Process received markers data from parts/locations'''
    # Display found parts information
    print(f'Found {msg.num_markers} parts/locations.')
    for i in range(msg.num_markers):
        print(f'Part/Location {msg.id[i]:.2f} : (range, bearing) = (' +
              f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})')
    
'''
Main function
Controls the robot using the keyboard keys and outputs posture and velocity
related information.
'''
if __name__ == '__main__':
    # Terminal settings
    fd = sys.stdin.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

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
    forklift_pos_cmd = Float64(0)
    robot_name = '/robot_0'

    # Output usage information
    print('Reading from keyboard\n' +
            'Use i, j, k, l and space to move front, left, back, right and' +
            ' stop, respectively.\n' +
            'Use e, d to move the forklift up and down\n' +
            'Press q to quit.\n' +
            '---------------------------\n\r')

    # Get parameters
    a_scale = rospy.get_param("~scale_angular", 1.0)
    l_scale = rospy.get_param("~scale_linear", 1.0)

    # Setup subscriber for odometry
    sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry,
                                odomCallback, queue_size=1)

    # Setup publisher for speed control
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Setup subscriber for the forklift status
    sub_forklift = rospy.Subscriber(
        robot_name + '/forklift_position_controller/state',
        JointControllerState, forkliftCallback, queue_size=1)

    # Setup publishers for the forklift commands
    forklift_pub = rospy.Publisher(
        robot_name + '/forklift_position_controller/command',
        Float64, queue_size=1)

    # Setup subscriber for the parts sensor
    sub_parts_status = rospy.Subscriber('/parts_sensor', UInt8MultiArray,
                                        partsSensorCallback, queue_size=1)   

    # Setup subscriber for parts detection (simulated with markers)
    sub_parts_loc = rospy.Subscriber(robot_name + '/markers', Markers,
                                     markersCallback, queue_size=1)

    # Init ROS
    rospy.init_node('robot_keyboard_teleop', anonymous=True)

    # Terminal
    tty.setraw(sys.stdin.fileno())

    # Infinite loop
    rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
    while not rospy.is_shutdown():
        # If there are not new values, sleep
        if odom_updated is False:
            rate.sleep()
            continue

        key_pressed = False
        odom_updated = False

        # Get data from the robot and print it
        print(f'Robot estimated pose = {true_pose.x:.2f} [m], ' +
                f'{true_pose.y:.2f} [m], ' +
                f'{true_pose.theta*180.0/pi:.2f} [º]\r')

        # Show estimated velocity
        print(f'Robot estimated velocity = {true_lin_vel:.2f} [m/s], '
                f'{true_ang_vel*180.0/pi:.2f} [º/s]\r')

        # Get char
        nChar = sys.stdin.read(1)

        if not nChar:
            # Decelerate automatically if no key was pressed
            lin_vel *= 0.9
            ang_vel *= 0.9
        elif nChar == 'q':
            break
        elif nChar == 'i':
            # Increase linear velocity
            lin_vel += delta_lin_vel
            key_pressed = True
        elif nChar == 'k':
            # Decrease linear velocity
            lin_vel -= delta_lin_vel
            key_pressed = True
        elif nChar == 'j':
            # Increase angular velocity
            ang_vel += delta_ang_vel
            key_pressed = True
        elif nChar == 'l':
            # Decrease angular velocity
            ang_vel -= delta_ang_vel
            key_pressed = True
        elif nChar == ' ':
            # Stop robot
            lin_vel = 0
            ang_vel = 0
        elif nChar == 'e':
            # Send forklit up
            forklift_pos_cmd.data = FORKLIFT_UP
            forklift_pub.publish(forklift_pos_cmd)
        elif nChar == 'd':
            # Send forklit up
            forklift_pos_cmd.data = FORKLIFT_DOWN
            forklift_pub.publish(forklift_pos_cmd)


        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Show desired velocity
        print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
                f'{ang_vel*180.0/pi:.2f} [º/s]\r')

        # Show forklift position
        print(f'Forklift position = {forklift_position:.2f} [m]\r')

        # Show parts status
        print('Parts status: ' + parts_status.__str__() + '\r', flush=True)

        # Send velocity commands
        vel_cmd.angular.z = a_scale*ang_vel
        vel_cmd.linear.x = l_scale*lin_vel
        vel_pub.publish(vel_cmd)

        # Limit the amount of messages when no key is pressed
        if key_pressed is False:
            rate.sleep()

        # Move cursor back up n lines (and erase them)
        for n in range(0, 5):
            print('\033[1A', end="")
            print('\033[2K', end="")

    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
