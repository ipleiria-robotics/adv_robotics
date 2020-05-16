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

'''@package docstring
Teleoperation application for the 2nd Advanced Robotics project.
'''

# Library packages needed
import atexit  # Terminal
import termios  # Terminal
from select import select  # Terminal
import sys
from math import atan2

# ROS API
import rospy
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64, UInt8MultiArray, UInt8

# Our ROS-related modules
from markers_msgs.msg import Markers
import lw2_sim.srv

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Forklift information
forklift_position = -1.0
forklift_up = False
forklift_down = False
FORKLIFT_DOWN = 0.0  # Down position
FORKLIFT_UP = 0.07  # Up position

# Parts information
parts_status = []

# Battery level (assume 100% for starters)
battery_level = 100

# Terminal related
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


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


def batteryLevelCallback(msg):
    ''' Process battery level related messages'''
    global battery_level
    # If the battery as dropped below 5%
    if (msg.data < 5) and (battery_level >= 5):
        print('Battery is getting extremely low...')
    elif (msg.data < 20) and (battery_level >= 20):
        print('Battery is getting low...')
    # Store the current value
    battery_level = msg.data


def forkliftCallback(msg):
    ''' Process forklift related messages '''
    global forklift_position, forklift_up, forklift_down
    global FORKLIFT_DOWN, FORKLIFT_UP
    forklift_position = msg.process_value

    # Show forklift position
    print(f'Forklift position = {forklift_position:.3f} [m]')

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

    # Show parts status
    print('Parts status: ' + parts_status.__str__())


def markersCallback(msg: Markers):
    ''' Process received markers data from parts/locations'''
    # Display found parts information
    if msg.num_markers > 0:
        print(f'Found {msg.num_markers} parts/locations.')
        for i in range(msg.num_markers):
            print(f'Part/Location seen at : (range, bearing) = (' +
                  f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})')


def setNormalTerm():
    ''' Set the terminal back to its original settings '''
    global old_settings
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)


'''
Main function
Controls the robot using the keyboard keys and outputs posture and velocity
related information.
'''
if __name__ == '__main__':
    # Terminal settings
    new_settings = termios.tcgetattr(fd)
    new_settings[3] = (new_settings[3] & ~termios.ICANON & ~termios.ECHO)
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_settings)
    atexit.register(setNormalTerm)

    #
    # Create robot related objects
    #
    # Linear and angular velocities for the robot (initially stopped)
    lin_vel = 0.0
    ang_vel = 0.0
    l_scale = 1.0
    a_scale = 1.0
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
          'Use e and d to move the forklift up and down, respectively\n' +
          'Use + and - to move start and stop the charging, respectively\n' +
          'Press q to quit.\n' +
          '---------------------------\n')

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
    sub_parts_status = rospy.Subscriber(robot_name + '/parts_sensor', UInt8MultiArray,
                                        partsSensorCallback, queue_size=1)

    # Setup subscriber for parts detection (simulated with markers)
    sub_parts_loc = rospy.Subscriber(robot_name + '/markers', Markers,
                                     markersCallback, queue_size=1)

    # Setuo subscriber for the battery level
    sub_batt_level = rospy.Subscriber(robot_name + '/battery/level', UInt8,
                                      batteryLevelCallback, queue_size=1)

    # Battery charging service
    charge_battery = rospy.ServiceProxy(robot_name + '/battery/charge',
                                        lw2_sim.srv.StartCharging)

    # Init ROS
    rospy.init_node('robot_keyboard_teleop')

    # Infinite loop
    rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
    while not rospy.is_shutdown():
        # Check if a key was pressed
        dr, dw, de = select([sys.stdin], [], [], 0)
        key_pressed = (dr != [])

        if not key_pressed:
            # Decelerate automatically if no key was pressed
            lin_vel *= 0.9
            ang_vel *= 0.9
        else:
            # Get char
            nChar = sys.stdin.read(1)
            if nChar == 'q':
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
            elif nChar == '+':
                # Request charging
                resp = charge_battery(True)
                if resp.charging is False:
                    print('Unable to start charging. Check the robot position!')
                else:
                    print('Robot is now charging.')
            elif nChar == '-':
                # Request charging to stop
                resp = charge_battery(False)
                if resp.charging is False:
                    print('Robot has stopped charging.')
                else:
                    print('Unable to stop charging.')   

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Show desired velocity
        # print(f'Robot desired velocity = {lin_vel:.2f} [m/s], ' +
        #       f'{ang_vel*180.0/pi:.2f} [°/s]\n')

        # Send velocity commands
        vel_cmd.angular.z = a_scale*ang_vel
        vel_cmd.linear.x = l_scale*lin_vel
        vel_pub.publish(vel_cmd)

        # Limit the amount of messages when no key is pressed
        if key_pressed is False:
            rate.sleep()

    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    setNormalTerm()
