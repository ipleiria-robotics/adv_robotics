#!/usr/bin/env python3
# Copyright (c) 2019, Hugo Costelha
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of the Player Project nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# ROS API
import time
import os
import tty
import termios
import sys
import fcntl
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion

from math import pi, atan2

# include <ros/ros.h>
# include <geometry_msgs/Twist.h> // Velocity messages
# include <geometry_msgs/Pose2D.h> // Velocity messages
# include <nav_msgs/Odometry.h> // Odometry messages
# include <tf/tf.h> // Geometry transformations

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = 1.14  # 90º/s (in rad/s)

# Global variables
true_pose = Pose2D()
true_lin_vel = 0.0
true_ang_vel = 0.0
odom_updated = False

#
# Checks if there a key was pressed, and returns a positive number if true.
#
# int checkForKey()
# {
#   struct timeval tv = { 0L, 0L };
#   fd_set fds;
#   FD_SET(0, &fds);
#   return select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
# }

def clipValue(value: float, min: float, max: float) -> float:
  if value > max:
    return max
  elif value < min:
    return min
  else:
   return value

def quaternionToYaw(q):
    '''Returns the yaw in radians of a quaternion.
    Reimplements part of euler_from_quaternion from the tf package because tf doesn't play well in Python 3.
    '''
    t0 = 2.0 * (q.w * q.z + q.x * q.y)
    t1 = 1.0 - 2.0 * (q.y**2 + q.z**2)
    return atan2(t0, t1)

def odomCallback(data):
  global true_pose, true_ang_vel, true_lin_vel, odom_updated
  # Store updated values
  true_pose.x = data.pose.pose.position.x
  true_pose.y = data.pose.pose.position.y
  
  #(_,_,yaw) = euler_from_quaternion(data.pose.pose.orientation)
  true_pose.theta = quaternionToYaw(data.pose.pose.orientation)

  true_lin_vel = data.twist.twist.linear.x
  true_ang_vel = data.twist.twist.angular.z

  odom_updated = True


#
# Main function
# Controls the robot using the keyboard keys and outputs posture and velocity
# related information.
#
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

  try:
    # Setup subscriber
    sub_odom = rospy.Subscriber('odom', Odometry, odomCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Init ROS
    rospy.init_node('robot_keyboard_teleop', anonymous=True)

    # Get parameters
    a_scale = rospy.get_param("~scale_angular", 1.0)
    l_scale = rospy.get_param("~scale_linear", 1.0)

    # Output usage information
    print('Reading from keyboard\n' +
          'Use i, j, k, l and space to move front, left, back, right and stop, respectively\n' +
          'Press q to quit.\n' +
          '---------------------------\n')

    # Terminal
    tty.setraw(sys.stdin.fileno())

    # Infinite loop
    rate = rospy.Rate(10) # 10 Hz, Rate when no key is being pressed

    while not rospy.is_shutdown():
      # If there are not new values, sleep 
      if odom_updated == False:
        rate.sleep()
        continue

      key_pressed = False
      odom_updated = False

      # Get data from the robot and print it if available
      #rospy.spin()
      # show pose estimated from odometry
      # print(std::setiosflags(std::ios::fixed) << std::setprecision(3)
      print(f'Robot estimated pose = {true_pose.x:.2f} [m], {true_pose.y:.2f} [m], {true_pose.theta*180.0/pi:.2f} [º]\r')

      # Show estimated velocity
      print(f'Robot estimated velocity = {true_lin_vel:.2f} [m/s], {true_ang_vel*180.0/pi:.2f} [º/s]\r')

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
      
      # Limit maximum velocities
      lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
      ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

      # Show desired velocity
      print(f'Robot desired velocity = {lin_vel:.2f} [m/s], {ang_vel*180.0/pi:.2f} [º/s]\r', flush=True)

      # Send velocity commands
      vel_cmd.angular.z = a_scale*ang_vel
      vel_cmd.linear.x = l_scale*lin_vel
      vel_pub.publish(vel_cmd)

      if key_pressed == False:
        rate.sleep() # Limit the amount of messages when no key is pressed

      # Move cursor back up n lines (and erase them)
      for n in range(0,3):
        print('\033[1A', end="")
        print('\033[2K', end="")

  except rospy.ROSInterruptException:
      pass

  finally:
    # If we are quitting, stop the robot
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
