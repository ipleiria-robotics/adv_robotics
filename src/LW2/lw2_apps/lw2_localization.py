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

# ROS API
import rospy
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import LaserScan


def odomLaserCallback(odom_msg: Odometry, msg: LaserScan):
    '''Function to call when new odometry and laser information are available
    '''
    # Odometry values
    # odom_msg.pose.pose.position.x  # X coordinate
    # odom_msg.pose.pose.position.y  # Y coordinate
    # quaternionToYaw(odom_msg.pose.pose.orientation)  # Angle

    # odom_msg.twist.twist.linear.x  # Linear speed
    # odom_msg.twist.twist.angular.z  # Angular speed

    # Laser
    # msg.ranges has the distances measured
    # msg.intensities has the relefections intensities


'''
Main function
Controls the robot using the keyboard keys and outputs posture and velocity
related information.
'''
if __name__ == '__main__':
    robot_name = '/robot_0'
    # Setup subscriber for odometry and laser (sincronized)
    sub_odom = message_filters.Subscriber(robot_name + '/odom', Odometry)
    sub_laser = message_filters.Subscriber(robot_name + "/base_scan",
                                           LaserScan)
    ts = message_filters.TimeSynchronizer([sub_odom, sub_laser], 5)
    ts.registerCallback(odomLaserCallback)

    # Init ROS
    rospy.init_node('localization')

    # Infinite loop
    rate = rospy.Rate(10)  # 10 Hz, Rate when no key is being pressed
    while not rospy.is_shutdown():
        # Do something
        rate.sleep()
