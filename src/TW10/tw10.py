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

@package docstring
Application of an Extended Kalman Filter for robot localization.
'''

# Our libraries and functions
from markers_msgs.msg import Markers
import LocalFrameWorldFrameTransformations as ft
from utils import clipValue, quaternionToYaw, drawPos
from ExtendedKalmanFilter import ExtendedKalmanFilter

# ROS API
import rospy
import message_filters
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Other libraries
import matplotlib as mpl
from matplotlib import pyplot as plt
import numpy as np
from threading import Lock
from math import radians, ceil
import random
import sys

# Global variables and constants
PREDICTION_STEP_ENABLED = True
OBSERVATION_STEP_ENABLED = True
ekf = ExtendedKalmanFilter()
outfile = open('Output.txt', 'w')  # Save debug information to this file
# Real robot pose from the simulator (for debugging purposes only)
real_pose = Pose2D(0, 0, 0)
real_pose_lock = Lock()  # Prevent simultaneous read/write to real_pose
odom_robot_pose = Pose2D()  # Last obtained pose from odometry
odom_updated_once = False  # True if the odometry was updated at least once
# Real landmarks positions [m] (for debugging purposes only)
markers_true_wpos = [ft.Point2D(-8.0,  4.0),  # 1
                     ft.Point2D(-8.0, -4.0),  # 2
                     ft.Point2D(-3.0, -8.0),  # 3
                     ft.Point2D(3.0, -8.0),  # 4
                     ft.Point2D(8.0, -4.0),  # 5
                     ft.Point2D(8.0, 4.0),  # 6
                     ft.Point2D(3.0, 8.0),  # 7
                     ft.Point2D(-3.0, 8.0)]  # 8
# Map related constants
MAP_RESOLUTION = 0.05  # [m/px]
MAP_LENGTH = 20.0  # Width and height of the map [m]
DELTA_SAVE = 20  # Time [secs] interval between each save of the map
prev_time = 0.0  # Last time the debug image was saved to disk
# Map for debugging purposes
dbg_map = np.full([ceil(MAP_LENGTH/MAP_RESOLUTION),
                   ceil(MAP_LENGTH/MAP_RESOLUTION),
                   3],
                  255,
                  np.uint8)

# Robot navigation/motion related constants and variables
MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]
# Linear and angular velocities for the robot (initially stopped)
lin_vel = 0.0  # [m/s]
ang_vel = 0.0  # [°/s]
avoid = False
rotating = False
rotate_left = False
stop_front_dist = 0.6  # Stop if closer than this distance
min_front_dist = 1.0  # Avoid if closer than this distance
vel_pub = None  # Velocity commands publisher (to be initialized later)
vel_cmd = Twist()  # Velocity commands


def on_key(event):
    '''Quit if the q key was pressed'''
    global plt

    if(event.key == 'q'):
        plt.close('all')  # Close all figures
        rospy.signal_shutdown('Quitting...')  # Ask ROS to shutdown


def showDebugInformation(timestamp: float, erase_landmarks: bool = True):
    '''
    Debug information function which shows the map with the real robot pose
    (green), odometry computed pose (magenta) and EKF estimated pose (blue).
    '''
    global odom_robot_pose, real_pose, real_pose_lock, ekf, prev_time
    global DELTA_DEBUG

    # Make sure real_pose is not being updated
    real_pose_lock.acquire()
    myreal_pose = Pose2D(real_pose.x, real_pose.y, real_pose.theta)
    real_pose_lock.release()

    # Write data to the file
    outfile.write(
        f'{timestamp:.2f}: {odom_robot_pose.x:.2f} ' +
        f'{odom_robot_pose.y:.2f} {odom_robot_pose.theta:.2f} ' +
        f'{myreal_pose.x:.2f} {myreal_pose.y:.2f} {myreal_pose.theta:.2f} ' +
        f'{ekf.state[0, 0]:.2f} {ekf.state[1, 0]:.2f} {ekf.state[2, 0]:.2f}')
    for n in range(ekf.num_landmarks):
        outfile.write(
            f' | {n}: {ekf.state[3+2*n,0]} {ekf.state[3+2*n+1,0]}\n')

    # Uncomment this if you want to erase previous debug information - it sets
    # all values to 255
    # dbg_map[:] = 255

    ''' Draw poses in image '''
    # Real pose (in green)
    drawPos(dbg_map, myreal_pose.x, myreal_pose.y, myreal_pose.theta,
            MAP_RESOLUTION, (0, 255, 0))
    # Pose estimated by odometry (in magenta)
    drawPos(dbg_map,
            odom_robot_pose.x, odom_robot_pose.y, odom_robot_pose.theta,
            MAP_RESOLUTION, (255, 0, 255))
    # Pose estimated by the EKF filter (in blue)
    drawPos(dbg_map, ekf.state[0, 0], ekf.state[1, 0], ekf.state[2, 0],
            MAP_RESOLUTION, (0, 0, 255))
    # Landmarks true positions (avialable only for debuggnin purposes)
    for n in range(markers_true_wpos.__len__()):
        drawPos(dbg_map, markers_true_wpos[n].x, markers_true_wpos[n].y, 0,
                MAP_RESOLUTION, (0, 255, 0), False, 5)
    # Draw landmarks estimates
    for n in range(ekf.num_landmarks):
        drawPos(dbg_map, ekf.state[3+2*n, 0], ekf.state[4+2*n, 0], 0,
                MAP_RESOLUTION, (255, 0, 0), False, 5)

    # Create/show window for the map
    fig = plt.figure("Debug")
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.cla()
    plt.imshow(dbg_map)
    # Add legend
    real_patch = mpl.patches.Patch(color='lime', label='Real pose')
    odo_patch = mpl.patches.Patch(color='magenta', label='Odometry pose')
    ekf_patch = mpl.patches.Patch(color='blue', label='EKF pose')
    real_landmark_patch = mpl.patches.Patch(
        edgecolor='black', facecolor=(0, 1, 0), label='Real landmark')
    est_landmark_patch = mpl.patches.Patch(
        edgecolor='black', facecolor=(1, 0, 0), label='Estimated landmark')
    plt.legend(loc='upper left',
               handles=[real_patch, odo_patch, ekf_patch,
                        real_landmark_patch, est_landmark_patch])
    plt.pause(0.05)  # Update window

    # Save map from time to time
    if timestamp - prev_time >= DELTA_SAVE:
        prev_time = timestamp
        plt.imsave('map.png', dbg_map)

    # Erase landmarks estimates (to keep the map viewable), if asket to
    if erase_landmarks:
        for n in range(ekf.num_landmarks):
            drawPos(dbg_map, ekf.state[3+2*n, 0], ekf.state[4+2*n, 0], 0,
                    MAP_RESOLUTION, (255, 255, 255), True, 5)


def realPoseCallback(msg: Odometry):
    '''
     Store real, error-free pose values given by the simulator (for debugging
    puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
    '''
    global real_pose, real_pose_lock

    real_pose_lock.acquire()
    real_pose.x = msg.pose.pose.position.x
    real_pose.y = msg.pose.pose.position.y
    real_pose.theta = quaternionToYaw(msg.pose.pose.orientation)
    real_pose_lock.release()


def laserCallback(msg: LaserScan):
    '''
    Use the laser information to navigate
    '''
    global lin_vel, ang_vel, rotating, rotate_left, vel_cmd, vel_pub
    global MAX_LIN_VEL, MAX_ANG_VEL

    # Update distance to closest front obstacles
    # Find index of -45 degrees
    min_angle_idx = round((radians(-45) - msg.angle_min)/msg.angle_increment)
    # Find index of 45 degrees
    max_angle_idx = round((radians(45) - msg.angle_min)/msg.angle_increment)
    # Get smaller value in the given range
    closest_front_obstacle = min(msg.ranges[min_angle_idx:max_angle_idx+1])

    # Perform navigation base on the detected obstacle
    avoid = False
    if closest_front_obstacle < min_front_dist:
        if closest_front_obstacle < stop_front_dist:
            avoid = True
            lin_vel = -0.100
        else:
            avoid = True
            lin_vel = 0.0
    else:
        lin_vel = 0.5
        ang_vel = 0.0
        rotating = False

    if avoid:
        lin_vel = 0.0
        if not rotating:
            # Favor rotation in the same direction as the last rotation
            if random.random() < 0.1:
                rotate_left = not rotate_left

            if rotate_left:
                ang_vel = radians(15.0)  # Rotate left
            else:
                ang_vel = radians(-15.0)  # Rotate right

            rotating = True

    # Limit maximum velocities
    # (not needed here)
    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    # Send velocity commands
    vel_cmd.angular.z = ang_vel
    vel_cmd.linear.x = lin_vel
    vel_pub.publish(vel_cmd)


def odomMarkersCallback(odom_msg, markers_msg):
    '''
    This callback is called whenever we habe an odometry and a markers message.
    Teh received messages are received here simultaneously and correspond to
    the same time stamp
    '''
    global odom_updated_once, odom_robot_pose, ekf, markers_true_wpos

    ''' Step 1 - Prediction step using odometry values

        If this is the first time this function is being called, then use the
        odometry values as our best estimate for the robot starting pose.
        In this case, do not compute the prediction step, but simply initialize
        the EKF state with this value
    '''
    new_odo_robot_pose = Pose2D(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        quaternionToYaw(odom_msg.pose.pose.orientation))
    if not odom_updated_once:
        # Initialize the EKF state with the initial odometry value
        # ekf.state[0, 0] = new_odo_robot_pose.x
        # ekf.state[1, 0] = new_odo_robot_pose.y
        # ekf.state[2, 0] = new_odo_robot_pose.theta
        # Use the three lines above, instead of the next three lines, if you
        # want to use the odometry valus for initialiing the Kalman filter.
        # Currently we are not using that information on startup, but assuming
        # instead that the robot is starting at (0.0,0.0, 0.0)
        ekf.state[0, 0] = 0.0
        ekf.state[1, 0] = 0.0
        ekf.state[2, 0] = 0.0
        odom_updated_once = True  # We have received or first value
    elif PREDICTION_STEP_ENABLED:  # Run only if step enabled
        # Compute the robot motion given the last know robot pose from odometry
        # and the current one
        local_pose = ft.world2LocalP(odom_robot_pose, new_odo_robot_pose)
        # Execute the actual prediction step
        ekf.predictStep(local_pose.x, local_pose.y, local_pose.theta)

    # Store current odometry as a reference for the next iteration
    odom_robot_pose = new_odo_robot_pose

    ''' Step 2 - Perform the update step based on the detected markers

        This step is only performed if the robot detected at least one
        landmark, and if this step is enabled
    '''
    if (markers_msg.num_markers > 0) and OBSERVATION_STEP_ENABLED:
        ekf.updateStep(markers_msg, markers_true_wpos)

    # Show debug information
    showDebugInformation(odom_msg.header.stamp.to_sec())


if __name__ == '__main__':
    '''
    Main function
    Random navigation with obstacle avoidance and EKF-based localization
    '''
    print('Random navigation with obstacle avoidance and EKF-based ' +
          'localization\n---------------------------')

    outfile.write(
        'Estimated and real pose of the robot, and of the landmarks\n\n' +
        '[T]: Odometry [X Y Theta] Real [X Y Theta] ' +
        'Kalman [X Y Theta]\n\n')

    '''
    Update/create map related variables

    We will create an image with the map wich contains the original map plus
    a border around it, so as to allow estimates outside of the original
    map.
    '''

    # Debug map is all white initially
    dbg_map[:] = (255, 255, 255)

    ''' ROS related code '''
    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('tw08')

    # Setup subscribers
    # Odometry
    sub_odom = message_filters.Subscriber(robot_name + '/odom', Odometry)
    # Detected markers
    sub_markers = message_filters.Subscriber(robot_name + "/markers", Markers)
    # Odometry and markers messages are received simultaneously, synchronized
    ts = message_filters.TimeSynchronizer([sub_odom, sub_markers], 5)
    ts.registerCallback(odomMarkersCallback)
    # Laser scans
    sub_laser = rospy.Subscriber(robot_name + "/base_scan", LaserScan,
                                 laserCallback, queue_size=1)
    # Real, error-free robot pose (for debug purposes only)
    sub_real_pose = rospy.Subscriber(robot_name+'/base_pose_ground_truth',
                                     Odometry, realPoseCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # Block until shutdown, running all callbacks parallel in background
    rospy.spin()

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # Close file
    outfile.close()

    # Store final map
    plt.imsave('map.png', dbg_map)
    plt.close('all')  # Close all figures
    sys.exit(0)
