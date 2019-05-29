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
from utils import quaternion2yaw, rpy2quaternion, drawPoses
from ExtendedKalmanFilter import ExtendedKalmanFilter

# ROS API
import rospy
import message_filters
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

# Other libraries
import matplotlib as mpl
from matplotlib import pyplot as plt
import numpy as np
from threading import Lock
import os
import sys

# Global variables and constants
PREDICTION_STEP_ENABLED = True
OBSERVATION_STEP_ENABLED = True
DEBUG_ENABLED = False  # Disable debug for faster execution
ekf = ExtendedKalmanFilter()
outfile = open('Output-localization.txt', 'w')  # Save debug information
# Real robot pose from the simulator (for debugging purposes only)
real_pose = Pose2D(0, 0, 0)
real_pose_lock = Lock()  # Prevent simultaneous read/write to real_pose
odom_robot_pose = Pose2D()  # Last obtained pose from odometry
odom_updated_once = False  # True if the odometry was updated at least once
# Landmarks positions [m]
markers_wpos = [ft.Point2D(-3.32, -2.28),  # 1
                ft.Point2D(-3.32, 2.28),  # 2
                ft.Point2D(3.32, 2.28),  # 3
                ft.Point2D(3.32, -2.28)]  # 4
pose_pub = None  # Estimated pose publisher
# Map related constants
MAP_RESOLUTION = 0.00992  # [m/px]
DELTA_SAVE = 20  # Time [secs] interval between each save of the map
prev_time = 0.0  # Last time the debug image was saved to disk
# Map for debugging purposes (initialized later)
dbg_map = None


def on_key(event):
    '''Quit if the q key was pressed'''
    global plt

    if(event.key == 'q'):
        plt.close('all')  # Close all figures
        rospy.signal_shutdown('Quitting...')  # Ask ROS to shutdown


def showDebugInformation(timestamp: float):
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

    # Uncomment this if you want to erase previous debug information - it sets
    # all values to 255
    # dbg_map[:] = 255

    ''' Draw poses in image '''
    # Real pose (in green)
    drawPoses(dbg_map, myreal_pose.x, myreal_pose.y, myreal_pose.theta,
              MAP_RESOLUTION, (0, 255, 0))
    # Pose estimated by odometry (in magenta)
    drawPoses(dbg_map,
              odom_robot_pose.x, odom_robot_pose.y, odom_robot_pose.theta,
              MAP_RESOLUTION, (255, 0, 255))
    # Pose estimated by the EKF filter (in blue)
    drawPoses(dbg_map, ekf.state[0, 0], ekf.state[1, 0], ekf.state[2, 0],
              MAP_RESOLUTION, (0, 0, 255))

    # Create/show window for the map
    fig = plt.figure("Debug")
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.cla()
    plt.imshow(dbg_map)
    # Add legend
    real_patch = mpl.patches.Patch(color='lime', label='Real pose')
    odo_patch = mpl.patches.Patch(color='magenta', label='Odometry pose')
    ekf_patch = mpl.patches.Patch(color='blue', label='EKF pose')
    plt.legend(loc='upper left', handles=[real_patch, odo_patch, ekf_patch])
    plt.pause(0.05)  # Update window

    # Save map from time to time
    if timestamp - prev_time >= DELTA_SAVE:
        prev_time = timestamp
        plt.imsave('map.png', dbg_map)


def realPoseCallback(msg: Odometry):
    '''
     Store real, error-free pose values given by the simulator (for debugging
    puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
    '''
    global real_pose, real_pose_lock

    real_pose_lock.acquire()
    real_pose.x = msg.pose.pose.position.x
    real_pose.y = msg.pose.pose.position.y
    real_pose.theta = quaternion2yaw(msg.pose.pose.orientation)
    real_pose_lock.release()


def odomMarkersCallback(odom_msg, markers_msg):
    '''
    This callback is called whenever we habe an odometry and a markers message.
    Teh received messages are received here simultaneously and correspond to
    the same time stamp
    '''
    global odom_updated_once, odom_robot_pose, ekf, markers_wpos

    ''' Step 1 - Prediction step using odometry values

        If this is the first time this function is being called, then use the
        odometry values as our best estimate for the robot starting pose.
        In this case, do not compute the prediction step, but simply initialize
        the EKF state with this value
    '''
    new_odo_robot_pose = Pose2D(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        quaternion2yaw(odom_msg.pose.pose.orientation))
    if not odom_updated_once:
        # Initialize the EKF state with the initial odometry value
        ekf.state[0, 0] = new_odo_robot_pose.x
        ekf.state[1, 0] = new_odo_robot_pose.y
        ekf.state[2, 0] = new_odo_robot_pose.theta
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
        ekf.updateStep(markers_msg, markers_wpos)

    # Publish computed pose value
    createAndPublishPose(odom_msg.header.stamp)

    # Show debug information
    if DEBUG_ENABLED:
        showDebugInformation(odom_msg.header.stamp.to_sec())


def createAndPublishPose(timestamp):
    '''
    Create and publish the message with the estimated robot pose resulting from
    the application of the EKF with the ICP algorithm.
    '''
    global odom_robot_pose, real_pose, real_pose_lock, ekf, prev_time
    global DELTA_DEBUG

    # Update and send pose message
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = timestamp
    pose_msg.pose.pose.position.x = float(ekf.state[0, 0])
    pose_msg.pose.pose.position.y = float(ekf.state[1, 0])
    pose_msg.pose.pose.position.z = 0.0  # Robot is on the ground
    # Store robot orientation in quaternions by converting from RPY. In this
    # we only have a rotation around the Z axis
    q = rpy2quaternion(0, 0, ekf.state[2, 0])
    pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    # Fill covariance matrix. We will consider 0 for all the values we are not
    # computing, i.e., Z, RX and RY, and correlated values.
    pose_msg.pose.covariance = np.array([
        # X, Y, Z, RX, RY, RZ
        ekf.P[0, 0], ekf.P[0, 1], 0.0, 0.0, 0.0, ekf.P[0, 2],  # X
        ekf.P[1, 0], ekf.P[1, 1], 0.0, 0.0, 0.0, ekf.P[1, 2],  # Y
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Z
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RX
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RY
        ekf.P[2, 0], ekf.P[2, 1], 0.0, 0.0, 0.0, ekf.P[2, 2]])  # RZ

    # Publish message
    pose_pub.publish(pose_msg)


if __name__ == '__main__':
    '''
    Main function
    Random navigation with obstacle avoidance and EKF-based localization
    '''
    print('EKF-based localization\n---------------------------')

    outfile.write('Estimated and real pose of the robot\n\n' +
                  '[T]: Odometry [X Y Theta] Real [X Y Theta] ' +
                  'Particle [X Y Theta]\n\n')

    '''
    Update/create map related variables

    We will create an image with the map wich contains the original map plus
    a border around it, so as to allow estimates outside of the original
    map.
    '''
    if DEBUG_ENABLED:
        # Read original map
        map_file_path = os.environ['HOME']
        map_file_path += '/ros/src/mystage_ros/world/AR/factory.png'
        org_full_size_map = (plt.imread(map_file_path)*255).astype(np.uint8)

        # Our actual map, for debugging purposes, will be equal to the original
        # one, but in color
        dbg_map = np.empty([org_full_size_map.shape[0],
                            org_full_size_map.shape[1],
                            3],
                           np.uint8)
        # Copy the org_map to the middle of the dbg_map
        dbg_map[:, :, 0] = org_full_size_map.copy()
        dbg_map[:, :, 1] = org_full_size_map.copy()
        dbg_map[:, :, 2] = org_full_size_map.copy()

    ''' ROS related code '''
    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('localization')

    # Setup subscribers
    # Odometry
    sub_odom = message_filters.Subscriber(robot_name + '/odom', Odometry)
    # Detected markers
    sub_markers = message_filters.Subscriber(robot_name + "/markers", Markers)
    # Odometry and markers messages are received simultaneously, synchronized
    ts = message_filters.TimeSynchronizer([sub_odom, sub_markers], 2)
    ts.registerCallback(odomMarkersCallback)

    # Real, error-free robot pose (for debug purposes only)
    sub_real_pose = rospy.Subscriber(robot_name+'/base_pose_ground_truth',
                                     Odometry, realPoseCallback, queue_size=1)

    # Setup pose publisher
    pose_pub = rospy.Publisher(robot_name + '/odom_combined',
                               PoseWithCovarianceStamped, queue_size=2)

    # Block until shutdown, running all callbacks parallel in background
    rospy.spin()

    # Close file
    outfile.close()

    # Store final map
    plt.imsave('map.png', dbg_map)
    plt.close('all')  # Close all figures
    sys.exit(0)
