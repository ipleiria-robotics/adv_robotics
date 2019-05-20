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
Application of an Extended Kalman Filter for robot localization using ICP.
'''

# Our libraries and functions
import LocalFrameWorldFrameTransformations as ft
from utils import quaternion2yaw, rpy2quaternion, drawPoses, drawCross
from ExtendedKalmanFilter import ExtendedKalmanFilter

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Other libraries
import matplotlib as mpl
from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
import cv2
from threading import Lock
from math import floor, ceil, cos, sin, sqrt, inf, degrees, atan2
import os
import sys

# Debug control constants
FULL_ICP_DEBUG = False  # Show information on every ICP step
BASIC_DEBUG = True  # Show only the most relevant debug information

# General global variables
PREDICTION_STEP_ENABLED = True  # If true, enable the prediction step
ekf = ExtendedKalmanFilter()
outfile = open('Output.txt', 'w')  # Save debug information to this file
# Real robot pose from the simulator (for debugging purposes only)
real_pose = Pose2D(0, 0, 0)
real_pose_lock = Lock()  # Prevent simultaneous read/write to real_pose
odom_robot_pose = Pose2D()  # Last obtained pose from odometry
odom_updated_once = False  # True if the odometry was updated at least once
pose_pub = None  # Estimated pose publisher

# Map related constants
MAP_RESOLUTION = 0.032  # [m/px]
MAP_LENGTH = 16.0  # Width and height of the map [m]
MAP_BORDER = 2.0  # Additional border to add around the map [m]
DELTA_SAVE = 20  # Time [secs] interval between each save of the map
prev_time = 0.0  # Last time the debug image was saved to disk
# Original map (reduced size)
org_map = np.empty([ceil(MAP_LENGTH/MAP_RESOLUTION),
                    ceil(MAP_LENGTH/MAP_RESOLUTION),
                    3],
                   np.uint8)
# Map for debugging purposes
dbg_map = np.full([ceil((MAP_LENGTH+MAP_BORDER)/MAP_RESOLUTION),
                   ceil((MAP_LENGTH+MAP_BORDER)/MAP_RESOLUTION),
                   3],
                  255,
                  np.uint8)
# Debug
if BASIC_DEBUG:
    # ICP image for debugging purposes
    dbg_icp = np.full([ceil((MAP_LENGTH+MAP_BORDER)/MAP_RESOLUTION),
                       ceil((MAP_LENGTH+MAP_BORDER)/MAP_RESOLUTION),
                      3],
                      255,
                      np.uint8)
#    # Create window for the ICP debug image
#    fig = plt.figure("Debug ICP")
#    plt_icp = plt.gca()
#    plt_icp.cla()

# ICP related constants and variables
icpfile = open('ICP.txt', 'w')  # Save ICP debug information to this file
MAX_ICP_ITERATIONS = 10  # Maximum number of ICP iterations
MIN_ICP_DELTA_ERROR = 0.01  # Minimum error difference to terminate ICP [m]
MIN_ICP_DELTATIME = 1.0  # Minimum time between consecutive IPC runs
last_icp_call_time = 0  # Timestamp of the last ICP run
kdtree = None  # Will hold the KD-Tree built from the environment map


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
        plt.imsave('mapa.png', dbg_map)


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


def odomCallback(odom_msg: Odometry):
    '''
    This callback is called whenever we habe an odometry and a markers message.
    Teh received messages are received here simultaneously and correspond to
    the same time stamp
    '''
    global odom_updated_once, odom_robot_pose, ekf, PREDICTION_STEP_ENABLED
    global BASIC_DEBUG

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

    # Publish computed pose value
    createAndPublishPose(odom_msg.header.stamp)

    # Show debug information
    if BASIC_DEBUG:
        showDebugInformation(odom_msg.header.stamp.to_sec())


def laserCallback(laser_msg: LaserScan):
    '''
    This callback is called whenever we habe an odometry and a markers message.
    Teh received messages are received here simultaneously and correspond to
    the same time stamp
    '''
    global ekf, cloud_map, BASIC_DEBUG, dbg_icp, FULL_ICP_DEBUG
    global kdtree, last_icp_call_time

    ''' Step 2 - Perform the update step based on the ICP algorithm

        The ICP algorithm will return an estimate for the current robot pose.
        We will use that estimate as the sensor data for the observation/update
        step.
    '''
    last_error = inf

    # Do the ICP algorithm only if enough time has elapsed.
    # We consider the data time as our time reference.
    current_time = laser_msg.header.stamp.to_sec()
    if current_time - last_icp_call_time < MIN_ICP_DELTATIME:
        return  # To soon, do nothing (return)
    last_icp_call_time = current_time

    # Debug
    if BASIC_DEBUG:
        # Clear debug image
        dbg_icp.fill(255)

        # Show original cloud
        for i in range(cloud_map.shape[0]):
            # Get point in map grid coordinates
            col = int(round(dbg_icp.shape[1]/2.0 +
                            cloud_map[i, 0]/MAP_RESOLUTION))
            row = int(round(dbg_icp.shape[0]/2.0 -
                            cloud_map[i, 1]/MAP_RESOLUTION))

            cv2.circle(dbg_icp, (col, row), 1, (0, 0, 0), -1)
        # Clear window for the ICP debug image
        #plt_icp.cla()
        # cv2.imshow('Debug ICP', dbg_icp)
        # cv2.waitKey(5)

    # Build sensed point cloud
    # This could be vectorized for increased speet. It is kept as it us for
    # increased readibility.
    angle = laser_msg.angle_min
    # Compute the number of elements
    num_sensed_points = (np.logical_and(
                            np.array(laser_msg.ranges) > laser_msg.range_min,
                            np.array(laser_msg.ranges) < laser_msg.range_max)
                         ).sum()
    sensed_cloud_polar = np.empty((num_sensed_points, 2))
    i = 0
    j = 0
    while angle <= laser_msg.angle_max:
        if (laser_msg.ranges[i] > laser_msg.range_min) and \
           (laser_msg.ranges[i] < laser_msg.range_max):
            # Store values in polar coordinates, as given by the sensor
            sensed_cloud_polar[j, 0] = laser_msg.ranges[i]
            sensed_cloud_polar[j, 1] = angle
            j = j + 1
        angle = angle + laser_msg.angle_increment
        i = i + 1
    # Convert to local cartesian coordinates from polar coordinates using
    # x = r.cos(angle)
    # y = r.sin(angle)
    sensed_cloud = np.empty_like(sensed_cloud_polar)
    sensed_cloud[:, 0] = (sensed_cloud_polar[:, 0] *
                          np.cos(sensed_cloud_polar[:, 1]))
    sensed_cloud[:, 1] = (sensed_cloud_polar[:, 0] *
                          np.sin(sensed_cloud_polar[:, 1]))

    # Debug
    if FULL_ICP_DEBUG:
        print('--------------------------------')
        # Output ICP data to a file for debugging purposes
        testfile = open('data_for_ICP.txt', 'w')
        testfile.write('-- Data file to test ICP elsewhere:\n' +
                       '-- Robot pose:\n' +
                       f'{ekf.state[0, 0]:.2f} {ekf.state[1, 0]:.2f} ' +
                       f'{ekf.state[2, 0]:.2f}')
        testfile.write('-- Original point cloud:')
        for i in range(cloud_map.shape[0]):
            testfile.write(f'{cloud_map[i, 0]:.2f} {cloud_map[i, 1]}')
        testfile.write('-- Sensed point cloud:')
        for i in range(sensed_cloud.shape[0]):
            testfile.write(f'{sensed_cloud[i, 0]} {sensed_cloud[i, 1]}')
        testfile.close()

    # This variable will hold the total transformations (rotation and
    # translation), which will correspond to the estimated robot pose.
    totalRotation = np.array([[cos(ekf.state[2, 0]), -sin(ekf.state[2, 0])],
                              [sin(ekf.state[2, 0]),  cos(ekf.state[2, 0])]])
    totalTranslation = np.array([[ekf.state[0, 0]],
                                 [ekf.state[1, 0]]])
    # Store initial transformations (rotation and translation)
    rotation = totalRotation.copy()
    translation = totalTranslation.copy()

    ''' ICP - The actual ICP algorithm starts here '''
    for num_icp_iterations in range(MAX_ICP_ITERATIONS):
        # Debug
        if FULL_ICP_DEBUG:
            print(f'--------------------> ICP iteration {num_icp_iterations}')

        # Update sensed cloud with last transformation
        # We do sensed_cloud*rotation_matrix.T, since the elements are
        # row-wise.
        # Recall that rotation.T is the trasponse of the rotation matrix
        sensed_cloud = sensed_cloud @ rotation.T
        # Translate
        sensed_cloud[:, 0] = sensed_cloud[:, 0] + translation[0, 0]
        sensed_cloud[:, 1] = sensed_cloud[:, 1] + translation[1, 0]

        # Debug
        if BASIC_DEBUG:
            # Show sensed cloud
            for i in range(sensed_cloud.shape[0]):
                # Get point in map grid coordinates
                col = int(round(dbg_icp.shape[1]/2.0 +
                                sensed_cloud[i, 0]/MAP_RESOLUTION))
                row = int(round(dbg_icp.shape[0]/2.0 -
                                sensed_cloud[i, 1]/MAP_RESOLUTION))

                cv2.circle(dbg_icp, (col, row), 1, (255, 0, 0), -1)
            # Show ICP
            # cv2.imshow('Debug ICP', dbg_icp)
            # cv2.waitKey(5)

        # Compute correspondence using the KDTree nearest neighbour
        dists, indexes = kdtree.query(sensed_cloud)
        # Build vector with only the correspondant pairs.
        # Once again, this could be vectorized for increased speed.
        correspondences = np.empty_like(sensed_cloud)
        for i in range(indexes.shape[0]):
            correspondences[i, 0] = cloud_map[indexes[i], 0]
            correspondences[i, 1] = cloud_map[indexes[i], 1]
            # Debug
            if BASIC_DEBUG:
                # Debug correspondences
                pt1 = (int(round(dbg_icp.shape[1]/2 +
                                 correspondences[i, 0]/MAP_RESOLUTION)),
                       int(round(dbg_icp.shape[0]/2 -
                                 correspondences[i, 1]/MAP_RESOLUTION)))
                pt2 = (int(round(dbg_icp.shape[1]/2 +
                                 sensed_cloud[i, 0]/MAP_RESOLUTION)),
                       int(round(dbg_icp.shape[1]/2 -
                                 sensed_cloud[i, 1]/MAP_RESOLUTION)))
                cv2.line(dbg_icp, pt1, pt2, (40, 150, 255))

        # Compute the translation as the difference between the center of mass
        # of each set of points in the correspondence
        # Compute mean along the rows (axis 0)
        mean_sensed_cloud = np.mean(sensed_cloud, 0, keepdims=True)
        mean_correspondences = np.mean(correspondences, 0, keepdims=True)

        # Debug
        if FULL_ICP_DEBUG:
            print(f'Sensed centroid: ' +
                  f'{mean_sensed_cloud[0, 0]} ' +
                  f'{mean_sensed_cloud[0, 1]}\n' +
                  f'Data centroid: ' +
                  f'{mean_correspondences[0, 0]} ' +
                  f'{mean_correspondences[0, 1]}')
        if BASIC_DEBUG:
            # Debug centers
            drawCross(dbg_icp, mean_sensed_cloud[0, 0],
                      mean_sensed_cloud[0, 1], MAP_RESOLUTION, (255, 0, 0))
            drawCross(dbg_icp, mean_correspondences[0, 0],
                      mean_correspondences[0, 1], MAP_RESOLUTION, (0, 0, 0))

        # Compute the rotation by:
        #  1. Compute the inercia W
        #  2. Perform SVD as W = USV'
        #  3. Compute ratation as R = VU'
        W = np.zeros((2, 2))
        for i in range(correspondences.shape[0]):
            W = W + (sensed_cloud[i, :] - mean_sensed_cloud).T @ \
                (correspondences[i, :] - mean_correspondences)
        U, S, Vt = np.linalg.svd(W)
        # Debug
        if FULL_ICP_DEBUG:
            print(f'W matrix:\n{W}\n' +
                  f'U matrix:\n{U}\n' +
                  f'S matrix:\n{S}\n' +
                  f'V matrix transposed:\n{Vt}')
        # For the SVD approach to always work, we use the approach detailed in
        # D.W. Eggert, A. Lorusso, R.B. Fisher, "Estimating 3-D rigid body
        # transformations: a comparison of four major algorithms", Machine
        # Vision and Applications, pp 272-290, Springer-Verlag, 1997.
        S = np.eye(2, 2)
        S[1, 1] = np.linalg.det(U @ Vt)  # U * Vt
        rotation = Vt.T @ S @ U.T

        # Compute translation
        translation = mean_correspondences.T - rotation @ mean_sensed_cloud.T

        # Update total transformation
        totalRotation = rotation @ totalRotation
        totalTranslation = rotation @ totalTranslation + translation

        # Debug
        if FULL_ICP_DEBUG:
            print(f' Current rotation:\n{rotation}\n' +
                  f' Current translation:\n{translation}\n'
                  f' Current total rotation:\n{totalRotation}\n' +
                  f' Current total translation:\n{totalTranslation}\n' +
                  f'Total rotation angle [degrees]: ' +
                  f'{degrees(atan2(rotation[1, 0], rotation[0, 0]))}')
        # Debug
        if BASIC_DEBUG:
            # Show ICP
            cv2.imshow('Debug ICP', dbg_icp)
            cv2.waitKey(5)
            # plt.imshow(dbg_icp)
            # plt.pause(0.05)  # Update window

        # Compute RMS error (before last computed transformation)
        sqr_dists = dists**2  # Compute the square of the distances
        result = np.mean(sqr_dists)  # Compute the mean
        curr_error = sqrt(result)  # Compute the square root
        # Debug
        if FULL_ICP_DEBUG:
            print(f'Current error: {curr_error}')
            icpfile.write(f'{curr_error}')

        # If the error variation is small enough, we're done
        if curr_error <= last_error:
            if last_error - curr_error < MIN_ICP_DELTA_ERROR:
                last_error = curr_error
                break
        else:
            # Debug
            if FULL_ICP_DEBUG:
                print('Got error increase!!!!')
        last_error = curr_error

    # Debug
    if BASIC_DEBUG:
        print(
            'Final ICP pose (x, y, theta): ' +
            f'{totalTranslation[0, 0]:.2f} {totalTranslation[1, 0]:.2f} ' +
            f'{degrees(atan2(totalRotation[1, 0], totalRotation[0, 0])):.2f}')
        icpfile.write('----------------------------')

    ''' Run the actual step 2 of the EKF, given the result of the above ICP run
        For the error, we will also consider the result above. '''
    W = np.array([[last_error, 0.0, 0.0],
                  [0.0, last_error, 0.0],
                  [0.0, 0.0, last_error]])
    z = np.array([[totalTranslation[0, 0]],  # x
                  [totalTranslation[1, 0]],  # y
                  [atan2(totalRotation[1, 0], totalRotation[0, 0])]])  # Theta
    ekf.updateStep(z, W)

    # Publish computed pose value
    createAndPublishPose(laser_msg.header.stamp)


if __name__ == '__main__':
    '''
    Main function
    EKF-based localization with ICP
    '''
    print('EKF-based localization with ICP ' +
          '\n---------------------------')

    # Associate key
    #fig = plt.figure("Debug ICP")
    #fig.canvas.mpl_connect('key_press_event', on_key)
    cv2.namedWindow('Debug ICP', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

    outfile.write('Estimated and real pose of the robot\n\n' +
                  '[T]: Odometry [X Y Theta] Real [X Y Theta] ' +
                  'EKF [X Y Theta]\n\n')

    '''
    Update/create map related variables

    We will create an image with the map wich contains the original map plus
    a border around it, so as to allow estimates outside of the original
    map.
    '''

    # Read original map
    map_file_path = os.environ['HOME']
    map_file_path += '/ros/src/mystage_ros/world/AR/cave_walls_only.png'
    org_full_size_map = (plt.imread(map_file_path)*255).astype(np.uint8)
    #  Resize it to our desired resolution
    org_map = cv2.resize(org_full_size_map[:, :, 1],
                         (org_map.shape[0], org_map.shape[1]))

    # Our actual map, for debugging purposes, will be equal to the original
    # one, with the border around it. As such, we copy the original map to
    # the middle of the dbg_map
    start_row = floor((dbg_map.shape[0]-org_map.shape[0])/2)
    end_row = start_row + org_map.shape[0]
    start_col = floor((dbg_map.shape[1]-org_map.shape[1])/2)
    end_col = start_col + org_map.shape[1]
    # Copy the org_map to the middle of the dbg_map
    dbg_map[start_row:end_row, start_col:end_col, 0] = org_map
    dbg_map[start_row:end_row, start_col:end_col, 1] = org_map
    dbg_map[start_row:end_row, start_col:end_col, 2] = org_map

    ''' Initilize Cloud based on map '''
    # Get total number of points in the original map (this number will be the
    # size of the cloud).
    # We compute this value as img_width*img_height-num_non_zero_points, since
    # the walls are black in our original map.
    # Note that countNonZero only works with grayscale images, so we just check
    # for one of the colors.
    num_map_points = (org_map.shape[1]*org_map.shape[0] -
                      cv2.countNonZero(org_map))
    # Prepare our cloud map with the correct number of points
    cloud_map = np.empty([num_map_points, 2])
    # Go through the image and store all the walls points in the cloud
    # NOTE: this 2 for cycles could be vectorized into few insctructions for
    # increased execution speed. I leave it as it is for readibility
    i = 0
    for y in range(org_map.shape[0]):
        for x in range(org_map.shape[1]):
            # If this pixel is 0, then it is a wall point. Add it to the cloud.
            if org_map[y, x] == 0:
                # print(f'{(x-_org_map.shape[1]/2)*MAP_RESOLUTION}\t' +
                #       f'{-(y-_org_map.shape[0]/2)*_MAP_RESOLUTION}'})
                cloud_map[i, 0] = (x-org_map.shape[1]/2)*MAP_RESOLUTION
                cloud_map[i, 1] = -(y-org_map.shape[0]/2)*MAP_RESOLUTION
                i = i + 1
    # Sanity check: the number of read points must be equal to the number of
    # expected points
    if i != num_map_points:
        rospy.logerr('Expected %d points but got %d points\n',
                     num_map_points, i)
        rospy.signal_shutdown()

    # Build KD-Tree for nearest neighbours search using our cloud map points
    kdtree = cKDTree(cloud_map)

    ''' ROS related code '''
    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('tw09_localization_node')

    # Setup subscribers
    # Odometry
    sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry, odomCallback,
                                queue_size=1)
    # Laser data
    sub_laser = rospy.Subscriber(robot_name + "/base_scan", LaserScan,
                                 laserCallback, queue_size=1)

    # Real, error-free robot pose (for debug purposes only)
    sub_real_pose = rospy.Subscriber(robot_name+'/base_pose_ground_truth',
                                     Odometry, realPoseCallback, queue_size=1)

    # Setup pose publisher
    pose_pub = rospy.Publisher(robot_name + '/odom_combined',
                               PoseWithCovarianceStamped, queue_size=2)

    # Block until shutdown, running all callbacks parallel in background
    rospy.spin()

    # Close files
    outfile.close()
    icpfile.close()

    # Store final map
    plt.imsave('map.png', dbg_map)
    plt.close('all')  # Close all figures
    sys.exit(0)
