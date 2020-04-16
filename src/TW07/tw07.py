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

# Matrices and OpenCV related functions
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
import cv2

# Library packages needed
from math import radians, ceil, sin, cos, sqrt, pi
import time
import sys
import os
import random
from threading import Lock, Condition

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import message_filters

# Our functions
from utils import clipValue, quaternionToYaw, drawPoses, normalize
import LocalFrameWorldFrameTransformations as ft
from markers_msgs.msg import Markers

# Specify if the particle filter steps should run
STEP_PREDICTION = False
STEP_UPDATE = False
STEP_RESAMPLE = False

# Debug related variables
DELTA_DEBUG = 1  # Show debug information ony once every DELTA_PRINT seconds

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = 1.57  # 90º/s (in rad/s)

# Map related constants
MAP_RESOLUTION = 0.032  # [m/px]
MAP_LENGTH = 16.0  # Width and height of the map [m]
DELTA_SAVE = 20  # Time [secs] interval between each save of the map

# Particle filter constants
NUM_PARTICLES = 200  # Number of particles used in the particle filter
DISTANCE_ERROR_GAIN = 0.5  # Gain factor for the weights given the distance
ANGLE_ERROR_GAIN = 5.0  # Gain factor when computing the weights from the angle
SIGMA1 = 0.2  # Distance standard deviation error [m]
SIGMA2 = radians(0.3)  # Theta standard deviation error [rad]

# General global variables
outfile = open('Output.txt', 'w')  # Save robot poses to this file
odo_robot_pose = Pose2D()
odom_updated_once = False  # True if the odometry was updated at least once
imshow_lock = Lock()
imshow_cond = Condition(imshow_lock)
# Real robot pose from the simulator (for debugging purposes only)
real_pose = Pose2D()
end_program = False  # End infinite loop when True

# Particle-filter global variables
# This variable will hold our final estimation at each iteration
pose_estimate = Pose2D()
# Vector of NUM_PARTICLES particles, where each particle corresponds to a
# possible robot pose (X,Y,Theta).
particles_x = np.empty([NUM_PARTICLES, 1])
particles_y = np.empty([NUM_PARTICLES, 1])
particles_theta = np.empty([NUM_PARTICLES, 1])
# Particles weight
particles_weight = np.full([NUM_PARTICLES, 1], 1.0/NUM_PARTICLES)
# The old particles vectors will be needed in the resampling algorithm
old_particles_x = np.empty([NUM_PARTICLES, 1])
old_particles_y = np.empty([NUM_PARTICLES, 1])
old_particles_theta = np.empty([NUM_PARTICLES, 1])
max_weight = 1.0  # Weight of the best particle (to be computed)

# Map global variables
org_map = np.zeros([ceil(MAP_LENGTH/MAP_RESOLUTION),
                    ceil(MAP_LENGTH/MAP_RESOLUTION)],
                   np.uint8)
# Color map for debugging purposes
dbg_map = np.zeros([ceil(MAP_LENGTH/MAP_RESOLUTION),
                    ceil(MAP_LENGTH/MAP_RESOLUTION),
                    3],
                   np.uint8)
tmp_map = np.empty_like(dbg_map)
# Control the time between map file save
prev_time = 0.0

# Create robot related variables
# Linear and angular velocities for the robot (initially stopped)
lin_vel = 0.0
ang_vel = 0.0
avoid = False
rotating = False
rotate_left = False
stop_front_dist = 0.6  # Stop if closer than this distance
min_front_dist = 1.0  # Avoid if closer than this distance
vel_pub = None  # Velocity commands publisher (to be initialized later)
vel_cmd = Twist()  # Velocity commands

# Store the positions of the landmarks [m]
markers_wpos = [ft.Point2D(-8.0,  4.0),  # 1
                ft.Point2D(-8.0, -4.0),  # 2
                ft.Point2D(-3.0, -8.0),  # 3
                ft.Point2D(3.0, -8.0),  # 4
                ft.Point2D(8.0, -4.0),  # 5
                ft.Point2D(8.0, 4.0),  # 6
                ft.Point2D(3.0, 8.0),  # 7
                ft.Point2D(-3.0, 8.0)]  # 8


def on_key(event):
    '''Quit if the q key was pressed'''
    global end_program
    if(event.key == 'q'):
        plt.close('all')  # Close all figures
        end_program = True  # In order to end the "infinite" loop


def showDebugInformation(timestamp: float):
    '''
    Debug information function

    We draw all the debug information (particles, etc.) before the
    resampling step, since the particles weight will be changed, and some
    particles will be lost in the process.
    '''
    global outfile, odo_robot_pose, real_pose, pose_estimate
    global dbg_map, tmp_map, NUM_PARTICLES, MAP_RESOLUTION, plt
    global particles_x, particles_y, particles_theta, particles_weight

    # Write data to the file
    outfile.write(f'{timestamp:.2f}: {odo_robot_pose.x:.2f} ' +
                  f'{odo_robot_pose.y:.2f} {odo_robot_pose.theta:.2f} ' +
                  f'{real_pose.x:.2f} {real_pose.y:.2f} ' +
                  f'{real_pose.theta:.2f} {pose_estimate.x:.2f} ' +
                  f'{pose_estimate.y:.2f} {pose_estimate.theta:.2f}')

    # Uncomment this if you want to erase previous debug information - it sets
    # all values to 255
    # dbg_map[:] = 255

    ''' Draw poses in image '''
    # Real pose (in green)
    drawPoses(dbg_map, real_pose.x, real_pose.y, real_pose.theta,
              MAP_RESOLUTION, (0, 255, 0))
    # Pose estimated by odometry (in magenta)
    drawPoses(dbg_map,
              odo_robot_pose.x, odo_robot_pose.y, odo_robot_pose.theta,
              MAP_RESOLUTION, (255, 0, 255))
    # Pose estimated by particle filter (in blue)
    drawPoses(dbg_map, pose_estimate.x, pose_estimate.y, pose_estimate.theta,
              MAP_RESOLUTION, (0, 0, 255))

    tmp_map = dbg_map.copy()
    # Draw all particles
    # The intensity color (highest is red, lowest is yellow) varies with the
    # particle weight.
    for j in range(0, NUM_PARTICLES):
        drawPoses(tmp_map,
                  particles_x.item(j),
                  particles_y.item(j),
                  particles_theta.item(j),
                  MAP_RESOLUTION,
                  (255, round(255*(1.0-particles_weight.item(j)/max_weight)),
                   0))

    # Redraw estimate by particle filter (in light blue), to stay on top
    drawPoses(tmp_map,
              pose_estimate.x, pose_estimate.y, pose_estimate.theta,
              MAP_RESOLUTION, (0, 255, 242))

    # Show map with poses
    imshow_lock.acquire()
    plt.figure("Debug")
    plt.cla()
    plt.imshow(tmp_map)
    # Add legend
    real_patch = mpl.patches.Patch(color='lime', label='Real pose')
    odo_patch = mpl.patches.Patch(color='magenta', label='Odometry pose')
    pf_patch = mpl.patches.Patch(color='blue', label='PF pose')
    bestp_patch = mpl.patches.Patch(color='cyan', label='Last best particle')
    plt.legend(loc='upper left', handles=[real_patch, odo_patch,
                                          pf_patch, bestp_patch])
    # Trigger figure update in main thread
    imshow_cond.notify()
    imshow_lock.release()


def realPoseCallback(msg: Odometry):
    '''
     Store real, error-free pose values given by the simulator (for debugging
    puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
    '''
    global real_pose

    real_pose.x = msg.pose.pose.position.x
    real_pose.y = msg.pose.pose.position.y
    real_pose.theta = quaternionToYaw(msg.pose.pose.orientation)


def odomMarkersCallback(odom_msg: Odometry, markers_msg: Markers):
    '''
    This callback is called whenever we habe an odometry and a markers message.
    Teh received messages are received here simultaneously and correspond to
    the same time stamp
    '''
    global DISTANCE_ERROR_GAIN, SIGMA1, SIGMA2, MAP_LENGTH, MAP_RESOLUTION
    global NUM_PARTICLES, odo_robot_pose, odom_updated_once, pose_estimate
    global org_map, max_weight, prev_time
    global old_particles_x, old_particles_y, old_particles_theta
    global particles_x, particles_y, particles_theta, particles_weight

    # Store updated pose values (this variable is shared/changed in different
    # threads)

    ''' Step 1 - Update particles given the robot motion: '''
    new_odo_robot_pose = Pose2D(odom_msg.pose.pose.position.x,
                                odom_msg.pose.pose.position.y,
                                quaternionToYaw(odom_msg.pose.pose.orientation))
    if STEP_PREDICTION and odom_updated_once:
        # Get estimated motion (Δd and Δθ) from odometry
        local_pose = ft.world2LocalP(odo_robot_pose, new_odo_robot_pose)
        distance = local_pose.x
        dtheta = local_pose.theta

        '''
        Δd = Δd*normal_noise(1.0, sigma1)
        Δθ = Δθ*normal_noise(1.0, sigma2)

        | x(k+1|x(k),u(k)) | = | x(k) +  Δx(k) |
        | y(k+1|x(k),u(k)) | = | y(k) +  Δy(k) |
        | θ(k+1|x(k),u(k)) | = | θ(k) +  Δθ(k) |
         where
        Δx(k) = Δd*cos(θ(k))
        Δy(k) = Δd*sin(θ(k))
        Δθ(k) = Δθ(k)
        '''

        # This could be vectorized to improved speed and avoid the for loop!
        for n in range(0, NUM_PARTICLES):
            rnd_distance = random.gauss(distance, distance*SIGMA1)
            rnd_dtheta = random.gauss(dtheta, dtheta*SIGMA2)
            delta_X = rnd_distance*cos(particles_theta[n])
            delta_Y = rnd_distance*sin(particles_theta[n])

            particles_x[n] += delta_X
            particles_y[n] += delta_Y
            particles_theta[n] += rnd_dtheta

            #  Check if any of particles is outside the map our in an occupied
            # cell. If so, move that particle to the previous position (the
            # orientation is maintained).
            coord_x = round(org_map.shape[1]/2.0 +
                            particles_x.item(n)/MAP_RESOLUTION)
            coord_y = round(org_map.shape[0]/2.0 -
                            particles_y.item(n)/MAP_RESOLUTION)
            if((coord_x < 0) or (coord_x >= org_map.shape[1]) or
               (coord_y < 0) or (coord_y >= org_map.shape[0]) or
               (org_map[coord_y, coord_x] < 127)):
                    # Undo this particle update (position only)
                    particles_x[n] -= delta_X
                    particles_y[n] -= delta_Y

            # Normalize the orientation
            particles_theta[n] = normalize(particles_theta[n])
    elif not odom_updated_once:
        odom_updated_once = True

    # Store current odometry value
    odo_robot_pose = new_odo_robot_pose

    #
    # Steps 2 (weights) and 3 (resampling) only run if we got a markers message
    #
    ''' Step 2 - Update the particle weights given the sensor model and map
     knowledge '''
    if STEP_UPDATE:
        '''
        For now we only perform this step if there was any marker detected.
        We could use the expected and not detected beacons for each particle,
        but it woul become too expensive.

        The weight for each particle will be:
         w(j) = mean(1/(1+sqrt((x_e(i)-x_r(i))^2+(y_e(i)-y_r(i))^2) *
                    DISTANCE_ERROR_GAIN))
        where x_e(i)/y_e(i) and x_r(i)/y_r(i) are the expected and obtained
        x/y world coordinates of the detected marker respectively. The GAIN are
        constant gains wich can be tuned to value smaller detection errors.
        '''
        if markers_msg.num_markers >= 1:
            run_steps23 = True
            # Reset normalization factor
            norm_factor = 0
            for j in range(0, NUM_PARTICLES):
                # Compute the weight for each particle
                # For each obtained beacon value
                particles_weight[j] = 0
                for n in range(0, markers_msg.num_markers):
                    # Obtain beacon position in world coordinates
                    particle = Pose2D(particles_x.item(j),
                                      particles_y.item(j),
                                      particles_theta.item(j))

                    marker_lpos = ft.Point2D(markers_msg.range[n] *
                                             cos(markers_msg.bearing[n]),
                                             markers_msg.range[n] *
                                             sin(markers_msg.bearing[n]))
                    marker_wpos = ft.local2Worldp(particle, marker_lpos)

                    particles_weight[j] += \
                        1.0/(1+sqrt((markers_wpos[markers_msg.id[n]-1].x -
                                     marker_wpos.x)**2 +
                                    (markers_wpos[markers_msg.id[n]-1].y -
                                     marker_wpos.y)**2) * DISTANCE_ERROR_GAIN)

                # Perform the mean. We summed all elements above and now divide
                # by the number of elements summed.
                particles_weight[j] /= markers_msg.num_markers
                # Update the normalization factor
                norm_factor += particles_weight[j]
        else:
            run_steps23 = False

        # Normalize the weight
        if run_steps23:
            max_weight = 0.0
            for j in range(0, NUM_PARTICLES):
                particles_weight[j] /= norm_factor
                # Store the particle with the best weight as our pose estimate
                if particles_weight[j] > max_weight:
                    pose_estimate.x = particles_x.item(j)
                    pose_estimate.y = particles_y.item(j)
                    pose_estimate.theta = particles_theta.item(j)
                    # This max_factor is just used for debug, so that we have
                    # more different colors between particles.
                    max_weight = particles_weight.item(j)

    # Show debug information
    showDebugInformation(odom_msg.header.stamp.to_sec())

    ''' Step 3 - Resample '''
    if STEP_RESAMPLE and run_steps23:
        '''
        The resampling step is the exact implementation of the algorithm
        described in the theoretical classes, i.e., the "Importance resampling
        algorithm".
        '''

        # Save the current values
        # The old particles will be needed in the resampling algorithm
        old_particles_x = particles_x.copy()
        old_particles_y = particles_y.copy()
        old_particles_theta = particles_theta.copy()
        old_particles_weight = particles_weight.copy()

        delta = random.random()/NUM_PARTICLES
        c = old_particles_weight[0]
        i = 0
        for j in range(0, NUM_PARTICLES):
            u = delta + j/(1.0*NUM_PARTICLES)
            while u > c:
                i += 1
                c += old_particles_weight[i]
            particles_x[j] = old_particles_x[i]
            particles_y[j] = old_particles_y[i]
            particles_theta[j] = old_particles_theta[i]
            # The weight is indicative only, it will be recomputed on marker
            # detection
            particles_weight[j] = old_particles_weight[i]

    '''
    Particle filter steps end here
    '''

    # Save map from time to time
    curr_time = time.time()
    if curr_time - prev_time >= DELTA_DEBUG:
        prev_time = curr_time
        plt.imsave('map.png', dbg_map)


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


if __name__ == '__main__':
    '''
    Main function
    Use particle filter for localization.
    '''

    outfile.write('Estimated and real pose of the robot\n\n' +
                  '[T]: Odometry [X Y Theta] Real [X Y Theta] ' +
                  'Particle [X Y Theta]\n\n')

    #
    # Create map related variables
    #
    #  We will create an image with the map wich contains the original map plus
    # a border around it, so as to allow estimates outside of the original
    # map.
    map_file_path = os.environ['HOME']
    map_file_path += '/ros/src/mystage_ros/world/AR/cave.png'
    cave_map = (plt.imread(map_file_path)*255).astype(np.uint8)

    # Read original map and resize it to our resolution (in color)
    dbg_map = cv2.resize(cave_map, (dbg_map.shape[0], dbg_map.shape[1]))

    # Read original map and resize it to our resolution
    # We need the original map, black and white, so that it is faster to find
    # occupied cells
    org_map = (dbg_map[:, :, 0]).copy()

    # Create window for the map
    fig = plt.figure("Debug")
    plt.imshow(dbg_map)
    plt.pause(0.5)
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Particle filter related variables with their initial values
    '''
    # Initialize particles with uniform random distribution across all space
    particles_x = np.random.uniform(-MAP_LENGTH/2.0,
                                    MAP_LENGTH/2.0,
                                    particles_x.shape)  # X
    particles_y = np.random.uniform(-MAP_LENGTH/2.0,
                                    MAP_LENGTH/2.0,
                                    particles_y.shape)  # Y
    particles_theta = np.random.uniform(-pi, pi,
                                        particles_theta.shape)  # Theta

    # Re-add particles that are outside of the map or inside an obstacle, until
    # it is in a free space inside the map.
    # This could be implemented in phewer lines with no cycles, but is kept as
    # is for clarity.
    for n in range(0, NUM_PARTICLES):
        while True:
            map_y = int(np.rint(org_map.shape[0]/2.0 -
                        particles_y[n]/MAP_RESOLUTION))
            map_x = int(np.rint(org_map.shape[1]/2.0 +
                        particles_x[n]/MAP_RESOLUTION))
            if((map_y < 0) or (map_y >= org_map.shape[0]) or  # Out of bonds
               (map_x < 0) or (map_x >= org_map.shape[1]) or  # Out of bonds
               (org_map[map_y, map_x] < 127)):  # Obstacle here
                # Particle is no good, generate new position for it
                particles_x[n] = random.uniform(-MAP_LENGTH/2.0,
                                                MAP_LENGTH/2.0)
                particles_y[n] = random.uniform(-MAP_LENGTH/2.0,
                                                MAP_LENGTH/2.0)
            else:
                # Particle is good, proceed to the next on
                break

    '''
    Random navigation with obstacle avoidance and particle filter-based
    localization
    '''
    print('Random navigation with obstacle avoidance and particle filter-' +
          'based localization\n---------------------------')

    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('tw07')

    # Setup subscribers
    # Real, error-free robot pose (for debug purposes only)
    sub_real_pose = rospy.Subscriber(robot_name+'/base_pose_ground_truth',
                                     Odometry, realPoseCallback, queue_size=1)
    # Odometry
    sub_odom = message_filters.Subscriber(robot_name + '/odom', Odometry)
    # Detected Markers
    sub_markers = message_filters.Subscriber(robot_name + "/markers", Markers)
    # Odometry and markers messages are received simultaneously, synchronized
    ts = message_filters.TimeSynchronizer([sub_odom, sub_markers], 5)
    ts.registerCallback(odomMarkersCallback)
    # Laser scans
    sub_laser = rospy.Subscriber(robot_name + "/base_scan", LaserScan,
                                 laserCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Wait until we get one odometry message
    while (not end_program) and (not odom_updated_once):
        rate.sleep()
        if end_program:
            plt.close('all')  # Close all figures
            sys.exit(0)

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # "Loop" until shutdown
    imshow_lock.acquire()
    while(not end_program) and (not rospy.is_shutdown()):
        # Wait until event was triggered in the showDebugInformation
        imshow_cond.wait()
        plt.pause(0.01)  # Update window

    # Stop the robot
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # Close file
    outfile.close()

    # Store final map
    plt.imsave('map.png', dbg_map)
    plt.close('all')  # Close all figures
    sys.exit(0)
