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

# Matrices and OpenCV related functions
import numpy as np
from matplotlib import pyplot as plt

# Library packages needed
from math import radians, ceil, sin, cos, sqrt, pi
import time
import sys
import os
import random

# ROS API
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Our functions
from utils import clipValue, quaternionToYaw, drawPoses, normalize
import LocalFrameWorldFrameTransformations as ft
from markers_msgs.msg import Markers

# Specify if the particle filter steps should run
STEP_PREDICTION = True
STEP_UPDATE = True
STEP_RESAMPLE = True

# Debug related variables
DELTA_DEBUG = 1  # Show debug information ony once every DELTA_PRINT seconds

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = 1.57  # 90º/s (in rad/s)

# Map related constants
MAP_RESOLUTION = 0.032  # [m/px]
MAP_LENGTH = 16.0  # Width and height of the map [m]
SAFETY_BORDER = 0.4  # Border around the map [m]
DELTA_SAVE = 5  # Time interval between each save of the map

# Particle filter constants
NUM_PARTICLES = 200  # Number of particles used in the particle filter
LANDMARK_RANGE = 8.0  # Maximum distance to detectable landmarks [m]
LANDMARK_FOV = 180.0  # Landmarks are detectable if in front of robot [º]
DISTANCE_ERROR_GAIN = 0.5  # Gain factor for the weights give the distance
ANGLE_ERROR_GAIN = 5.0  # Gain factor when computing the weights from the angle
SIGMA1 = 0.1  # Distance standard deviation error [m]
SIGMA2 = radians(0.2)  # Theta standard deviation error [rad]

# General global variables
outfile = open('Output.txt', 'w')  # Save robot poses to this file
odo_robot_pose = Pose2D()
odo_lin_vel = 0.0  # Store current linear velocity
odo_ang_vel = 0.0  # Store current angular velocity
odom_updated = False  # True if we got an odometry update
odom_first_update = True  # True if the odometry was never updated
# Real robot pose from the simulator (for debugging purposes only)
real_pose = Pose2D()

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
last_step1_time = 0.0  # Last time we performed the Particle Filter step 1
# The old particles vectors will be needed in the resampling algorithm
old_particles_x = np.empty([NUM_PARTICLES, 1])
old_particles_y = np.empty([NUM_PARTICLES, 1])
old_particles_theta = np.empty([NUM_PARTICLES, 1])
max_weight = 1.0  # Weight of the best particle (to be computed)
iteration = 0  # Controlo the number of iterations for debugging purposes

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
    global plt

    if(event.key == 'q'):
        plt.close('all')  # Close all figures
        rospy.signal_shutdown('Quitting...')  # Ask ROS to shutdown


def showDebugInformation():
    '''
    Debug information function

    We draw all the debug information (particles, etc.) before the
    resampling step, since the particles weight will be changed, and some
    particles will be lost in the process.
    '''
    global outfile, last_step1_time, odo_robot_pose, real_pose, pose_estimate
    global dbg_map, tmp_map, NUM_PARTICLES, MAP_RESOLUTION, plt
    global particles_x, particles_y, particles_theta, particles_weight

    # Write data to the file
    outfile.write(f'{last_step1_time:.2f}: {odo_robot_pose.x:.2f} ' +
                  f'{odo_robot_pose.y:.2f} {odo_robot_pose.theta:.2f} ' +
                  f'{real_pose.x:.2f} {real_pose.y:.2f} ' +
                  f'{real_pose.theta:.2f} {pose_estimate.x:.2f} ' +
                  f'{pose_estimate.y:.2f} {pose_estimate.theta:.2f}')

    # Draw poses in image
    dbg_map[:] = 255  # Erase previous debug information, set all values to 255

    # Real poses (in green)
    drawPoses(dbg_map, real_pose.x, real_pose.y, real_pose.theta,
              MAP_RESOLUTION, (0, 255, 0))
    # Poses estimated by odometry (in magenta)
    drawPoses(dbg_map,
              odo_robot_pose.x, odo_robot_pose.y, odo_robot_pose.theta,
              MAP_RESOLUTION, (255, 0, 255))
    # Poses estimated by particle filter (in blue)
    drawPoses(dbg_map, pose_estimate.x, pose_estimate.y, pose_estimate.theta,
              MAP_RESOLUTION, (255, 0, 0))

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
                  (255, round(255*(1.0-particles_weight.item(j)/max_weight)), 0))

    # Redraw estimate by particle filter (in blue), to stay on top
    drawPoses(dbg_map,
              pose_estimate.x, pose_estimate.y, pose_estimate.theta,
              MAP_RESOLUTION, (255, 0, 0))

    # Show map with poses
    plt.figure("Debug")
    plt.cla()
    plt.imshow(tmp_map)
    plt.pause(0.01)


def ParticleFilterStep1(distance: float, dtheta: float):
    '''
    Implement step 1 of the particle filter - Prediction
    '''
    global NUM_PARTICLES, SIGMA1, SIGMA2, MAP_LENGTH, MAP_RESOLUTION, org_map
    global particles_x, particles_y, particles_theta, particles_weight

    if STEP_PREDICTION:
        '''
        Δd = Δd + Δd*normal_noise(sigma1)
        Δθ = Δθ + Δθ*normal_noise(sigma2)

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
            rnd_distance = random.gauss(distance, SIGMA1)
            rnd_dtheta = random.gauss(dtheta, SIGMA2)
            delta_X = rnd_distance*cos(particles_theta[n])
            delta_Y = rnd_distance*sin(particles_theta[n])

            particles_x[n] += delta_X
            particles_y[n] += delta_Y
            particles_theta[n] = rnd_dtheta

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


def realPoseCallback(msg: Odometry):
    '''
     Store real, error-free pose values given by the simulator (for debugging
    puposes only) --> DO NOT USE THIS FOR ANYTHING ELSE
    '''
    global real_pose

    real_pose.x = msg.pose.pose.position.x
    real_pose.y = msg.pose.pose.position.y
    real_pose.theta = quaternionToYaw(msg.pose.pose.orientation)


def odomCallback(msg: Odometry):
    '''
    Called when we get odometry messages
    '''
    global odo_robot_pose, odo_lin_vel, odo_ang_vel, odom_updated
    global odom_first_update, last_step1_time

    old_pose = odo_robot_pose

    # Store updated pose values
    odo_robot_pose.x = msg.pose.pose.position.x
    odo_robot_pose.y = msg.pose.pose.position.y
    odo_robot_pose.theta = quaternionToYaw(msg.pose.pose.orientation)
    # Store velocity computed from odometry
    odo_lin_vel = msg.twist.twist.linear.x
    odo_ang_vel = msg.twist.twist.angular.z
    # Store timestamp and update flag
    odom_updated = True

    # Only perform the particle filter update step if we have alread received
    # an odometry message in the past.
    if not odom_first_update:
        # Perform Step 1 of the particle filter
        # --> Update particles with robot movement:
        # We need to obtain the robot movement in the robot coordinates
        local_pose = ft.world2LocalP(old_pose, odo_robot_pose)
        ParticleFilterStep1(local_pose.x, local_pose.theta)
    else:
        odom_first_update = False  # First update is done

    last_step1_time = msg.header.stamp.to_sec()

    # Show updated particles
    #showDebugInformation()


def markersCallback(msg: Markers):
    '''
    Called whenever we have markers detected
    '''
    global odom_first_update, last_step1_time, odo_lin_vel, odo_ang_vel
    global NUM_PARTICLES, DISTANCE_ERROR_GAIN, odo_robot_pose, pose_estimate
    global max_weight, iteration, prev_time
    global old_particles_x, old_particles_y, old_particles_theta
    global particles_x, particles_y, particles_theta, particles_weight

    # This callback only makes sense if we have more than one marker
    if msg.num_markers < 1:
        return

    # Before using the markers information, we need to update the estimated
    # pose considering the amount of time that elapsed since the last update
    # from odometry.
    # We will use the linear and angular velocity of the robot for that.

    ''' Step 1 - Update particles given the robot motion: '''
    if not odom_first_update:
        dt = msg.header.stamp.to_sec() - last_step1_time
        distance = odo_lin_vel*dt  # Distance travelled
        dtheta = odo_ang_vel*dt  # Rotation performed
        ParticleFilterStep1(distance, dtheta)
        last_step1_time = msg.header.stamp.to_sec()

        # Store updated odometry pose
        odo_robot_pose.x += distance*cos(odo_robot_pose.theta)
        odo_robot_pose.y += distance*sin(odo_robot_pose.theta)
        odo_robot_pose.theta += dtheta

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

        # Reset normalization factor
        norm_factor = 0
        for j in range(0, NUM_PARTICLES):
            # Compute the weight for each particle
            # For each obtained beacon value
            particles_weight[j] = 0
            for n in range(0, msg.num_markers):
                # Obtain beacon position in world coordinates
                particle = Pose2D(particles_x.item(j),
                                  particles_y.item(j),
                                  particles_theta.item(j))
#                particle = Pose2D(particles_x[j],
#                                   particles_y[j],
#                                   particles_theta[j])

                marker_lpos = ft.Point2D(msg.range[n]*cos(msg.bearing[n]),
                                         msg.range[n]*sin(msg.bearing[n]))
                marker_wpos = ft.local2Worldp(particle, marker_lpos)

                particles_weight[j] += \
                    1.0/(1+sqrt((markers_wpos[msg.id[n]-1].x -
                                 marker_wpos.x)**2 +
                                (markers_wpos[msg.id[n]-1].y -
                                 marker_wpos.y)**2) * DISTANCE_ERROR_GAIN)

            # Perform the mean. We summed all elements above and now divide by
            # the number of elements summed.
            particles_weight[j] /= msg.num_markers
            # Update the normalization factor
            norm_factor += particles_weight[j]

        # Normalize the weight
        max_weight = 0.0
        for j in range(0, NUM_PARTICLES):
            particles_weight[j] /= norm_factor
            # Store the particle with the best weight has our posture estimate
            if particles_weight[j] > max_weight:
                pose_estimate.x = particles_x.item(j)
                pose_estimate.y = particles_y.item(j)
                pose_estimate.theta = particles_theta.item(j)
                # This max_factor is just used for debug, so that we have more
                # different colors between particles.
                max_weight = particles_weight.item(j)

    # Show debug information
    showDebugInformation()

    ''' Step 3 - Resample '''
    if STEP_RESAMPLE:
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
    iteration += 1
    if curr_time - prev_time >= DELTA_DEBUG:
        prev_time = curr_time
        plt.imsave('mapa.png', dbg_map)
        iteration = 0


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
                ang_vel = radians(30.0)  # Rotate left
            else:
                ang_vel = radians(-30.0)  # Rotate right

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
    map_file_path += '/ros/src/mystage_ros/world/AR/cave_walls_only.png'
    cave_map = (plt.imread(map_file_path)*255).astype(np.uint8)

    # Read original map and resize it to our resolution (in color)
    dbg_map[:, :, 0] = cave_map.copy()
    dbg_map[:, :, 1] = cave_map.copy()
    dbg_map[:, :, 2] = cave_map.copy()

    # Read original map and resize it to our resolution
    # We need the original map, black and white, so that it is faster to find
    # occupied cells
    org_map = cave_map.copy()

    # 
    #matplotlib.use('GTK3Cairo')
    #plt.pause(0.01)
    plt.switch_backend('GTK3Cairo')
    #plt.switch_backend('QT5Agg')

    # Create window for the map
    fig = plt.figure("Debug")
    plt.imshow(dbg_map)
    plt.pause(0.01)
    fig.canvas.mpl_connect('key_press_event', on_key)

    '''
    Particle filter related variables with their initial values
    '''
    # Initialize particles with uniform random distribution across all space
    particles_x = np.random.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER,
                                    MAP_LENGTH/2.0-SAFETY_BORDER,
                                    particles_x.shape)  # X
    particles_y = np.random.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER,
                                    MAP_LENGTH/2.0-SAFETY_BORDER,
                                    particles_y.shape)  # Y
    particles_theta = np.random.uniform(-pi, pi,
                                        particles_theta.shape)  # Theta

    # Re-add particles that are outside of the map or inside an obstacle, until
    # it is in a free space inside the map.
    for n in range(0, NUM_PARTICLES):
        while((particles_x[n] > MAP_LENGTH/2.0) or
              (particles_x[n] < -MAP_LENGTH/2.0) or
              (particles_y[n] > MAP_LENGTH/2.0) or
              (particles_y[n] < -MAP_LENGTH/2.0) or
              (org_map[int(np.rint(org_map.shape[0]/2.0
                                   - particles_y[n]/MAP_RESOLUTION)),
                       int(np.rint(org_map.shape[1]/2.0
                                   + particles_x[n]/MAP_RESOLUTION))] < 127)):
            particles_x[n] = random.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER,
                                            MAP_LENGTH/2.0-SAFETY_BORDER)
            particles_y[n] = random.uniform(-MAP_LENGTH/2.0+SAFETY_BORDER,
                                            MAP_LENGTH/2.0-SAFETY_BORDER)

    '''
    Random navigation with obstacle avoidance and particle filter-based
    localization
    '''
    print('Random navigation with obstacle avoidance and particle filter ' +
          ' based localization\n---------------------------')

    robot_name = '/robot_0'

    # Init ROS
    rospy.init_node('tw08', anonymous=True)

    # Setup subscribers
    # Odometry
    sub_odom = rospy.Subscriber(robot_name + '/odom', Odometry,
                                odomCallback, queue_size=10)
    # Real, error-free robot pose (for debug purposes only)
    sub_real_pose = rospy.Subscriber(robot_name+'/base_pose_ground_truth',
                                     Odometry, realPoseCallback, queue_size=1)
    # Laser scans
    sub_laser = rospy.Subscriber(robot_name + "/base_scan", LaserScan,
                                 laserCallback, queue_size=1)
    # Markers detected
    sub_markers = rospy.Subscriber(robot_name + "/markers", Markers,
                                   markersCallback, queue_size=1)

    # Setup publisher
    vel_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=1)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    # Wait until we get one odometry message
    while(not odom_updated):
        rate.sleep()

    # Stop the robot (if not stopped already)
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # "Loop" until shutdown
    #while not rospy.is_shutdown():
    #    plt.pause(0.01)

    #plt.show()
    rospy.spin()

    # Stop the robot
    vel_cmd.angular.z = 0
    vel_cmd.linear.x = 0
    vel_pub.publish(vel_cmd)

    # Close file
    outfile.close()

    # Store final map
    plt.imsave('mapa.png', dbg_map)
    plt.close('all')  # Close all figures
    sys.exit(0)
