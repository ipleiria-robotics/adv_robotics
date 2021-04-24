#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
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
import threading
import pytransform3d.rotations as pyrot
import pytransform3d.transformations as pytr


# ROS API
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped, PoseArray, \
    Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import message_filters
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# Our functions
from tw07.utils import quaternionToYaw, rpyToQuaternion, normalize, \
    meter2cell, CustomDuration
import LocalFrameWorldFrameTransformations as lfwft
from markers_msgs.msg import Markers

# Specify if the particle filter steps should run
RUN_PREDICTION_STEP = True
RUN_UPDATE_STEP = True
RUN_RESAMPLE_STEP = True

# Debug related variables
DELTA_DEBUG = 0.5  # Show debug information ony once every DELTA_PRINT seconds

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = 1.57  # 90º/s (in rad/s)


class TfListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.tf_buffer = tf2_ros.Buffer(cache_time=CustomDuration(sec=10))
        self.listener = tf2_ros.TransformListener(self.tf_buffer, node=self)
        self.get_logger().info("Started tf listener thread")


class ParticleFilter(Node):
    '''
    Localization using a Particle Filter
    '''
    def __init__(self, tf_buffer):
        '''
        Initialize class instance.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Robot name
        self.robot_name = 'robot_0'

        #
        #  Particle filter constants
        #
        # Number of particles used in the particle filter
        self.NUM_PARTICLES = 200
        # Gain factor for the weights given the distance
        self.DISTANCE_ERROR_GAIN = 0.5
        # Gain factor when computing the weights from the angle
        self.ANGLE_ERROR_GAIN = 5.0
        # Distance error standard deviation [m]
        self.SIGMA1 = 0.2
        # Theta error standard deviation [rad]
        self.SIGMA2 = radians(3.0)

        # General global variables
        self.outfile = open('Output.txt', 'w')  # Save robot poses to this file
        self.odo_robot_pose = Pose2D()
        # True if the odometry was updated at least once
        self.odom_updated_once = False
        # Real robot pose from the simulator (for debugging purposes only)
        self.real_pose = Pose2D()

        self.outfile.write(
            'Estimated pose of the robot\n\n' +
            '[T]: Odometry [X Y Theta] Particle Filter [X Y Theta]\n\n')

        # Particle-filter global variables
        # To store the estimation result
        self.robot_estimated_pose = Pose2D()
        # Vector of NUM_PARTICLES particles, where each particle corresponds to
        # a possible robot pose (X,Y,Theta).
        self.particles_x = np.empty(self.NUM_PARTICLES)
        self.particles_y = np.empty(self.NUM_PARTICLES)
        self.particles_theta = np.empty(self.NUM_PARTICLES)
        # Particles weight
        self.particles_weight = np.full(self.NUM_PARTICLES,
                                        1.0/self.NUM_PARTICLES)
        # The old particles vectors will be needed in the resampling algorithm
        self.old_particles_x = np.empty(self.NUM_PARTICLES)
        self.old_particles_y = np.empty(self.NUM_PARTICLES)
        self.old_particles_theta = np.empty(self.NUM_PARTICLES)
        self.max_weight = 1.0  # Weight of the best particle (to be updated)

        # Control the time between particles publishing
        self.prev_time = 0.0

        # Store the positions of the landmarks [m]
        self.markers_wpos = [lfwft.Point2D(-8.0,  4.0),  # 1
                             lfwft.Point2D(-8.0, -4.0),  # 2
                             lfwft.Point2D(-3.0, -8.0),  # 3
                             lfwft.Point2D(3.0, -8.0),  # 4
                             lfwft.Point2D(8.0, -4.0),  # 5
                             lfwft.Point2D(8.0, 4.0),  # 6
                             lfwft.Point2D(3.0, 8.0),  # 7
                             lfwft.Point2D(-3.0, 8.0)]  # 8
        # Max/min x/y values of the markers
        self.min_x = -8.0
        self.max_x = 8.0
        self.min_y = -8.0
        self.max_y = 8.0

        # Initialize the node itself
        super().__init__('tw07_particle_filter')

        # Create TF buffer and listener (for odom->base_link)
        self.tf_buffer = tf_buffer
        # Create broadcaster for map->base_link
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Estimated pose publisher
        self.pose_pub = self.create_publisher(PoseStamped,
                                              f'/{self.robot_name}/pose', 1)

        # Particles publisher
        self.particles_pub = self.create_publisher(
            PoseArray, f'/{self.robot_name}/particles', 1)

        # Subscribe the map topic (needed to consider the obstacles)
        # Since the map is only published when the map server starts, we need
        # to get the message that was last pubslihed, even if it as published
        # before we subscribed the topic. To enable that behavior so, we
        # specify the TRANSIENT_LOCAL Durability Policy.
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map',
                                                self.map_cb, qos_profile)

        # The makers and odometry subscription will be made only once we
        # receive a map

    def __del__(self):
        ''' Destructor, which will be called when the instance is destroyed.'''
        # Close file before leaving
        self.outfile.close()

    def map_cb(self, msg: OccupancyGrid):
        '''
        Receive an Occupancygrid type message with the map and store an
        internal copy of it.
        (Re)generate the initial particle distribution
        '''

        # Avoid simultaneous access to the map and particle filter
        self.lock.acquire()

        # Store map information internally
        self.map_origin = msg.info.origin
        self.map_resolution = msg.info.resolution
        self.occupied_thresh = 50
        self.occgrid = np.reshape(np.asanyarray(msg.data),
                                  (msg.info.height, msg.info.width))
        self.base_frame_id = msg.header.frame_id
        self.get_logger().info('Got and stored local copy of the map')

        '''
        Particle filter related variables with their initial values
        '''
        # Initialize particles with uniform random distribution across all
        # space (X, Y, Theta)
        self.particles_x = np.random.uniform(self.min_x, self.max_x,
                                             self.particles_x.shape)
        self.particles_y = np.random.uniform(self.min_y, self.max_y,
                                             self.particles_y.shape)
        self.particles_theta = np.random.uniform(-pi, pi,
                                                 self.particles_theta.shape)

        # Re-add particles that are inside an obstacle, until not particle is
        # inside an obstacle. This could be implemented in fewer lines with no
        # cycles, but is kept as is for increased clarity.
        for n in range(0, self.NUM_PARTICLES):
            while True:
                map_coord = meter2cell(
                    lfwft.Point2D(x=self.particles_x[n],
                                  y=self.particles_y[n]),
                    self.map_origin, self.map_resolution)

                # If this is not within the map or there an obstacle here
                if (map_coord.y < 0) or \
                   (map_coord.y >= self.occgrid.shape[0]) or \
                   (map_coord.x < 0) or \
                   (map_coord.x >= self.occgrid.shape[1]) or \
                   self.occgrid[map_coord.y, map_coord.x] > \
                   self.occupied_thresh:
                    # Particle is no good, generate new position for it
                    self.particles_x[n] = \
                        random.uniform(self.min_x, self.max_x)
                    self.particles_y[n] = \
                        random.uniform(self.min_y, self.max_y)
                else:
                    # Particle is good, proceed to the next on
                    break
        self.lock.release()
        self.get_logger().info('The Particle Filter has been initialized.')

        # Setup subscribers using a ApproximateTimeSynchronizer filter for the
        # odometry and the markers. We want to estimate the robot pose which is
        # closest in time from the published odometry and markers.
        # Setup odometry subscriber
        self.sub_odom = message_filters.Subscriber(
            self, Odometry, f'/{self.robot_name}/odom')
        # Detected landmarks
        self.sub_markers = message_filters.Subscriber(
            self, Markers, f'{self.robot_name}/markers')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_markers], 3, 0.2)
        ts.registerCallback(self.odomMarkersCallback)

    def odomMarkersCallback(self, odom_msg: Odometry, markers_msg: Markers):
        '''
        This callback is called whenever we have an odometry message and a
        markers message. The messages are received here simultaneously and
        correspond to (approximately) the same time stamp.
        '''

        ''' Step 1 - Update particles given the robot motion: '''
        new_odo_robot_pose = Pose2D(
            x=odom_msg.pose.pose.position.x,
            y=odom_msg.pose.pose.position.y,
            theta=quaternionToYaw(odom_msg.pose.pose.orientation))
        if RUN_PREDICTION_STEP and self.odom_updated_once:
            # Get estimated motion (Δd and Δθ) from odometry
            local_pose = lfwft.world2LocalP(self.odo_robot_pose,
                                            new_odo_robot_pose)
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

            # This could be vectorized to avoid the for loop, which would allow
            # highly reducing the time needed to execute!
            for n in range(0, self.NUM_PARTICLES):
                rnd_distance = random.gauss(distance, distance*self.SIGMA1)
                rnd_dtheta = random.gauss(dtheta, dtheta*self.SIGMA2)
                delta_X = rnd_distance*cos(self.particles_theta[n])
                delta_Y = rnd_distance*sin(self.particles_theta[n])

                self.particles_x[n] += delta_X
                self.particles_y[n] += delta_Y
                self.particles_theta[n] += rnd_dtheta

                # Normalize the orientation
                self.particles_theta[n] = normalize(self.particles_theta[n])

                # Check if any of particles is outside the map. If so, move
                # that particle to the previous position (the orientation
                # update is maintained).
                if (self.particles_x[n] < self.min_x) or \
                   (self.particles_x[n] > self.max_x) or \
                   (self.particles_y[n] < self.min_y) or \
                   (self.particles_y[n] > self.max_y):
                    # Undo this particle update (position only)
                    self.particles_x[n] -= delta_X
                    self.particles_y[n] -= delta_Y
                    continue  # No need to check for obstacles in this case

                with self.lock:
                    # If we already got the map, make sure that no particles
                    # end up inside obstacles
                    map_coord = meter2cell(
                        lfwft.Point2D(x=self.particles_x[n],
                                      y=self.particles_y[n]),
                        self.map_origin, self.map_resolution)
                    if (map_coord.y < 0) or \
                       (map_coord.y >= self.occgrid.shape[0]) or \
                       (map_coord.x < 0) or \
                       (map_coord.x >= self.occgrid.shape[1]) or \
                       self.occgrid[map_coord.y, map_coord.x] > \
                       self.occupied_thresh:
                        # Undo this particle update (position only)
                        self.particles_x[n] -= delta_X
                        self.particles_y[n] -= delta_Y

        elif not self.odom_updated_once:
            self.odom_updated_once = True

        # Store current odometry value
        self.odo_robot_pose = new_odo_robot_pose

        #
        # Steps 2 (observation, computing the weights) and 3 (resampling) only
        # run if we got a markers' message.
        #
        ''' Step 2 - Update the particle weights given the sensor model and map
        knowledge '''
        if RUN_UPDATE_STEP:
            '''
            For now we only perform this step if there was any marker detected.
            We could use the expected and not detected beacons for each
            particle, but it would become too expensive.

            The weight for each particle will be:
            w(j) = mean(1/(1+sqrt((x_e(i)-x_r(i))^2+(y_e(i)-y_r(i))^2) *
                        DISTANCE_ERROR_GAIN))
            where x_e(i)/y_e(i) and x_r(i)/y_r(i) are the expected and obtained
            x/y world coordinates of the detected marker, respectively. The
            GAIN are constant gains wich can be tuned to value smaller
            detection errors.
            '''
            if markers_msg.num_markers >= 1:
                run_steps23 = True
                # Reset normalization factor
                norm_factor = 0
                for j in range(0, self.NUM_PARTICLES):
                    # Compute the weight for each particle
                    # For each obtained beacon value
                    self.particles_weight[j] = 0
                    for n in range(0, markers_msg.num_markers):
                        # Obtain beacon position in world coordinates
                        particle = Pose2D(x=self.particles_x[j],
                                          y=self.particles_y[j],
                                          theta=self.particles_theta[j])
                        marker_lpos = lfwft.Point2D(
                            markers_msg.range[n] * cos(markers_msg.bearing[n]),
                            markers_msg.range[n] * sin(markers_msg.bearing[n]))
                        marker_wpos = lfwft.local2Worldp(particle, marker_lpos)

                        self.particles_weight[j] += 1.0/(1+sqrt(
                            (self.markers_wpos[markers_msg.id[n]-1].x -
                                marker_wpos.x)**2 +
                            (self.markers_wpos[markers_msg.id[n]-1].y -
                                marker_wpos.y)**2) *
                            self.DISTANCE_ERROR_GAIN)

                    # Perform the mean. We summed all elements above and now
                    # divide by the number of elements summed.
                    self.particles_weight[j] /= markers_msg.num_markers
                    # Update the normalization factor
                    norm_factor += self.particles_weight[j]
            else:
                run_steps23 = False

            # Normalize the weight
            if run_steps23:
                # If no marker was detected, the weigth did not change, so no
                # need to normalize it again
                self.particles_weight = self.particles_weight/norm_factor
            max_weight = 0.0

            for j in range(0, self.NUM_PARTICLES):
                # Store the particle with the best weight as our pose
                # estimate.
                if self.particles_weight[j] > max_weight:
                    self.robot_estimated_pose.x = self.particles_x[j]
                    self.robot_estimated_pose.y = self.particles_y[j]
                    self.robot_estimated_pose.theta = \
                        self.particles_theta[j]
                    # This max_factor is just used for debug, so that we
                    # have more different colors between particles.
                    max_weight = self.particles_weight[j]
            # self.get_logger().info(f'Max weight={max_weight}')

            # Publish the TF fom map to odom
            self.publishMapOdomTF(odom_msg.header.stamp)

            # Publish the estimated pose message. It needs to be
            # PoseStamped, a 3D pose with a timestamp. We will create one
            # from the robot_estimated_pose.
            pose_to_publish = PoseStamped()
            pose_to_publish.header.frame_id = self.base_frame_id
            pose_to_publish.header.stamp = odom_msg.header.stamp
            pose_to_publish.pose.position = \
                Point(x=self.robot_estimated_pose.x,
                      y=self.robot_estimated_pose.y,
                      z=0.)
            pose_to_publish.pose.orientation = \
                rpyToQuaternion(0., 0., self.robot_estimated_pose.theta)
            self.pose_pub.publish(pose_to_publish)

        ''' Step 3 - Resample '''
        if RUN_RESAMPLE_STEP and run_steps23:
            '''
            The resampling step is the exact implementation of the algorithm
            described in the theoretical classes, i.e., the "Importance
            resampling algorithm".
            '''

            # Save the current values
            # The old particles will be needed in the resampling algorithm
            old_particles_x = self.particles_x.copy()
            old_particles_y = self.particles_y.copy()
            old_particles_theta = self.particles_theta.copy()
            old_particles_weight = self.particles_weight.copy()

            delta = random.random()/self.NUM_PARTICLES
            c = old_particles_weight[0]
            i = 0
            for j in range(0, self.NUM_PARTICLES):
                u = delta + j/(1.0*self.NUM_PARTICLES)
                while u > c:
                    i += 1
                    c += old_particles_weight[i]
                self.particles_x[j] = old_particles_x[i]
                self.particles_y[j] = old_particles_y[i]
                self.particles_theta[j] = old_particles_theta[i]
                # The weight is indicative only, it will be recomputed on
                # marker detection
                self.particles_weight[j] = old_particles_weight[i]

        '''
        Particle filter steps end here
        '''

        # Publish debug information from time to time
        if odom_msg.header.stamp.sec - self.prev_time > DELTA_DEBUG:
            self.prev_time = odom_msg.header.stamp.sec

            # Write data to the file
            self.outfile.write(f'{odom_msg.header.stamp.sec:.0f}.' +
                               f'{odom_msg.header.stamp.nanosec:.0f}: ' +
                               f'{self.odo_robot_pose.x:.2f} ' +
                               f'{self.odo_robot_pose.y:.2f} ' +
                               f'{self.odo_robot_pose.theta:.2f} ' +
                               f'{self.robot_estimated_pose.x:.2f} ' +
                               f'{self.robot_estimated_pose.y:.2f} ' +
                               f'{self.robot_estimated_pose.theta:.2f}')
            # Publish the particles
            self.publishParticles(odom_msg.header.stamp)

    def publishMapOdomTF(self, stamp):
        '''
        Given an estimated pose, publish the map->odom transform.
        '''

        # We want to know the map->base_footprint transform, but a frame cannot
        # have two parents. As such, we need to have map->odom->base_footprint,
        # with the odom->base_link transform being published by the robot
        # low-level software, and the map->odom being published here by us.

        try:
            # Get the transformation from odom to base_footprint
            odom_to_base_footprint_trans = self.tf_buffer.lookup_transform(
                f'{self.robot_name}/base_footprint',
                f'{self.robot_name}/odom', stamp, Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'No required transformation found: {e}')
            return
        
        # Get the odom to base_fooprint matrix transformation from the
        # transformation computed above
        odom_to_base_footprint_tf = pytr.transform_from_pq(
            [odom_to_base_footprint_trans.transform.translation.x,
             odom_to_base_footprint_trans.transform.translation.y,
             odom_to_base_footprint_trans.transform.translation.z,
             odom_to_base_footprint_trans.transform.rotation.w,
             odom_to_base_footprint_trans.transform.rotation.x,
             odom_to_base_footprint_trans.transform.rotation.y,
             odom_to_base_footprint_trans.transform.rotation.z])

        # Get the transformation matrix from the base_footprint to the map from
        # the particle filter estimate
        base_footprint_to_map_tf = pytr.transform_from(
            pyrot.active_matrix_from_angle(2, self.robot_estimated_pose.theta),
            [self.robot_estimated_pose.x, self.robot_estimated_pose.y, 0.0])

        # Get the transformation (point and quaternion) from odom to map
        odom_to_map_pq = pytr.pq_from_transform(pytr.concat(
                odom_to_base_footprint_tf,
                base_footprint_to_map_tf))

        # Publish transformation from odom to map (map->odom link)
        odom_to_map_trans_stamped = tf2_ros.TransformStamped()
        odom_to_map_trans_stamped.header.stamp = stamp
        odom_to_map_trans_stamped.header.frame_id = self.base_frame_id
        odom_to_map_trans_stamped.child_frame_id = f'{self.robot_name}/odom'
        odom_to_map_trans_stamped.transform.translation.x = odom_to_map_pq[0]
        odom_to_map_trans_stamped.transform.translation.y = odom_to_map_pq[1]
        odom_to_map_trans_stamped.transform.translation.z = odom_to_map_pq[2]
        odom_to_map_trans_stamped.transform.rotation = Quaternion(
            x=odom_to_map_pq[4],
            y=odom_to_map_pq[5],
            z=odom_to_map_pq[6],
            w=odom_to_map_pq[3])
        self._tf_broadcaster.sendTransform(odom_to_map_trans_stamped)

    def publishParticles(self, timestamp):
        '''
        Debug information function.
        Given the list of particles, publish a list of the corresponding poses.
        We will also publish an image with the particles colored by their
        weights.
        '''
        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame_id
        pose_array.header.stamp = timestamp
        for n in range(0, self.NUM_PARTICLES):
            pose = Pose(position=Point(x=self.particles_x[n],
                                       y=self.particles_y[n],
                                       z=0.0),
                        orientation=rpyToQuaternion(0., 0.,
                                                    self.particles_theta[n]))
            pose_array.poses.append(pose)
        self.particles_pub.publish(pose_array)


def main(args=None):
    '''
    Main function
    Use particle filter for localization.
    '''

    print('Particle filter-based localization\n---------------------------')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    tf_listener_node = TfListener()
    particle_filter_node = ParticleFilter(tf_buffer=tf_listener_node.tf_buffer)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_listener_node)
    executor.add_node(particle_filter_node)

    executor.spin()
    executor.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')
