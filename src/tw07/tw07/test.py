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

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped, PoseArray
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
        self._tf_publisher = tf2_ros.TransformBroadcaster(self)

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

        # Publish the TF fom map to odom
        self.publishMapOdomTF(odom_msg.header.stamp)

        '''
        Particle filter steps end here
        '''

    def publishMapOdomTF(self, stamp):
        '''
        Given an estimated pose, publish the map->odom transform.
        '''

        # We want to know the map->base_link transform, but a frame cannot have
        # two parents. As such, we need to have map->odom->base_link, with the
        # odom->base_link transform being published by the robot low-level
        # software, and the map->odom being published here by us.

        # Get the odom->base_link trasform
        #print(self.tf_buffer.all_frames_as_yaml())
        try:
            tf = self.tf_buffer.lookup_transform(
                f'{self.robot_name}/base_footprint',
                f'{self.robot_name}/odom', stamp, Duration(seconds=0.1))
            self.get_logger().info('Got odom transform')
            #laser_rotation = quaternion_to_euler(tf.transform.rotation)[0]
            #laser_translation = tf.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'No required transformation found: {e}')
            return
        

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


class TfListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.tf_buffer = tf2_ros.Buffer(cache_time=CustomDuration(sec=20))
        self.listener = tf2_ros.TransformListener(self.tf_buffer, spin_thread=False, node=self)
        self.get_logger().info("Started tf listener thread")


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
