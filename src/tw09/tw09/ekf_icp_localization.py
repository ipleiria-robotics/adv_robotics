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


# Library packages needed
import threading
import pytransform3d.rotations as pyrot
import pytransform3d.transformations as pytr
import numpy as np
from math import atan2, cos, degrees, inf, sin, sqrt
import cv2
from scipy.spatial import cKDTree
import sys

# ROS API
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped, \
    PoseWithCovarianceStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import ros2_numpy as rnp

# Our functions
from tw09.ExtendedKalmanFilter import ExtendedKalmanFilter
from tw07.utils import cell2meter, quaternionToYaw, rpyToQuaternion
import tw07.LocalFrameWorldFrameTransformations as lfwft
from markers_msgs.msg import Markers

# Specify if the particle filter steps should run
RUN_PREDICTION_STEP = False
RUN_OBSERVATION_STEP = False

# Amount of debug information to show
FULL_ICP_DEBUG = True  # Show information on every ICP step


class TfListener(Node):
    '''
    Class used to keep an updated internal copy of the TF tree.
    '''
    def __init__(self):
        '''
        Initialize class instance.
        '''
        super().__init__('tf_listener')
        # Create the TF tree buffer to store all the information, keeping track
        # of the last 10 seconds (cache_time)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # Create the TF listener which will receive the TFs
        self.listener = tf2_ros.TransformListener(self.tf_buffer, node=self)
        self.get_logger().info('Started TF listener thread.')


class EKFICPLocalization(Node):
    '''
    Localization using a.
    '''
    def __init__(self, tf_buffer):
        '''
        Initialize class instance.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Robot name
        self.robot_name = 'robot_0'

        # Initialize the node itself
        super().__init__('ekf_localization')

        # The actual Extended Kalman Filter
        self.ekf = ExtendedKalmanFilter()

        # General global variables
        self.outfile = open('Output.txt', 'w')  # Save robot poses to this file
        # Will store the latest odometry pose
        self.odom_robot_pose = Pose2D()
        # True if the odometry was updated at least once
        self.odom_updated_once = False

        # Send initial information to the output file
        self.outfile.write(
            'Estimated pose of the robot\n\n' +
            '[T]: Odometry [X Y Theta] Extended Kalman Filter [X Y Theta]\n\n')

        # ICP related constants and variables
        # Save ICP debug information to this file
        self.icpfile = open('ICP.txt', 'w')
        # Maximum number of ICP iterations
        self.MAX_ICP_ITERATIONS = 10
        # Minimum error difference to terminate ICP [m]
        self.MIN_ICP_DELTA_ERROR = 0.01
        # Minimum time between consecutive IPC runs
        self.MIN_ICP_DELTATIME = Duration(seconds=1.0, nanoseconds=0.0)
        # Timestamp of the last ICP run
        self.last_icp_call_time = self.get_clock().now()
        # Will hold the KD-Tree built from the environment map
        self.kdtree = None

        # To store the pose estimation result
        self.robot_estimated_pose = Pose2D()

        # Create TF buffer and listener (for odom->base_link)
        self.tf_buffer = tf_buffer
        # Create broadcaster for map->base_link
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Estimated pose publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self.robot_name}/pose', 2)

        # Markers array publisher (used for debugging ICP)
        self.markers_array_pub = self.create_publisher(
            MarkerArray, f'/{self.robot_name}/visualization_marker', 1)

        # We will subscribe only when a map is received
        self.sub_odom = None
        self.sub_laserscan = None

        # Subscribe the map topic (needed to generate the point cloud)
        # Since the map is only published when the map server starts, we need
        # to get the message that was last pubslihed, even if it was published
        # before we subscribed the topic. To enable that behavior so, we
        # specify the TRANSIENT_LOCAL Durability Policy.
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map',
                                                self.map_cb, qos_profile)

        self.get_logger().info('The EKF with ICP-based localization node is ' +
                               'starting. Waiting for the map...')

    def __del__(self):
        ''' Destructor, which will be called when the instance is destroyed.'''
        # Close file before leaving
        self.outfile.close()
        if FULL_ICP_DEBUG:
            self.icpfile.close()

    def publishDebugInformation(self, timestamp,
                                map_cloud, sensed_cloud,
                                map_cloud_center, sensed_cloud_center,
                                correspondences):
        '''
        Publish ICP related information for debugging purposes. The sensed
        original point clouds, their center of mass and the computed
        correspondances will be publised has a MarkerArray, which can be
        visualized in RViz. This information is only publisedh if there is at
        least one subscriber.
        All arguments, except the timestamp, are expected to be numpy arrays.
        All coordinates should be in world/map coordinates.
        '''

        if self.markers_array_pub.get_subscription_count() == 0:
            # If no node is receiving this information, then there is no need
            # to publish it. (This test could be done for all publishers.)
            return

        markers_array = MarkerArray()
        marker_id = 0
        scale = 0.1
        scale_vector = Vector3(x=scale, y=scale, z=scale)
        pose_origin = Pose(position=Point(x=0.0, y=0.0, z=0.0),
                           orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        # Original cloud test (green points list)
        # We could build this marker only once when the map is received!
        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = timestamp
        marker.type = Marker.POINTS
        marker.ns = self.robot_name
        marker.id = marker_id
        marker_id += 1
        marker.action = Marker.MODIFY
        marker.pose = pose_origin  # Points are in map coordinates
        marker.scale = scale_vector
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        # marker.lifetime = Duration()  # Keep until updated
        marker.frame_locked = False  # Do not update from TFs.
        marker.points = rnp.msgify(Point, map_cloud).tolist()
        markers_array.markers.append(marker)

        # Sensed cloud test (red points list)
        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = timestamp
        marker.type = Marker.POINTS
        marker.ns = self.robot_name
        marker.id = marker_id
        marker_id += 1
        marker.action = Marker.MODIFY
        marker.pose = pose_origin  # Points are in map coordinates
        marker.scale = scale_vector
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        # marker.lifetime = Duration()  # Keep until updated
        marker.frame_locked = False  # Do not update from TFs.
        marker.points = rnp.msgify(Point, sensed_cloud).tolist()
        markers_array.markers.append(marker)

        # Correspondance lines test (yellow lines list)
        pts = np.empty((sensed_cloud.shape[0] + correspondences.shape[0], 2),
                       dtype=sensed_cloud.dtype)
        pts[0::2, :] = sensed_cloud
        pts[1::2, :] = correspondences
        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = timestamp
        marker.type = Marker.LINE_LIST
        marker.ns = self.robot_name
        marker.id = marker_id
        marker_id += 1
        marker.action = Marker.MODIFY
        marker.pose = pose_origin  # Points are in map coordinates
        marker.scale.x = 0.5*scale
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        # marker.lifetime = Duration()  # Keep until updated
        marker.frame_locked = False  # Do not update from TFs.
        marker.points = rnp.msgify(Point, pts).tolist()
        markers_array.markers.append(marker)

        # Center of mass (original) test (green sphere)
        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = timestamp
        marker.type = Marker.SPHERE
        marker.ns = self.robot_name
        marker.id = marker_id
        marker_id += 1
        marker.action = Marker.MODIFY
        marker.pose = Pose(
            position=Point(x=map_cloud_center[0, 0],
                           y=map_cloud_center[0, 1],
                           z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.scale = Vector3(x=scale, y=scale, z=scale)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        # marker.lifetime = Duration()  # Keep until updated
        marker.frame_locked = False  # Do not update from TFs.
        markers_array.markers.append(marker)

        # Center of mass (sensed) test (red shpere)
        marker = Marker()
        marker.header.frame_id = self.base_frame_id
        marker.header.stamp = timestamp
        marker.type = Marker.SPHERE
        marker.ns = self.robot_name
        marker.id = marker_id
        marker_id += 1
        marker.action = Marker.MODIFY
        marker.pose = Pose(
            position=Point(x=sensed_cloud_center[0, 0],
                           y=sensed_cloud_center[0, 1],
                           z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.scale = Vector3(x=scale, y=scale, z=scale)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        # marker.lifetime = Duration()  # Keep until updated
        marker.frame_locked = False  # Do not update from TFs.
        markers_array.markers.append(marker)

        # Publish Markers Array
        self.markers_array_pub.publish(markers_array)

    def createAndPublishPose(self, timestamp):
        '''
        Create and publish a message with the estimated robot pose resulting
        from the application of the EKF with the ICP algorithm. The pose will
        only be published if there is at least one subscriber.
        Also call the map->odom TF broadcast function to publish the
        transformation.
        '''
        # Publish the TF fom map to odom
        self.publish_map_odom_tf(timestamp)

        # Update and send pose message (if there is at least one subscriber)
        if self.pose_pub.get_subscription_count() == 0:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.base_frame_id
        pose_msg.header.stamp = timestamp
        pose_msg.pose.pose.position.x = float(self.ekf.state[0, 0])
        pose_msg.pose.pose.position.y = float(self.ekf.state[1, 0])
        pose_msg.pose.pose.position.z = 0.0  # Robot is on the ground
        # Store robot orientation in quaternions by converting from RPY. In
        # this case we only have a rotation around the Z axis
        pose_msg.pose.pose.orientation = \
            rpyToQuaternion(0, 0, self.ekf.state[2, 0])

        # Fill covariance matrix. We will consider 0 for all the values we are
        # not computing, i.e., Z, RX and RY, and correlated values.
        pose_msg.pose.covariance = np.array([
            # X, Y, Z, RX, RY, RZ
            self.ekf.P[0, 0], self.ekf.P[0, 1], 0.0, 0.0, 0.0, self.ekf.P[0, 2],  # X
            self.ekf.P[1, 0], self.ekf.P[1, 1], 0.0, 0.0, 0.0, self.ekf.P[1, 2],  # Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RX
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RY
            self.ekf.P[2, 0], self.ekf.P[2, 1], 0.0, 0.0, 0.0, self.ekf.P[2, 2]])  # RZ

        # Publish message
        self.pose_pub.publish(pose_msg)

    def map_cb(self, msg: OccupancyGrid):
        '''
        Receive an Occupancygrid type message with the map and store an
        internal copy of it. Compute the corresponding point cloud and
        subscribe to the odometry and laser scan to start localizing.
        '''

        # Avoid simultaneous access to the map and particle filter
        self.lock.acquire()

        self.get_logger().info('Map received. Initializing point cloud...')

        # Store map information internally
        occgrid = np.reshape(np.asanyarray(msg.data),
                             (msg.info.height, msg.info.width))
        # We will use this frame_id as our map frame id
        self.base_frame_id = msg.header.frame_id

        ''' Initilize point cloud based on the received map '''
        # Get total number of points in the original map (this number will be
        # the size of the cloud).
        # Obstacles have value 100 in original map. We assume that any value
        # above or equal to 50 is an obstacle.
        num_map_points = np.count_nonzero(occgrid >= 50)
        # Prepare our cloud map with the correct number of points
        self.map_cloud = np.empty([num_map_points, 2])
        # Go through the image and store all the walls points in the cloud
        # NOTE: this 2 for cycles could be vectorized into few instructions for
        # increased execution speed. I leave it as it is for readibility
        i = 0
        for y in range(occgrid.shape[0]):
            for x in range(occgrid.shape[1]):
                # If this pixel is >= 500, then it is a wall point. Add it to
                # the cloud.
                if occgrid[y, x] >= 50:
                    pt = cell2meter(lfwft.Point2D(x=x, y=y), msg.info.origin,
                                    msg.info.resolution)
                    self.map_cloud[i, 0] = pt.x
                    self.map_cloud[i, 1] = pt.y
                    i = i + 1
        # Sanity check: the number of read points must be equal to the number
        # of expected points
        if i != num_map_points:
            self.get_logger().error(
                 f'Expected {num_map_points} points but got {i} points')
            self.lock.release()
            return  # No sense in continuing

        # Build KD-Tree for nearest neighbours search using our cloud map
        #  points
        self.kdtree = cKDTree(self.map_cloud)

        # Debug
        if FULL_ICP_DEBUG:
            # Output ICP data map to a file for debugging purposes
            testfile = open('data_for_ICP.txt', 'w')
            testfile.write('-- Data file to test ICP elsewhere:\n' +
                           '\n\n-- Original point cloud:\n')
            testfile.write(np.array2string(self.map_cloud,
                                           threshold=sys.maxsize))


        # Release access to shared data
        self.lock.release()

        self.get_logger().info('Point cloud is now initialized.')

        # Now that we have our map, we can start processing odometry and
        # markers information, so lets subsribe the corresponding topics.
        # Setup subscribers using a ApproximateTimeSynchronizer filter for the
        # odometry and the markers. We want to estimate the robot pose which is
        # closest in time from the published odometry and markers.
        # Do it only if not done yet.

        if (self.sub_odom is not None) and (self.sub_laserscan is not None):
            return

        # Setup odometry subscriber
        self.sub_odom = message_filters.Subscriber(
            self, Odometry, f'/{self.robot_name}/odom')
        # Laser scan
        self.sub_laserscan = message_filters.Subscriber(
            self, LaserScan, f'{self.robot_name}/base_scan')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_laserscan], 3, 0.2)
        ts.registerCallback(self.odom_laserscan_cb)

    def odom_laserscan_cb(self, odom_msg: Odometry, laser_msg: LaserScan()):
        '''
        This callback is called whenever we have an odometry message and a
        laser scan message. The messages are received here simultaneously and
        correspond to (approximately) the same time stamp.
        We can do this here because the messages are being published at the
        same frequency/time. In a real-world application, it would be better
        to have separate callbacks for the odometry and the laser scan, taking
        into account their corresponding timestamps.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock.acquire()

        ''' Step 1 - Prediction step using odometry values

            If this is the first time this function is being called, then use
            the odometry values as our best estimate for the robot starting
            pose.
            In this case, do not compute the prediction step, but simply
            initialize the EKF state with this value
        '''
        new_odom_robot_pose = Pose2D(
            x=odom_msg.pose.pose.position.x,
            y=odom_msg.pose.pose.position.y,
            theta=quaternionToYaw(odom_msg.pose.pose.orientation))
        if not self.odom_updated_once:
            # Initialize the EKF state with the initial odometry value
            self.ekf.state[0, 0] = new_odom_robot_pose.x
            self.ekf.state[1, 0] = new_odom_robot_pose.y
            self.ekf.state[2, 0] = new_odom_robot_pose.theta
            self.odom_updated_once = True  # We have received or first value
            # TODO: Initialize the filter covariance from the odometry one
        elif RUN_PREDICTION_STEP:  # Run only if step enabled
            # Compute the robot motion given the last know robot pose from
            # odometry and the current one
            local_pose = lfwft.world2LocalP(self.odom_robot_pose,
                                            new_odom_robot_pose)
            # Execute the actual prediction step
            self.ekf.predictStep(local_pose.x, local_pose.y, local_pose.theta)

        # Store current odometry as a reference for the next iteration
        self.odom_robot_pose = new_odom_robot_pose

        ''' Step 2 - Perform the update step based on the ICP algorithm result.
            The ICP algorithm will return an estimate for the current robot
            pose, starting in the last pose estimate.
            We will use that estimate as the sensor data for the observation /
            update step.
            This step runs at a smaller frequency then the prediction step,
            given that this part takes much longer.
        '''
        last_error = inf

        # Run the Observation step (which uses the ICP algorithm) only if
        # enough time has elapsed. We consider the data time as our reference
        # time.
        current_time = Time.from_msg(laser_msg.header.stamp)
        if current_time < self.last_icp_call_time + self.MIN_ICP_DELTATIME:
            self.lock.release()
            # In this case, publish the pose using the odometry timestamp
            self.createAndPublishPose(odom_msg.header.stamp)
            return  # To soon, return without running the observation step.
        # Else
        self.last_icp_call_time = current_time

        # Build sensed point cloud
        # This could be vectorized for increased speed. It is kept as it is for
        # increased readability.
        angle = laser_msg.angle_min
        # Compute the number of elements
        num_sensed_points = (
            np.logical_and(np.array(laser_msg.ranges) > laser_msg.range_min,
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
                j = j + 1  # We have one more point
            angle = angle + laser_msg.angle_increment
            i = i + 1  # Proceed to the next laser measurement
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
            self.get_logger().debug('--------------------------------')
            # Output ICP data to a file for debugging purposes
            testfile = open('data_for_ICP.txt', 'a')
            testfile.write('\n\n-- Robot pose:\n')
            testfile.write(np.array_str(self.ekf.state))
            testfile.write('\n\n-- Sensed point cloud:\n')
            testfile.write(np.array2string(sensed_cloud, threshold=sys.maxsize))
            testfile.close()

        # This variable will hold the total transformations (rotation and
        # translation), which will correspond to the estimated robot pose.
        totalRotation = np.array(
            [[cos(self.ekf.state[2, 0]), -sin(self.ekf.state[2, 0])],
             [sin(self.ekf.state[2, 0]),  cos(self.ekf.state[2, 0])]])
        totalTranslation = np.array([[self.ekf.state[0, 0]],
                                     [self.ekf.state[1, 0]]])
        # Store initial transformations (rotation and translation)
        rotation = totalRotation.copy()
        translation = totalTranslation.copy()

        ''' ICP - The actual ICP algorithm starts here '''
        for num_icp_iterations in range(self.MAX_ICP_ITERATIONS):
            # Debug
            if FULL_ICP_DEBUG:
                self.get_logger().debug(
                    f'-------------------> ICP iteration {num_icp_iterations}')

            # Update sensed cloud with last transformation. this will transform
            # it from local to global (map) coordinates.
            # We do sensed_cloud*rotation_matrix.T, since the elements are
            # row-wise.
            # Recall that rotation.T is the transponse of the rotation matrix
            sensed_cloud = sensed_cloud @ rotation.T
            # Translate
            sensed_cloud[:, 0] = sensed_cloud[:, 0] + translation[0, 0]
            sensed_cloud[:, 1] = sensed_cloud[:, 1] + translation[1, 0]

            # Compute correspondence using the KDTree nearest neighbour
            dists, indexes = self.kdtree.query(sensed_cloud)
            # Build vector with only the correspondant pairs.
            # Once again, this could be vectorized for increased speed.
            correspondences = self.map_cloud[indexes[:], :]

            # Compute the translation as the difference between the center of
            # mass of each set of points in the correspondence.
            # Compute mean along the rows (axis 0)
            mean_sensed_cloud = np.mean(sensed_cloud, 0, keepdims=True)
            mean_correspondences = np.mean(correspondences, 0, keepdims=True)

            # Debug
            if FULL_ICP_DEBUG:
                self.get_logger().debug(f'Sensed centroid: ' +
                                        f'{mean_sensed_cloud[0, 0]} ' +
                                        f'{mean_sensed_cloud[0, 1]}\n' +
                                        f'Data centroid: ' +
                                        f'{mean_correspondences[0, 0]} ' +
                                        f'{mean_correspondences[0, 1]}')

            # Compute the rotation by:
            #  1. Computing the inercia W
            #  2. Performing SVD as W = USV'
            #  3. Computing the rotation as R = VU'
            W = np.zeros((2, 2))
            for i in range(correspondences.shape[0]):
                W = W + (sensed_cloud[i, :] - mean_sensed_cloud).T @ \
                    (correspondences[i, :] - mean_correspondences)
            U, S, Vt = np.linalg.svd(W)
            # Debug
            if FULL_ICP_DEBUG:
                self.get_logger().debug(f'W matrix:\n{W}\n' +
                                        f'U matrix:\n{U}\n' +
                                        f'S matrix:\n{S}\n' +
                                        f'V matrix transposed:\n{Vt}')
            # For the SVD approach to always work, we use the approach detailed
            # in D.W. Eggert, A. Lorusso, R.B. Fisher, "Estimating 3-D rigid
            # body transformations: a comparison of four major algorithms",
            # Machine Vision and Applications, pp 272-290, Springer-Verlag,
            # 1997.
            S = np.eye(2, 2)
            S[1, 1] = np.linalg.det(U @ Vt)  # U * Vt
            rotation = Vt.T @ S @ U.T

            # Compute translation
            translation = \
                mean_correspondences.T - rotation @ mean_sensed_cloud.T

            # Update total transformation
            totalRotation = rotation @ totalRotation
            totalTranslation = rotation @ totalTranslation + translation

            # Debug
            if FULL_ICP_DEBUG:
                self.get_logger().debug(
                    f' Current rotation:\n{rotation}\n' +
                    f' Current translation:\n{translation}\n'
                    f' Current total rotation:\n{totalRotation}\n' +
                    f' Current total translation:\n{totalTranslation}\n' +
                    f'Total rotation angle [degrees]: ' +
                    f'{degrees(atan2(rotation[1, 0], rotation[0, 0]))}')
                # Publish debug information
                self.publishDebugInformation(odom_msg.header.stamp,
                                             self.map_cloud, sensed_cloud,
                                             mean_correspondences,
                                             mean_sensed_cloud,
                                             correspondences)

            # Compute RMS error (before last computed transformation)
            sqr_dists = dists**2  # Compute the square of the distances
            result = np.mean(sqr_dists)  # Compute the mean
            curr_error = sqrt(result)  # Compute the square root
            # Debug
            if FULL_ICP_DEBUG:
                self.get_logger().debug(f'Current error: {curr_error}')
                self.icpfile.write(f'{curr_error}')

            # The error should never increase!
            if curr_error <= last_error:
                # If the error variation is small enough, we're done
                if last_error - curr_error < self.MIN_ICP_DELTA_ERROR:
                    last_error = curr_error
                    break
            else:
                # Debug
                if FULL_ICP_DEBUG:
                    self.get_logger().debug('Got unexpected error increase!!!')
            last_error = curr_error

        # Debug
        self.get_logger().debug(
            'Final ICP pose (x, y, theta): ' +
            f'{totalTranslation[0, 0]:.2f} {totalTranslation[1, 0]:.2f} ' +
            f'{degrees(atan2(totalRotation[1, 0], totalRotation[0, 0])):.2f}')
        if FULL_ICP_DEBUG:
            self.icpfile.write(
             'Final ICP pose (x, y, theta): ' +
             f'{totalTranslation[0, 0]:.2f} {totalTranslation[1, 0]:.2f} ' +
             f'{degrees(atan2(totalRotation[1, 0], totalRotation[0, 0])):.2f}')
            self.icpfile.write('----------------------------')

        ''' Run the actual step 2 of the EKF, given the result of the above ICP
            run. For the error, we will also consider the result above. '''
        W = np.array([[last_error, 0.0, 0.0],
                      [0.0, last_error, 0.0],
                      [0.0, 0.0, last_error]])
        z = np.array([[totalTranslation[0, 0]],  # x
                      [totalTranslation[1, 0]],  # y
                      [atan2(totalRotation[1, 0], totalRotation[0, 0])]])  # θ
        self.ekf.updateStep(z, W)

        # Publish computed pose value
        self.createAndPublishPose(laser_msg.header.stamp)

        # Store result to a file
        self.outfile.write(f'{odom_msg.header.stamp.sec:.0f}.' +
                           f'{odom_msg.header.stamp.nanosec:.0f}: ' +
                           f'{self.odom_robot_pose.x:.2f} ' +
                           f'{self.odom_robot_pose.y:.2f} ' +
                           f'{self.odom_robot_pose.theta:.2f} ' +
                           f'{self.ekf.state[0, 0]:.2f} ' +
                           f'{self.ekf.state[1, 0]:.2f} ' +
                           f'{self.ekf.state[2, 0]:.2f}\n')
        
        # We no longer need access to the data
        self.lock.release()

    def publish_map_odom_tf(self, timestamp):
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
                f'{self.robot_name}/odom', timestamp, Duration(seconds=0.1))
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
            pyrot.active_matrix_from_angle(2, self.ekf.state[2, 0]),
            [self.ekf.state[0, 0], self.ekf.state[1, 0], 0.0])

        # Get the transformation (point and quaternion) from odom to map
        odom_to_map_pq = pytr.pq_from_transform(pytr.concat(
                odom_to_base_footprint_tf,
                base_footprint_to_map_tf))

        # Publish transformation from odom to map (map->odom link)
        odom_to_map_trans_stamped = tf2_ros.TransformStamped()
        odom_to_map_trans_stamped.header.stamp = timestamp
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


def main(args=None):
    '''
    Main function
    Use Extended Kalman Filter with Iteratic Closesy Point for localization.
    '''

    print('EKF-based localization with ICP\n-----------------------')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our TF listener node
    tf_listener_node = TfListener()
    # Create our Particle filter node
    ekf_icp_localization_node = EKFICPLocalization(
        tf_buffer=tf_listener_node.tf_buffer)

    # We will execute each node in its own thread. This is important to make
    # sure that the TF listener is continuously updated.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_listener_node)
    executor.add_node(ekf_icp_localization_node)

    # Run both nodes until shutdown
    executor.spin()
    executor.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')
