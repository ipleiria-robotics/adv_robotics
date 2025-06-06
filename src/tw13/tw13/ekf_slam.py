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
import transforms3d._gohlketransforms as trf
import numpy as np


# ROS API
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, Pose2D, PoseStamped, Quaternion, \
    Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import ros2_numpy as rnp

# Our functions
from tw13.ExtendedKalmanFilter import ExtendedKalmanFilter
from ar_py_utils.utils import quaternionToYaw, rpyToQuaternion
import ar_py_utils.LocalFrameWorldFrameTransformations as lfwft
from markers_msgs.msg import Markers

# Specify if the particle filter steps should run
RUN_PREDICTION_STEP = True
RUN_OBSERVATION_STEP = True

# Output debug information to the file only once every DELTA_PRINT seconds
DELTA_DEBUG = 1


class TfListener(Node):
    '''
    Class used to keep an updated internal copy of the TF tree.
    '''
    def __init__(self):
        '''
        Initialize class instance.
        '''
        super().__init__("tf_listener")
        # Create the TF tree buffer to store all the information, keeping track
        # of the last 10 seconds (cache_time)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # Create the TF listener which will receive the TFs
        self.listener = tf2_ros.TransformListener(self.tf_buffer, node=self)
        self.get_logger().info("Started TF listener thread.")


class EKFSLAM(Node):
    '''
    SLAM using an EKF
    '''
    def __init__(self, tf_buffer):
        '''
        Initialize class instance.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Initialize the node itself
        super().__init__('ekf_slam')

        # Robot name
        self.robot_name = self.get_namespace()[1:]

        # We will use this frame_id as our map frame id
        # TODO: Make this a parameter
        self.base_frame_id = 'map'

        # The actual Extended Kalman Filter
        self.ekf = ExtendedKalmanFilter()

        #
        #  Parameters
        #

        # Gain factor for the weights given the distance
        base_frame_id_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Map frame id (defaults to "map"')
        self.declare_parameter('base_frame_id', 'map',
                               base_frame_id_param_desc)

        # General global variables
        self.outfile = open('Output.txt', 'w')  # Save robot poses to this file
        # Will store the latest odometry pose
        self.odom_robot_pose = Pose2D()
        # True if the odometry was updated at least once
        self.odom_updated_once = False

        # Send initial information to the output file
        self.outfile.write(
            'Estimated pose of the robot\n\n' +
            '[T]: Odometry [X Y Theta] SLAM [X Y Theta]\n\n')

        # Particle-filter global variables
        # To store the estimation result
        self.robot_estimated_pose = Pose2D()

        # Control the time between particles publishing
        self.prev_time = 0.0

        # Store the positions of the read landmarks [m] (for debugging purposes
        # only, since these are detected at runtime)
        self.markers_true_wpos = np.array([[-8.0,  4.0],  # 1
                                           [-8.0, -4.0],  # 2
                                           [-3.0, -8.0],  # 3
                                           [3.0, -8.0],  # 4
                                           [8.0, -4.0],  # 5
                                           [8.0, 4.0],  # 6
                                           [3.0, 8.0],  # 7
                                           [-3.0, 8.0]])  # 8
        # Max/min x/y values of the markers (could be taken from above)
        self.min_x = -8.0
        self.max_x = 8.0
        self.min_y = -8.0
        self.max_y = 8.0

        # Create TF buffer and listener (for odom->base_link)
        self.tf_buffer = tf_buffer
        # Create broadcaster for map->base_link
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Estimated pose publisher
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 1)

        # Markers array publisher for debugging EKF-SLAM
        self.markers_array_pub = self.create_publisher(
            MarkerArray, 'landmarks', 1)

        # Not that we have oiur map, we can start processing odometry and
        # markers information, so lets subsribe the corresponding topics.
        # Setup subscribers using a ApproximateTimeSynchronizer filter for the
        # odometry and the markers. We want to estimate the robot pose which is
        # closest in time from the published odometry and markers.
        # Setup odometry subscriber
        self.sub_odom = message_filters.Subscriber(self, Odometry, 'odom')
        # Detected landmarks
        self.sub_markers = message_filters.Subscriber(self, Markers, 'markers')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_markers], 3, 0.2)
        ts.registerCallback(self.odom_markers_cb)

        self.get_logger().info(
            'The EKF-SLAM has been initialized.')

    def __del__(self):
        ''' Destructor, which will be called when the instance is destroyed.'''
        # Close file before leaving
        self.outfile.close()

    def publishDebugInformation(self, timestamp):
        '''
        Publish EKF-SLAM related information for debugging purposes, namely
        the computed landmarks and the real landmarks. This information is only
        publisedh if there is at least one subscriber.
        All arguments, except the timestamp, are expected to be numpy arrays.
        All coordinates should be in world/map coordinates.
        '''
        if self.markers_array_pub.get_subscription_count() == 0:
            # If no node is receiving this information, then there is no need
            # to publish it. (This test could be done for all publishers.)
            return

        markers_array = MarkerArray()
        marker_id = 0
        scale = 0.3
        scale_vector = Vector3(x=scale, y=scale, z=scale)
        pose_origin = Pose(position=Point(x=0.0, y=0.0, z=0.0),
                           orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        # Original landmarks (green points list)
        # We could build this marker only once during initialization!
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
        marker.points = rnp.msgify(Point, self.markers_true_wpos).tolist()
        markers_array.markers.append(marker)

        # Detected landmarks (red points list), if there are any
        if self.ekf.state.shape[0] > 3:
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
            x_marks = np.arange(3, self.ekf.state.shape[0], 2)
            y_marks = np.arange(4, self.ekf.state.shape[0], 2)
            marker.points = rnp.msgify(
                Point,
                np.transpose(np.vstack((self.ekf.state[x_marks, 0],
                                        self.ekf.state[y_marks, 0])))).tolist()
            markers_array.markers.append(marker)

        # Publish Markers Array
        self.markers_array_pub.publish(markers_array)

    def odom_markers_cb(self, odom_msg: Odometry, markers_msg: Markers):
        '''
        This callback is called whenever we have an odometry message and a
        markers message. The messages are received here simultaneously and
        correspond to (approximately) the same time stamp.
        '''

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
            # self.ekf.state[0, 0] = new_odom_robot_pose.x
            # self.ekf.state[1, 0] = new_odom_robot_pose.y
            # self.ekf.state[2, 0] = new_odom_robot_pose.theta
            # Use the three lines above, instead of the next three lines, if
            # you want to use the odometry valus for initialiing the Kalman
            # filter. Currently we are not using that information on startup,
            # but assuming instead that the robot is starting at (0.0,0.0, 0.0)
            self.ekf.state[0, 0] = 0.0
            self.ekf.state[1, 0] = 0.0
            self.ekf.state[2, 0] = 0.0
            self.odom_updated_once = True  # We have received or first value
        elif RUN_PREDICTION_STEP:  # Run only if step enabled
            # Compute the robot motion given the last know robot pose from
            # odometry and the current one
            local_pose = lfwft.world2LocalPose(self.odom_robot_pose,
                                               new_odom_robot_pose)
            # Execute the actual prediction step
            self.ekf.predictStep(local_pose.x, local_pose.y, local_pose.theta)

        # Store current odometry as a reference for the next iteration
        self.odom_robot_pose = new_odom_robot_pose

        ''' Step 2 - Perform the update step based on the detected markers

            This step is only performed if the robot detected at least one
            landmark, and if this step is enabled
        '''
        if (markers_msg.num_markers > 0) and RUN_OBSERVATION_STEP:
            self.ekf.updateStep(markers_msg)
        else:
            self.get_logger().debug('No markers detected.')

        # Publish the TF fom map to odom
        self.publish_map_odom_tf(odom_msg.header.stamp)

        # Publish the estimated pose message. It needs to be
        # PoseStamped, a 3D pose with a timestamp. We will create one
        # from the robot_estimated_pose.
        pose_to_publish = PoseStamped()
        pose_to_publish.header.frame_id = self.get_parameter(
                'base_frame_id').get_parameter_value().string_value
        pose_to_publish.header.stamp = odom_msg.header.stamp
        pose_to_publish.pose.position = \
            Point(x=self.ekf.state[0, 0],
                  y=self.ekf.state[1, 0],
                  z=0.)
        pose_to_publish.pose.orientation = \
            rpyToQuaternion(0., 0., self.ekf.state[2, 0])
        self.pose_pub.publish(pose_to_publish)

        # Publish debug information from time to time
        if odom_msg.header.stamp.sec - self.prev_time >= DELTA_DEBUG:
            self.prev_time = odom_msg.header.stamp.sec

            # Landmarks information for RViz
            self.publishDebugInformation(odom_msg.header.stamp)

            # Write data to the file
            self.outfile.write(f'{odom_msg.header.stamp.sec:.0f}.' +
                               f'{odom_msg.header.stamp.nanosec:.0f}: ' +
                               f'{self.odom_robot_pose.x:.2f} ' +
                               f'{self.odom_robot_pose.y:.2f} ' +
                               f'{self.odom_robot_pose.theta:.2f} ' +
                               f'{self.ekf.state[0, 0]:.2f} ' +
                               f'{self.ekf.state[1, 0]:.2f} ' +
                               f'{self.ekf.state[2, 0]:.2f}\n')

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
        odom_to_base_footprint_tf = trf.concatenate_matrices(
            trf.translation_matrix(
                [odom_to_base_footprint_trans.transform.translation.x,
                 odom_to_base_footprint_trans.transform.translation.y,
                odom_to_base_footprint_trans.transform.translation.z]),  # T
            trf.quaternion_matrix(
                [odom_to_base_footprint_trans.transform.rotation.w,
                 odom_to_base_footprint_trans.transform.rotation.x,
                 odom_to_base_footprint_trans.transform.rotation.y,
                 odom_to_base_footprint_trans.transform.rotation.z]))  # R

        # Get the transformation matrix from the base_footprint to the map from
        # the particle filter estimate
        base_footprint_to_map_tf = trf.concatenate_matrices(
            trf.translation_matrix(
                [self.ekf.state[0, 0],
                 self.ekf.state[1, 0],
                 0.0]),  # T
            trf.rotation_matrix(
                self.ekf.state[2, 0], [0, 0, 1]))  # R

        # Get the transformation from odom to map
        odom_to_map = trf.concatenate_matrices(base_footprint_to_map_tf,
                                               odom_to_base_footprint_tf)

        # Publish transformation from odom to map (map->odom link)
        odom_to_map_trans_stamped = tf2_ros.TransformStamped()
        odom_to_map_trans_stamped.header.stamp = timestamp
        odom_to_map_trans_stamped.header.frame_id = self.get_parameter(
                'base_frame_id').get_parameter_value().string_value
        odom_to_map_trans_stamped.child_frame_id = f'{self.robot_name}/odom'
        odom_to_map_trans_stamped.transform.translation.x = odom_to_map[0, 3]
        odom_to_map_trans_stamped.transform.translation.y = odom_to_map[1, 3]
        odom_to_map_trans_stamped.transform.translation.z = odom_to_map[2, 3]
        trf_quat = trf.quaternion_from_matrix(odom_to_map, True)
        odom_to_map_trans_stamped.transform.rotation = Quaternion(
            x=trf_quat[1],
            y=trf_quat[2],
            z=trf_quat[3],
            w=trf_quat[0])
        self._tf_broadcaster.sendTransform(odom_to_map_trans_stamped)


def main(args=None):
    '''
    Main function
    Use an EKF-based SLAM.
    '''

    print('EKF-based SLAM\n-----------------------')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our TF listener node
    tf_listener_node = TfListener()
    # Create our Particle filter node
    ekf_slam_node = EKFSLAM(
        tf_buffer=tf_listener_node.tf_buffer)

    # We will execute each node in its own thread. This is important to make
    # sure that the TF listener is continuously updated.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_listener_node)
    executor.add_node(ekf_slam_node)

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
