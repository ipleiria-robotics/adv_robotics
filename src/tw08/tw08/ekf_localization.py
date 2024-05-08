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

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose2D, PoseWithCovarianceStamped, \
    Quaternion
from nav_msgs.msg import Odometry
import message_filters
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Our functions
from tw08.ExtendedKalmanFilter import ExtendedKalmanFilter
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


class EKFLocalization(Node):
    '''
    Localization using a.
    '''
    def __init__(self, tf_buffer):
        '''
        Initialize class instance.
        '''

        # Prevent simultaneous read/write to the class variables
        self.lock = threading.Lock()

        # Initialize the node itself
        super().__init__('ekf_localization')

        # Robot name
        self.robot_name = self.get_namespace()[1:]

        # The actual Extended Kalman Filter
        self.ekf = ExtendedKalmanFilter()

        #
        #  Parameters
        #

        # Map frame id parameter
        # This is defined here, and can be changed as a parameter, given
        # that we are not receiving a map.
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
            '[T]: Odometry [X Y Theta] Extended Kalman Filter [X Y Theta]\n\n')

        # Particle-filter global variables
        # To store the estimation result
        self.robot_estimated_pose = Pose2D()

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
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              'pose', 2)

        # Not that we have oiur map, we can start processing odometry and
        # markers information, so lets subsribe the corresponding topics.
        # Setup subscribers using a ApproximateTimeSynchronizer filter for the
        # odometry and the markers. We want to estimate the robot pose which is
        # closest in time from the published odometry and markers.
        # Setup odometry subscriber
        self.sub_odom = message_filters.Subscriber(
            self, Odometry, 'odom')
        # Detected landmarks
        self.sub_markers = message_filters.Subscriber(
            self, Markers, 'markers')
        # Joint callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_odom, self.sub_markers], 3, 0.2)
        ts.registerCallback(self.odom_markers_cb)

        self.get_logger().info(
            'The Extended Kalman Filter has been initialized.')

    def __del__(self):
        ''' Destructor, which will be called when the instance is destroyed.'''
        # Close file before leaving
        self.outfile.close()

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
        run_filter = True
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
        elif RUN_PREDICTION_STEP:  # Run only if this step is enabled
            # Compute the robot motion given the last know robot pose from
            # odometry and the current one
            local_pose = lfwft.world2LocalPose(self.odom_robot_pose,
                                               new_odom_robot_pose)
            # If the robot has not moved, do not run any of the steps.
            if (np.abs(local_pose.x) < 0.001) and \
               (np.abs(local_pose.y) < 0.001) and \
               (np.abs(local_pose.theta) < 0.01):
                run_filter = False
            else:
                # Execute the actual prediction step
                self.ekf.predictStep(local_pose.x, local_pose.y,
                                     local_pose.theta)

        # Store current odometry as a reference for the next iteration
        self.odom_robot_pose = new_odom_robot_pose

        ''' Step 2 - Perform the update step based on the detected markers

            This step is only performed if the robot detected at least one
            landmark, and if this step is enabled
        '''
        if run_filter is True:
            if (markers_msg.num_markers > 0) and RUN_OBSERVATION_STEP:
                self.ekf.updateStep(markers_msg, self.markers_wpos)
            else:
                self.get_logger().debug('No markers detected.')

        # Publish the TF fom map to odom
        self.publish_map_odom_tf(odom_msg.header.stamp)

        # Publish the estimated pose message. It needs to be
        # PoseWithCovarianceStamped, a 3D pose with a timestamp and a
        # covariance. We will create one from the robot_estimated_pose.
        pose_to_publish = PoseWithCovarianceStamped()
        pose_to_publish.header.frame_id = self.get_parameter(
                'base_frame_id').get_parameter_value().string_value
        pose_to_publish.header.stamp = odom_msg.header.stamp
        pose_to_publish.pose.pose.position = \
            Point(x=self.ekf.state[0, 0],
                  y=self.ekf.state[1, 0],
                  z=0.)
        pose_to_publish.pose.pose.orientation = \
            rpyToQuaternion(0., 0., self.ekf.state[2, 0])

        # Fill covariance matrix. We will consider 0 for all the values we are
        # not computing, i.e., Z, RX and RY, and correlated values.
        pose_to_publish.pose.covariance = np.array([
            # X, Y, Z, RX, RY, RZ
            self.ekf.P[0, 0], self.ekf.P[0, 1], 0.0, 0.0, 0.0, self.ekf.P[0, 2],  # X
            self.ekf.P[1, 0], self.ekf.P[1, 1], 0.0, 0.0, 0.0, self.ekf.P[1, 2],  # Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RX
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RY
            self.ekf.P[2, 0], self.ekf.P[2, 1], 0.0, 0.0, 0.0, self.ekf.P[2, 2]])  # RZ
        # The pose_to_publish is complete, lets publish it
        self.pose_pub.publish(pose_to_publish)

        # Publish debug information from time to time
        if odom_msg.header.stamp.sec - self.prev_time >= DELTA_DEBUG:
            self.prev_time = odom_msg.header.stamp.sec

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
        odom_to_map_trans_stamped.header.frame_id = self.get_parameter(
                'base_frame_id').get_parameter_value().string_value
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
    Use Extended Kalman Filter for localization.
    '''

    print('Extended Kalman Filter-based localization\n-----------------------')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our TF listener node
    tf_listener_node = TfListener()
    # Create our Particle filter node
    ekf_localization_node = EKFLocalization(
        tf_buffer=tf_listener_node.tf_buffer)

    # We will execute each node in its own thread. This is important to make
    # sure that the TF listener is continuously updated.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_listener_node)
    executor.add_node(ekf_localization_node)

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
