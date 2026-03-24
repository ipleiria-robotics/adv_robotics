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
from threading import Lock
import numpy as np

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

# ROS TF-related
from geometry_msgs.msg import PoseWithCovarianceStamped, Point

# Our functions
import ar_py_utils.utils as utils
from markers_msgs.msg import Markers


class Trilateration(Node):
    '''
    Basic robot localization using trilateration.
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''
        self.lock = Lock()

        # Internal pose-related variables
        self.robot_real_pose = Pose2D()  # Store real (error-free) robot pose
        self.robot_estimated_pose = Pose2D()  # Store estimated robot pose
        self.real_pose_updated = False  # True if the real pose was updated

        # World reference frame
        self.base_frame_id = 'map'

        # Initialize the node itself
        super().__init__('lw1_localization')

        # Setup markers subscriber
        self.create_subscription(Markers, 'markers', self.markers_callback, 1)

        # Setup ground-truth subscriber
        self.create_subscription(Odometry,
                                 'base_pose_ground_truth',
                                 self.pose_gnd_truth_callback, 1)

        # Setup pose publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              'pose', 1)

    def markers_callback(self, msg: Markers):
        ''' Process received markers data '''

        self.get_logger().info(
            f'Got {msg.num_markers} beacons at time ' +
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec')

        # Display found beacons information
        for i in range(msg.num_markers):
            self.get_logger().info(
                f'Beacon {msg.id[i]:d} : (range, bearing) = (' +
                f'{msg.range[i]:.2f} m, {msg.bearing[i]:.2f} °)\n' +
                '---')

        ###
        # CHANGE THE CODE BELOW
        ###

        #
        # Change/use this code according to your needs
        #

        # A matrix, 3 x 2
        A = np.zeros((3, 2), float)

        # b matrix, 4 x 1 (columm vector)
        b = np.zeros((4, 1), float)

        # Store some test values in A
        A[0, 0] = 1
        A[0, 1] = 2
        A[1, 0] = -1
        A[1, 1] = 2
        A[2, 0] = 1
        A[2, 1] = -2
        # Add some values to A using the "append" instruction.
        # A becomes (4 x 2) has expected
        A = np.append(A, [[2., 2.5]], axis=0)

        # Store some test values in b
        b[0] = 1
        b[1] = 2
        b[2] = -1
        b[3] = 1.5

        # Compute r = inv(A'*A)*A'*b, where inv(X) = X^(-1) and @ is used for
        # matrix multiplication (using '*' would result in element-wise
        # multiplication, which is not what we want)
        # r = np.linalg.inv(np.T(A) @ A) @ np.T(A) @ b
        r = np.linalg.pinv(A) @ b

        # Debug code
        print('Matrix A:')
        print(A)
        print('Vector b:')
        print(b)
        print('Vector r:')
        print(r)

        # ^-- The code above is here just to show the minimization procedure
        # implementation described in the lab work guide.

        # Currently, we are publishing the ground-truth pose for testing
        # purposes. You need to replace this by your algorithm to compute the
        # robot pose.

        with self.lock:
            # Fill pose values using the localization results
            # CHANGE THIS: it is currently publishing the ground truth
            self.robot_estimated_pose = \
                Pose2D(x=self.robot_real_pose.x,
                       y=self.robot_real_pose.y,
                       theta=self.robot_real_pose.theta)

            # Publish the pose message. It needs to be PoseStamped, a 3D pose
            # with a timestamp. We will create one from the
            # robot_estimated_pose.
            pose_to_publish = PoseWithCovarianceStamped()
            pose_to_publish.header.frame_id = self.base_frame_id
            pose_to_publish.header.stamp = msg.header.stamp
            pose_to_publish.pose.pose.position = \
                Point(x=self.robot_estimated_pose.x,
                      y=self.robot_estimated_pose.y, z=0.)
            pose_to_publish.pose.pose.orientation = \
                utils.rpyToQuaternion(0., 0., self.robot_estimated_pose.theta)
            self.pose_pub.publish(pose_to_publish)

    def pose_gnd_truth_callback(self, msg: Odometry):
        '''
        Store internally a copy of the error-free robot pose.
        '''
        with self.lock:
            self.robot_real_pose.x = msg.pose.pose.position.x
            self.robot_real_pose.y = msg.pose.pose.position.y
            self.robot_real_pose.theta = \
                utils.quaternionToYaw(msg.pose.pose.orientation)


def main(args=None):
    '''
    Main function.
    '''

    # Output usage information
    print('Basic trilateration-based localization.\n' +
          '------------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    loc_node = Trilateration()

    # Get the node executing
    rclpy.spin(loc_node)

    # Cleanup memory and shutdown
    loc_node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')
