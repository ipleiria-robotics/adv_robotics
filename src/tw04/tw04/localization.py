#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
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
from math import pi, atan2, radians

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

# ROS TF-related
from geometry_msgs.msg import PoseWithCovarianceStamped, Point

# Our functions
import ar_py_utils.utils as utils
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D
from markers_msgs.msg import Markers

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 0.5  # [m/s]
MAX_ANG_VEL = radians(90)  # [rad/s]


class Trilateration(Node):
    '''
    Basic robot localization using trilateration.
    '''
    def __init__(self):
        '''
        Initializes the class instance.
        '''

        # Limits of the world where the robot is working
        # These two values could be a parameter
        self.X_MAX_POS = 3.32  # [m]
        self.Y_MAX_POS = 2.28  # [m]

        # Store the world positions of the landmarks
        self.beacons_wpos = \
            [Point2D(-self.X_MAX_POS, -self.Y_MAX_POS),  # 1
             Point2D(-self.X_MAX_POS, self.Y_MAX_POS),  # 2
             Point2D(self.X_MAX_POS, self.Y_MAX_POS),  # 3
             Point2D(self.X_MAX_POS, -self.Y_MAX_POS)]  # 4

        # Frame IDs
        self.base_frame_id = 'map'

        # Initialize the node itself
        super().__init__('tw04_localization')

        # Setup markers subscriber
        self.create_subscription(Markers, 'markers', self.markers_callback, 1)

        # Setup velocity commands publisher
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              'pose', 1)

    def markers_callback(self, msg: Markers):
        ''' Process received markers data '''

        self.get_logger().info(
            f'Got {msg.num_markers} beacons at time ' +
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec')

        # If we have less then 3 beacons, return an error
        if (msg.num_markers < 3):
            self.get_logger().warn('Unable to localize !!!')
            return

        # Display found beacons information
        for i in range(msg.num_markers):
            self.get_logger().info(
                f'Beacon {msg.id[i]:d} : (range, bearing) = (' +
                f'{msg.range[i]:.2f} m, {msg.bearing[i]:.2f} °)\n---')

        # Compute the robot localization based on the three beacons found
        b1w = self.beacons_wpos[msg.id[0]-1]  # Beacon 1 world position
        b2w = self.beacons_wpos[msg.id[1]-1]  # Beacon 2 world position
        b3w = self.beacons_wpos[msg.id[2]-1]  # Beacon 3 world position

        # Fill variables
        d1s = msg.range[0]*msg.range[0]
        d2s = msg.range[1]*msg.range[1]
        d3s = msg.range[2]*msg.range[2]
        x1 = b1w.x
        y1 = b1w.y
        x2 = b2w.x
        y2 = b2w.y
        x3 = b3w.x
        y3 = b3w.y

        # Compute the robot position
        # From the theoretical classes:
        #   A = (d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)/(2*(y2-y1))
        #   B = (x1-x2)/(y2-y1)
        #   C = (d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)/(2*(y3-y2))
        #   D = (x2-x3)/(y3-y2)
        # with
        #   robot_localized_pos->x = (A - C) / (D - B);
        #   robot_localized_pos->y = A + B * robot_localized_pos->x;
        # The following code implements the above expressions:
        robot_estimated_pose = Pose2D()
        robot_estimated_pose.x = 0.5 * \
            ((y3-y2)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2) -
             (y2-y1)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)) / \
            ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2))
        robot_estimated_pose.y = 0.5 * \
            ((x2-x3)*(d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2) -
             (x1-x2)*(d2s-d3s-x2*x2+x3*x3-y2*y2+y3*y3)) / \
            ((y2-y1)*(x2-x3)-(y3-y2)*(x1-x2))

        # Compute the robot orientation
        alpha0 = atan2(y1 - robot_estimated_pose.y,
                       x1 - robot_estimated_pose.x)
        robot_estimated_pose.theta = alpha0 - msg.bearing[0]
        # Limit the angle between -PI and PI
        if robot_estimated_pose.theta > pi:
            robot_estimated_pose.theta -= 2*pi
        elif robot_estimated_pose.theta < -pi:
            robot_estimated_pose.theta += 2*pi

        # Publish the pose message. It needs to be PoseStamped, a 3D pose with
        # a timestamp. We will create one from the robot_estimated_pose.
        pose_to_publish = PoseWithCovarianceStamped()
        pose_to_publish.header.frame_id = self.base_frame_id
        pose_to_publish.header.stamp = msg.header.stamp
        pose_to_publish.pose.pose.position = \
            Point(x=robot_estimated_pose.x, y=robot_estimated_pose.y, z=0.)
        pose_to_publish.pose.pose.orientation = \
            utils.rpyToQuaternion(0., 0., robot_estimated_pose.theta)
        self.pose_pub.publish(pose_to_publish)


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
