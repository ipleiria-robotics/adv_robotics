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


# Library packages needed
from math import pi, atan2, radians, degrees, sqrt
from threading import Lock

# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import message_filters

# ROS TF-related
from geometry_msgs.msg import PoseStamped, Point

# Our functions
import tw04.utils as utils
from tw03.LocalFrameWorldFrameTransformations import Point2D
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

        # Localization related variables. Initially have None value, until
        # we compute the robot localization our get the pose ground truth.
        # Note that the ground truth is just here for debugging reasons, and
        # should not be used in a real situation.
        self.robot_ground_truth_pose = None  # Error-free pose for debugging
        self.robot_estimated_pose = Pose2D()  # Our estimated robot pose
        # Lock to access pose-related variables
        self.pose_lock = Lock()

        # Robot name(space)
        self.robot_name = 'robot_0'

        # Frame IDs
        self.base_frame_id = 'map'

        # Initialize the node itself
        super().__init__('tw04_localization')

        # Setup subscribers
        # Pose ground truth (for debugging purposes only)
        self.create_subscription(Odometry,
                                 f'/{self.robot_name}/base_pose_ground_truth',
                                 self.pose_ground_truth_callback,
                                 1)
        # Markers
        self.create_subscription(Markers, f'/{self.robot_name}/markers',
                                 self.markers_callback, 1)

        # Setup publisher
        self.pose_pub = self.create_publisher(PoseStamped,
                                              f'/{self.robot_name}/pose', 1)

    def pose_ground_truth_callback(self, data: Odometry):
        ''' Function to call whe new pose ground truth  information is
        available. This is used only for debugging, and cannot be used in real
        world applications '''

        # Store received values
        with self.pose_lock:
            self.robot_ground_truth_pose = Pose2D(
                x=data.pose.pose.position.x,
                y=data.pose.pose.position.y,
                theta=utils.quaternionToYaw(data.pose.pose.orientation))

    def markers_callback(self, msg: Markers):
        ''' Process received markers data '''

        self.get_logger().info(
            'Got beacons message at ' +
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

        # If we have less then 3 beacons, return an error
        if(msg.num_markers < 2):
            self.get_logger().warn('Unable to localize !!!')
            return

        # Display found beacons information
        self.get_logger().info(f'Found {msg.num_markers} beacons.')
        for i in range(msg.num_markers):
            self.get_logger().info(
                f'Beacon {msg.id[i]:.2f} : (range, bearing) = (' +
                f'{msg.range[i]:.2f}, {msg.bearing[i]:.2f})')

        # Compute the robot localization based on the three beacons found
        b1w = self.beacons_wpos[msg.id[0]-1]  # Beacon 1 world position
        b2w = self.beacons_wpos[msg.id[1]-1]  # Beacon 2 world position

        # Fill variables
        d1s = msg.range[0]*msg.range[0]
        d2s = msg.range[1]*msg.range[1]
        x1 = b1w.x
        y1 = b1w.y
        x2 = b2w.x
        y2 = b2w.y

        if(y2 != y1):
            # Compute A and B
            A = (d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)/(2*(y2-y1))
            B = (x1-x2)/(y2-y1)
            # Compute a, b and c
            a = 1+B*B
            b = -2*x1 + 2*A*B - 2*B*y1
            c = A*A + x1*x1 + y1*y1 - 2*A*y1 - d1s
            h1_x = (-b+sqrt(b*b-4*a*c))/(2*a)
            h2_x = (-b-sqrt(b*b-4*a*c))/(2*a)
            h1_y = A + B * h1_x
            h2_y = A + B * h2_x
        else:
            # Compute E and F
            E = (d1s-d2s-x1*x1+x2*x2-y1*y1+y2*y2)/(2*(x2-x1))
            F = (y1-y2)/(x2-x1)
            # Compute a, b and c
            a = 1+F*F
            b = -2*y1 + 2*E*F - 2*F*x1
            c = E*E + x1*x1 + y1*y1 - 2*E*x1 - d1s
            h1_y = (-b+sqrt(b*b-4*a*c))/(2*a)
            h2_y = (-b-sqrt(b*b-4*a*c))/(2*a)
            h1_x = E + F * h1_y
            h2_x = E + F * h2_y

        with self.pose_lock:
            # Check which of the hypothesis is the best.
            if((h1_x > -self.X_MAX_POS) and (h1_x < self.X_MAX_POS) and
               (h1_y > -self.Y_MAX_POS) and (h1_y < self.Y_MAX_POS)):
                self.robot_estimated_pose.x = h1_x
                self.robot_estimated_pose.y = h1_y
            else:
                self.robot_estimated_pose.x = h2_x
                self.robot_estimated_pose.y = h2_y

            # Estimate the angle
            alpha0 = atan2(y1 - self.robot_estimated_pose.y,
                           x1 - self.robot_estimated_pose.x)
            self.robot_estimated_pose.theta = alpha0 - msg.bearing[0]
            # Limit the angle between -PI and PI
            if self.robot_estimated_pose.theta > pi:
                self.robot_estimated_pose.theta -= 2*pi
            elif self.robot_estimated_pose.theta < -pi:
                self.robot_estimated_pose.theta += 2*pi

            # Publish the pose message
            pose_to_publish = PoseStamped()
            pose_to_publish.header.frame_id = self.base_frame_id
            pose_to_publish.header.stamp = msg.header.stamp
            pose_to_publish.pose.position = \
                Point(x=self.robot_estimated_pose.x,
                      y=self.robot_estimated_pose.y,
                      z=0.)
            pose_to_publish.pose.orientation = \
                utils.rpyToQuaternion(0., 0., self.robot_estimated_pose.theta)
            self.pose_pub.publish(pose_to_publish)

            # Print the estimated pose
            self.get_logger().info(
                'Robot estimated pose: (X [m], Y [m], Theta [°] = (' +
                f'{self.robot_estimated_pose.x:0.2f}, ' +
                f'{self.robot_estimated_pose.y:0.2f}, ' +
                f'{degrees(self.robot_estimated_pose.theta):0.2f})')

            # Print the estimated error given the last ground truth pose
            if self.robot_ground_truth_pose is not None:
                # This is just for debuggin purposes and would not be used in
                # the real application
                angle_error = self.robot_estimated_pose.theta - \
                    self.robot_ground_truth_pose.theta
                # Limit the angle error between -PI and PI
                if angle_error > pi:
                    angle_error -= 2*pi
                elif angle_error < -pi:
                    angle_error += 2*pi
                self.get_logger().info(
                    f'Robot pose error (X [m], Y [m], Theta [°])= (' +
                    f'''{self.robot_estimated_pose.x -
                         self.robot_ground_truth_pose.x:.2f}, ''' +
                    f'''{self.robot_estimated_pose.y -
                         self.robot_ground_truth_pose.y:.2f}, ''' +
                    f'{angle_error:.2f})\n---')


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
