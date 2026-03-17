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


# ROS API
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Our utility functions
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D


class PeriodicFixedPathPublisher(Node):
    '''
    Path global planner using potential fields from laser sensor information.
    It publishes costmap (raw) with the potential fields result
    '''
    def __init__(self, targets):
        '''
        Initializes the class instance.
        '''

        # Initialize the node itself
        super().__init__('periodic_fixed_path_publisher')

        #  Store the given list of targets as a Path. Since te path is fixed,
        # we do this only once, at the beggining.
        num_targets = len(targets)
        self.path = Path()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'map'  # TODO: make this a parameter
        for i in range(num_targets+1):  # +1 to return to the 1st target
            # For the path we need to have a PoseStamped, which means we need
            # to include the orientation. We will use as orientation the angle
            # 0, since it later on we will only use the position to navigate.
            pose = PoseStamped()
            # We are not using the time for now, so just use the same for all
            pose.header.stamp = self.path.header.stamp
            # The reference frame is the same global frame for all points
            pose.header.frame_id = self.path.header.frame_id
            # Get the position from the targets list
            pose.pose.position.x = targets[i % num_targets].x
            pose.pose.position.y = targets[i % num_targets].y
            pose.pose.position.z = 0.0
            # Orientation will not be used (each target as the world heading)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            # Add to the path
            self.path.poses.append(pose)

        # Create the path publisher
        self.path_pub = self.create_publisher(Path, 'path', 1)
        # Immediately publish a path, so as not to wait for the periodic
        # callback for the path to be published for the first time.
        self.path_pub.publish(self.path)

        # Setup periodic callback to publish the path evey 50 secs
        self.pubtimer = self.create_timer(50.0, self.path_pub_cb)

    def path_pub_cb(self):
        '''
        Publish the fixed path computed initially.
        '''
        self.path_pub.publish(self.path)


def main(args=None):
    '''
    Main function.
    '''

    # Output usage information
    print('Fixed waypoint path periodic publisher\n' +
          '--------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # List of waypoints the robot should pass by. We are considering only the
    # desired position, with no orientation
    targets = [Point2D(-2.6, -1.7),  # 0
               Point2D(-2.6, 1.7),  # 1
               Point2D(0.0, 0.4),  # 2
               Point2D(2.6, 1.7),  # 3
               Point2D(2.6, -1.7),  # 4
               Point2D(0.0, -0.5)]  # 5

    # Create our navigation node
    node = PeriodicFixedPathPublisher(targets)

    # Get the node executing
    rclpy.spin(node)

    # Cleanup memory and shutdown
    node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')
