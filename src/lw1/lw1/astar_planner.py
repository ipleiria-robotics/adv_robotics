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
from math import degrees

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, \
    QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose2D
from nav_msgs.msg import OccupancyGrid, Path

# Our utility functions
import ar_py_utils.utils as utils


class AStarPlanner(Node):
    '''
    A* planner.
    '''

    def __init__(self):
        '''
        Initializes the class instance.
        '''
        # Initialize the node itself
        super().__init__('lw1_astar_planner')

        # Map subscriber
        # Since the map is only published when the map server starts, we need
        # to get the message that was last pubslihed, even if it as published
        # before we subscribed the topic. To enable that behavior so, we
        # specify the TRANSIENT_LOCAL Durability Policy.
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, 'map',
                                                self.map_cb, qos_profile)
        # Pose susbcriber
        self.create_subscription(PoseStamped, 'pose', self.pose_cb, 1)

        # Costmap as occupancyGrid (for RViz)
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, 'occgrid', 1)

        # Path publisher
        self.path_pub = self.create_publisher(Path, 'path', 1)

    def map_cb(self, msg):
        pass

    def pose_cb(self, msg):
        '''
            Navigation callback, executes on pose message received
        '''

        robot_pose = Pose2D(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            theta=utils.quaternionToYaw(msg.pose.orientation))

        # Show received estimated pose
        # NOTE: this is here only for testing purposes. You should avoid
        # printing information unless strictly needed.
        self.get_logger().info(
            f'Robot estimated pose (X, Y, Theta)= {robot_pose.x:.2f} [m]' +
            f', {robot_pose.y:.2f} [m], ' +
            f'{degrees(robot_pose.theta):.2f} [º]\r')


def main(args=None):
    '''
    Main function.
    '''

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    node = AStarPlanner()

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
