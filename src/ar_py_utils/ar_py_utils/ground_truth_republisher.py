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

# ROS API
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Republisher(Node):
    '''
    Republish ground truth with different map frame_id.
    '''
    def __init__(self):
        '''
        Initialize class instance.
        '''

        # Initialize the node itself
        super().__init__('republisher')

        # Pose publisher
        self.pose_pub = self.create_publisher(
            Odometry,
            'base_pose_ground_truth_map', 1)

        # Setup ground-truth subscriber
        self.create_subscription(Odometry,
                                 'base_pose_ground_truth',
                                 self.pose_gnd_truth_callback, 1)

    def pose_gnd_truth_callback(self, msg):
        '''
        Republish pose with map frame_id.
        '''
        msg.header.frame_id = 'map'
        self.pose_pub.publish(msg)


def main(args=None):
    '''
    Main function
    Ground truth republisher.
    '''

    print('Ground truth republisher\n---------------------------')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our republisher node
    republisher_node = Republisher()

    # Get the node executing
    rclpy.spin(republisher_node)

    # Cleanup memory and shutdown
    republisher_node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
    print('Quitting...')
