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
from math import radians, inf, atan2
import numpy as np

# ROS API
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

# Our utility functions (using the ones from tw02)
import tw04.utils as utils
from tw03.LocalFrameWorldFrameTransformations import Point2D, world2Local

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = radians(90)  # [rad/s]

# Wether to use odometry or the localization pose
USE_ODOM = False


class BasicPathNavigation(Node):
    '''
    Random navigation avoiding laser-based detected obstacles.
    '''

    def __init__(self):
        '''
        Initializes the class instance.
        '''
        # TODO: these could be parameters
        # Robot name
        self.robot_name = 'robot_0'

        # Internal navigation variables
        self.velocity_at_target = 0  # Desired velocity at next target
        self.curr_target = 0  # Current target location index
        self.Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum acceptance distance to target
        self.max_angle_to_target = radians(30.0)
        self.first_run = True  # True if the callback was never called

        # Array of points to be followed:
        self.targets = [Point2D(-2.6, -1.7),  # 1
                        Point2D(-2.6, 1.7),  # 2
                        Point2D(0.0, 0.4),  # 3
                        Point2D(2.6, 1.7),  # 4
                        Point2D(2.6, -1.7),  # 5
                        Point2D(0.0, -0.4)]  # 6
        self.num_targets = len(self.targets)

        # Initialize the node itself
        super().__init__('tw04_path_navigation')

        # Parameters
        # Maximum linear velocity
        ref_lin_vel_param_desc = \
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='Reference linear velocity!')
        self.declare_parameter('ref_lin_vel', 0.2,  ref_lin_vel_param_desc)
        # Maximum linear velocity
        ref_ang_vel_param_desc = \
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='reference angular velocity!')
        self.declare_parameter('ref_ang_vel', radians(30),
                               ref_ang_vel_param_desc)

        # Setup odometry or pose susbcriber
        if USE_ODOM:  # Use odometry
            self.create_subscription(Odometry, f'/{self.robot_name}/odom',
                                     self.pose_cb, 1)
        else:  # Use the localization result
            self.create_subscription(PoseStamped, f'/{self.robot_name}/pose',
                                     self.pose_cb, 1)

        # Setup velocity publisher
        self.vel_pub = self.create_publisher(Twist,
                                             f'/{self.robot_name}/cmd_vel', 1)

    def pose_cb(self, msg):
        if USE_ODOM:  # Get robot pose from the odometry message
            robot_pose = Pose2D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                theta=utils.quaternionToYaw(msg.pose.pose.orientation))
        else:  # Get the robot pose from the PoseStamped message
            robot_pose = Pose2D(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                theta=utils.quaternionToYaw(msg.pose.orientation))

        # If this is the first time we are entering this calback, determine
        # the closest point. We do that by computing the distance from the
        # robot to all desired target locations, and selecting the one with the
        # lowest distance.
        if self.first_run:
            self.first_run = False
            lowest_sq_distance = inf
            for i in range(self.num_targets):
                new_sq_distance = (robot_pose.x-self.targets[i].x)**2 + \
                                  (robot_pose.y-self.targets[i].y)**2
                if(new_sq_distance < lowest_sq_distance):
                    lowest_sq_distance = new_sq_distance
                    self.curr_target = i

        # Compute the squared distance to the target
        distance = (robot_pose.x-self.targets[self.curr_target].x)**2 + \
                   (robot_pose.y-self.targets[self.curr_target].y)**2
        # If the distance is small enough, proceed to the next target
        if(distance < self.min_distance):
            self.curr_target = (self.curr_target + 1) % self.num_targets
            self.get_logger().info(f'Going for target {self.curr_target+1}')
            # Recompute distance to the new target
            distance = (robot_pose.x-self.targets[self.curr_target].x)**2 + \
                       (robot_pose.y-self.targets[self.curr_target].y)**2

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = world2Local(robot_pose,
                                       self.targets[self.curr_target])
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = self.Kp_ang_vel * angle_to_target

        # The linear velocity will be proportional to the distance, increased
        # with the target velocity. We actually use the squared distance just
        # for performance reasons.
        if(abs(angle_to_target) < self.max_angle_to_target):
            lin_vel = self.Kp_lin_vel * distance + self.velocity_at_target
        else:
            lin_vel = 0.

        # Limit maximum velocities
        lin_vel = np.clip(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
        ang_vel = np.clip(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)

        # Send velocity commands
        vel_cmd = Twist()
        vel_cmd.angular.z = ang_vel
        vel_cmd.linear.x = lin_vel
        self.vel_pub.publish(vel_cmd)


def main(args=None):
    '''
    Main function.
    '''

    # Clear screen
    utils.clearTerminal()

    # Output usage information
    utils.printxy(0, 0,
                  'Basic path navigation with obstacle avoidance.\n' +
                  '------------------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our navigation node
    nav_node = BasicPathNavigation()

    # Get the node executing
    rclpy.spin(nav_node)

    # Cleanup memory and shutdown
    nav_node.destroy_node()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
