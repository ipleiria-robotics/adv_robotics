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
from math import radians, atan2
import numpy as np
from threading import Lock

# ROS API
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Pose2D
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# Our utility functions
import ar_py_utils.utils as utils
from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D, \
    world2LocalPoint

# The robot will not move with speeds faster than these, so we better limit out
# values
MAX_LIN_VEL = 1.0  # [m/s]
MAX_ANG_VEL = radians(90)  # [rad/s]


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


class BasicWaypointPathNavigation(Node):
    '''
    Implementation of a basic waypoint navigation. The robot rotates towards
    the goal and, once facing the goal, moves forward in direction to it.
    Obstacles are not being avoided, so the received path plan is expected to
    be obstacle-free.
    '''

    def __init__(self, tf_buffer):
        '''
        Initializes the class instance.
        '''
        # Used to avoid simultaneous access to internal variables
        self.lock = Lock()

        # TODO: these could be parameters
        # Internal navigation variables
        self.curr_target = 0  # Current target location index
        self.min_distance = 0.08  # Minimum acceptance distance to target
        self.max_angle_to_target = radians(30.0)
        self.first_run = True  # True if the callback was never called
        self.pubtimer = None  # Will hold the periodic callback

        # Followed path information
        self.global_path = None  # Will hold the path to follow
        self.curr_target_idx = None  # Will hold the current target index

        # Initialize the node itself
        super().__init__('tf_path_navigation')

        # Store our TF buffer
        self.tf_buffer = tf_buffer

        # Get our namespace (we need this for the TF)
        self.namespace = self.get_namespace()[1:]

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
        # Minimum linear velocity throughout the navigation
        base_lin_vel_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Linear velocity at the target!')
        self.declare_parameter('base_lin_vel', 0.1,
                               base_lin_vel_param_desc)
        # Linear velocity proportional controller gain
        kp_lin_vel_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Linear velocity proportional controller gain!')
        self.declare_parameter('kp_lin_vel', 1.0,  kp_lin_vel_param_desc)
        # Angular velocity proportional controller gain
        kp_ang_vel_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Angular velocity proportional controller gain!')
        self.declare_parameter('kp_ang_vel', 3.0,  kp_ang_vel_param_desc)

        # Subscribe the path plan to follow
        self.path_sub = self.create_subscription(Path, 'path', self.path_cb, 1)

        # Setup velocity publisher
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def path_cb(self, msg):
        '''
        Called whenever a new path is received. It will trigger the path
        following, which is done in the pose_cb.
        '''

        with self.lock:
            # Store an internal copy of the path
            self.global_path = msg.poses
            self.num_targets = len(msg.poses)
            # Start in the first pose of the path
            if (self.curr_target_idx is None) or \
               (self.curr_target_idx == self.num_targets-1):
                self.curr_target_idx = 0

            # Start the periodic callback to control the robot motion
            if self.pubtimer is not None:
                self.get_logger().warn('self.pubtimer was not None!')
                self.pubtimer.destroy()
            self.pubtimer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        '''
            Navigation callback, executes periodically and uses TF
        '''
        with self.lock:
            # If we have not received any path, or if the last one we have
            # received has already finished, there is not thing to do here.
            if self.global_path is None:
                return

            # Use TF to find our current pose (given by the localization)
            try:
                # Get the transformation from map to base_footprint
                map_to_base_footprint_trans = self.tf_buffer.lookup_transform(
                    'map', f'{self.namespace}/base_footprint',
                    rclpy.time.Time(), Duration(seconds=0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f'No required transformation found: {e}')
                return

            # Store the result
            robot_pose = Pose2D(
                x=map_to_base_footprint_trans.transform.translation.x,
                y=map_to_base_footprint_trans.transform.translation.y,
                theta=utils.quaternionToYaw(
                    map_to_base_footprint_trans.transform.rotation))

            # Get current target
            self.curr_target = Point2D(
                    self.global_path[self.curr_target_idx].pose.position.x,
                    self.global_path[self.curr_target_idx].pose.position.y)

            # Compute the squared distance to the target
            distance = (robot_pose.x-self.curr_target.x)**2 + \
                       (robot_pose.y-self.curr_target.y)**2
            # If the distance is small enough, proceed to the next target
            if (distance < self.min_distance):
                # If there are not more targets, stop the robot and return
                if self.curr_target_idx == self.num_targets-1:
                    # Stop the robot
                    vel_cmd = Twist()
                    vel_cmd.angular.z = 0.0
                    vel_cmd.linear.x = 0.0
                    self.vel_pub.publish(vel_cmd)
                    # Forget the path and return
                    self.global_path = None
                    self.get_logger().info('Finished path navigation')
                    # Stop the periodic callback
                    self.pubtimer.destroy()
                    self.pubtimer = None
                    return
                # Else, proceed with the next target
                self.curr_target_idx += 1
                self.get_logger().info(
                    f'Going for target {self.curr_target_idx}')
                # Update target and recompute distance
                self.curr_target = Point2D(
                    self.global_path[self.curr_target_idx].pose.position.x,
                    self.global_path[self.curr_target_idx].pose.position.y)
                # Recompute distance to the new target
                distance = (robot_pose.x-self.curr_target.x)**2 + \
                           (robot_pose.y-self.curr_target.y)**2

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = world2LocalPoint(robot_pose, self.curr_target)
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        kp_ang_vel = self.get_parameter(
            'kp_ang_vel').get_parameter_value().double_value
        ang_vel = kp_ang_vel * angle_to_target

        # The linear velocity will be proportional to the distance, increased
        # with the target velocity. We actually use the squared distance just
        # for performance reasons.
        base_lin_vel = self.get_parameter(
            'base_lin_vel').get_parameter_value().double_value
        if (abs(angle_to_target) < self.max_angle_to_target):
            kp_lin_vel = self.get_parameter(
                'kp_lin_vel').get_parameter_value().double_value
            lin_vel = kp_lin_vel * distance + base_lin_vel
        else:
            lin_vel = base_lin_vel

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
                  'Basic waypoint path navigation.\n' +
                  '-------------------------------\n')

    # Initiate python ROS Python control
    rclpy.init(args=args)

    # Create our TF listener node
    tf_listener_node = TfListener()
    # Create our navigation node, using our TF buffer
    nav_node = BasicWaypointPathNavigation(
        tf_buffer=tf_listener_node.tf_buffer)

    # We will execute each node in its own thread. This is important to make
    # sure that the TF listener is continuously updated.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tf_listener_node)
    executor.add_node(nav_node)

    # Run both nodes until shutdown
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()


'''
This is what is actually called when we run this python script. It then calls
the main function defined above.
'''
if __name__ == '__main__':
    main()
