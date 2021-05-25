#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
# All rights reserved
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
#
# Revision $Id$

'''@package docstring
Stop action: stop the robot.
'''

# Non-ROS modules
import os
from threading import Lock, Event
from numpy import sqrt
import functools

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Our modules
import tw10.myglobals as myglobals
from ar_utils.action import Stop


# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class StopActionServer(Node):
    '''
        Stop the robot motion.
    '''
    def __init__(self):
        super().__init__('action_stop')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()
        self.curr_odom = None

        # Setup publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist, f'/{myglobals.robot_name}/cmd_vel', 1)
        self.vel_cmd = Twist()  # Velocity commands

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Stop,
            f'/{myglobals.robot_name}/{ACTION_NAME}',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            cancel_callback=self.cancel_cb,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        ''' Destructor '''
        self.action_server.destroy()
        super().destroy_node()

    def goal_cb(self, goal_request):
        '''This function is called when a new goal is requested. Currently it
        always accept a new goal.'''
        self.get_logger().info(f'{ACTION_NAME} received new goal request:')
        return GoalResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        ''' This function runs whenever a new goal is accepted.'''
        with self.goal_lock:
            # This server only allows one goal at a time
            if (self.goal_handle is not None) and (self.goal_handle.is_active):
                self.get_logger().info(f'{ACTION_NAME} aborting previous goal')
                # Abort the existing goal
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        # Start runing the execute callback
        goal_handle.execute()

    def cancel_cb(self, goal_handle):
        ''' Callback to call when the action is cancelled '''
        self.get_logger().info(f'{ACTION_NAME} was cancelled!')
        # Change the action status to cancelled
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        ''' Callback to call when the action as a new goal '''
        self.get_logger().info(f'Executing action {ACTION_NAME}')

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        trigger_event = Event()  # Flag is intially set to False

        sub_odom = self.create_subscription(
            Odometry,
            myglobals.robot_name + '/odom',
            functools.partial(self.robotOdomCallback,
                              goal_handle=goal_handle,
                              trigger_event=trigger_event),
            1,
            callback_group=ReentrantCallbackGroup())

        while rclpy.ok():
            # Wait for a confimation (trigger), either due to the goal having
            # succeeded, or the goal having been cancelled.
            trigger_event.wait()
            with self.goal_lock:
                # Wait for new information to arrive
                if trigger_event.wait(5.0) is False:
                    self.get_logger().warn(
                        f'{ACTION_NAME} is still running')
                else:
                    # If the event was triggered, clear it
                    trigger_event.clear()

                with self.goal_lock:
                    # Check if the goal is no longer active or if a cancel was
                    # requested.
                    if (not goal_handle.is_active) or \
                       (goal_handle.is_cancel_requested):
                        if not goal_handle.is_active:
                            self.get_logger().info(
                                f'{ACTION_NAME}: goal aborted')
                        else:  # goal_handle.is_cancel_requested
                            goal_handle.canceled()  # Confirm goal is canceled
                            self.get_logger().info(
                                f'{ACTION_NAME}: goal canceled')
                        # No need for the callback anymore
                        self.destroy_subscription(sub_odom)
                        return Stop.Result(is_stopped=False)

                    # Check if the robot is moving
                    lin_speed = sqrt(self.curr_odom.twist.twist.linear.x**2 +
                                     self.curr_odom.twist.twist.linear.y**2)
                    if (lin_speed > 0.001) or \
                       (self.curr_odom.twist.twist.angular.z > 0.002):
                        # Ask the robot to stop
                        self.vel_cmd.angular.z = 0.0
                        self.vel_cmd.linear.x = 0.0
                        self.vel_pub.publish(self.vel_cmd)
                    else:  # The robot is stopped, we are done!
                        # No need for the callback anymore
                        self.destroy_subscription(sub_odom)
                        goal_handle.succeed()
                        self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                        return Stop.Result(is_stopped=self.goal_reached)

    def robotOdomCallback(self, msg: Odometry, goal_handle, trigger_event):
        '''
        Check current velocity and, if the robot is not stopped, stop it.
        '''
        with self.goal_lock:
            # If the goal is not active, there is nothing to do here
            if not goal_handle.is_active:
                self.get_logger().warn(
                    f'{ACTION_NAME} callback called without active goal!')
                return

            # Store odometry information
            self.curr_odom = msg

            # Trigger execute_cb to continue
            trigger_event.set()


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    move2pos_action_server = StopActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(move2pos_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
