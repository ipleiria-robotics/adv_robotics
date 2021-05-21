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

# ROS related modules
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

# Our modules
import tw10.myglobals as myglobals
from ar_utils.action import Stop

# Other modules
import os
from threading import Lock, Event
from numpy import sqrt

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
        self.goal_lock = Lock()
        self.trigger_event = Event()  # Flag is intially set to False
        self.goal_reached = False  # Keep track of the current goal status

        # Setup publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist, f'/{myglobals.robot_name}/cmd_vel', 1)
        self.vel_cmd = Twist()  # Velocity commands

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Stop,
            f'/{myglobals.robot_name}/{ACTION_NAME}',
            self.execute_cb,
            cancel_callback=self.cancel_cb)

    def cancel_cb(self):
        ''' Callback to call when the action is cancelled '''
        self.get_logger().info(f'{ACTION_NAME} was cancelled!')
        # Stop the robotPoseCallback callback
        self.sub_odom.destroy()
        # Allow the trigger callback to finish
        self.trigger_event.set()
        # Update internal information
        with self.goal_lock:
            if self.goal.status != GoalStatus.STATUS_CANCELED:
                self.goal.canceled()
        # Change the action status to cancelled
        return CancelResponse.ACCEPT

    def execute_cb(self, goal):
        ''' Callback to call when the action as a new goal '''
        with self.goal_lock:
            self.goal_reached = False  # New goal
            self.trigger_event.clear()  # Clear flag
            self.goal = goal  # Store desired goal
        self.get_logger().info(
            f'Executing action {ACTION_NAME}')

        # Setup subscriber for pose
        # The majority of the work will be done in the robotPoseCallback
        self.sub_odom = self.create_subscription(
            Odometry,
            myglobals.robot_name + '/odom',
            self.robotOdomCallback, 1)

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        self.trigger_event.wait()
        with self.goal_lock:
            if self.goal_reached:
                self.goal.succeed()
                self.get_logger().info(f'{ACTION_NAME} has succeeded!')
            elif self.goal.status != GoalStatus.STATUS_CANCELED:
                self.goal.cancelled()
            return Stop.Result(is_stopped=self.goal_reached)

    def robotOdomCallback(self, msg: Odometry):
        '''
        Check current velocity and, if the robot is not stopped, stop it.
        '''
        with self.goal_lock:
            # If the action is not active or a preemption was requested,
            # return immediately
            if self.goal_reached or (not self.goal.is_active) or \
               self.goal.is_cancel_requested:
                return

            # Check if the robot is moving
            lin_speed = sqrt(msg.twist.twist.linear.x**2 +
                             msg.twist.twist.linear.y**2)
            if (lin_speed > 0.001) or (msg.twist.twist.angular.z > 0.002):
                # Ask the robot to stop
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0
                self.vel_pub.publish(self.vel_cmd)
            else:  # The robot is stopped
                # Stop this callback
                self.sub_odom.destroy()
                # We are done, store final result
                self.goal_reached = True
                self.trigger_event.set()  # Trigger execute_cb to continue


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
