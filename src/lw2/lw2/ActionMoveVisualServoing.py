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

'''@package docstring
MoveVisualServoing action: move to the closest marker until the marker relative
position and bearing are smaller than a the desired values.
'''

# Non-ROS modules
import os
from threading import Lock, Event
import functools

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Our modules
import lw2.myglobals as myglobals
from ar_utils.action import MoveVisualServoing
from markers_msgs.msg import Markers

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class MoveVisualServoingActionServer(Node):
    '''
        Given a maximum distance and bearing, move the robot until its relative
        position to the nearest marker is smaller that the desired distance and
        bearing.
    '''
    def __init__(self):
        super().__init__('action_move_visual_servoing')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        self.curr_detected_markers = None

        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            MoveVisualServoing,
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
        self.get_logger().info(f'{ACTION_NAME} received new goal request')
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
        ''' Callback that's called when an action cancellation is requested '''
        self.get_logger().info(f'{ACTION_NAME} received a cancel request!')
        # The cancel request was accepted
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        ''' Callback to execute when the action has a new goal '''
        self.get_logger().info(
            f'Executing action {ACTION_NAME} with maximum distance ' +
            f'[{goal_handle.request.max_bearing:0.2f}' +
            ', and maximum bearing' +
            f' {goal_handle.request.max_distance:0.2f}]. [m]')

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        trigger_event = Event()  # Flag is intially set to False

        # Setup subscriber for the markers
        sub_pose = self.create_subscription(
                Markers,
                f'/{myglobals.robot_name}/markers',
                functools.partial(self.markersCallback,
                                  goal_handle=goal_handle,
                                  trigger_event=trigger_event),
                1,
                callback_group=ReentrantCallbackGroup())

        # Used for feedback purposes
        feedback = MoveVisualServoing.Feedback()

        while rclpy.ok():
            # Wait for new information to arrive
            if trigger_event.wait(5.0) is False:
                self.get_logger().warn(f'{ACTION_NAME} is still running')
            else:
                # If the event was triggered, clear it
                trigger_event.clear()

            with self.goal_lock:
                # Check if the goal is no longer active or if a cancel was
                # requested.
                if (not goal_handle.is_active) or \
                   (goal_handle.is_cancel_requested):
                    if not goal_handle.is_active:
                        self.get_logger().info(f'{ACTION_NAME}: goal aborted')
                    else:  # goal_handle.is_cancel_requested
                        goal_handle.canceled()  # Confirm goal is canceled
                        self.get_logger().info(f'{ACTION_NAME}: goal canceled')
                    # No need for the callback anymore
                    self.destroy_subscription(sub_pose)
                    # Return whatever result we have so far
                    return MoveVisualServoing.Result(success=False)

                ''' Control the robot velocity to reach the desired goal '''

                # Determine the marker to approach
                #--> CODE HERE!

                if False:
                    # If the maker is closer than the desired distance,and the
                    # angle is lower than the desired angle, return
                    # successfully.

                    #--> CODE HERE

                    # No need for the callback anymore
                    self.destroy_subscription(sub_pose)
                    # Stop the robot
                    self.vel_cmd.angular.z = 0.0
                    self.vel_cmd.linear.x = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    # We are done!
                    goal_handle.succeed()
                    self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                    return MoveVisualServoing.Result(success=True)

                # Otherwise, compute the linear and angular velocity to
                # approach the closest marker

                #--> CODE HERE

                # Publish velocty commands

                #--> CODE HERE

                # Publish feedback (current distance and bearing to the marker
                # being approached).
                #--> CODE HERE: UPDATE THESE LINES GIVEN YOUR CODE ABOVE
                feedback.distance = 0.0
                feedback.bearing = 0.0
                goal_handle.publish_feedback(feedback)

    def markersCallback(self, msg: Markers, goal_handle, trigger_event):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        with self.goal_lock:
            # If the goal is not active, there is nothing to do here
            if not goal_handle.is_active:
                self.get_logger().warn(
                    f'{ACTION_NAME} callback called without active goal!')
                return

            # Store current pose
            self.curr_detected_markers = msg

            # Trigger execute_cb to continue
            trigger_event.set()


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    move_vis_serv_action_server = MoveVisualServoingActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(move_vis_serv_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
