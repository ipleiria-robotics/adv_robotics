#!/usr/bin/env python3

# Copyright (c) 2023, Hugo Costelha
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
Move2Pos action: given a 2D position (X and Y), move to that position.
'''

# Non-ROS modules
from math import radians, atan2, sqrt
import os
from threading import Lock, Event
import functools

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped

# Our modules
import lw2.myglobals as myglobals
from ar_utils.action import Move2Pos
from ar_py_utils.utils import quaternionToYaw, clipValue
import ar_py_utils.LocalFrameWorldFrameTransformations as lfwft

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Move2PosActionServer(Node):
    '''
        Given a position goal, move the robot until that position is
        reached, independently of the orientation.
    '''
    def __init__(self):
        super().__init__('action_move2pos')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        self.curr_pose = Pose2D()

        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_lin_vel = 0.5  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 0.5  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum accepted distance to target [m]
        self.max_angle_to_target = radians(30.0)
        self.velocity_at_target = 0.0

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist, f'/{myglobals.robot_name}/cmd_vel', 1)
        # Pose subscriber (to be used later on)
        self.sub_pose = None

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Move2Pos,
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
            f'Executing action {ACTION_NAME} with goal position ' +
            f'[{goal_handle.request.target_position.x:0.2f},' +
            f' {goal_handle.request.target_position.y:0.2f}]. [m]')

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        trigger_event = Event()  # Flag is intially set to False

        # Setup subscriber for pose
        # The majority of the work will be done in the robotPoseCallback
        with self.goal_lock:
            if self.sub_pose is None:
                self.sub_pose = self.create_subscription(
                        PoseWithCovarianceStamped,
                        myglobals.robot_name + '/pose',
                        functools.partial(self.robotPoseCallback,
                                          goal_handle=goal_handle,
                                          trigger_event=trigger_event),
                        1,
                        callback_group=ReentrantCallbackGroup())

        # Desired position
        target_position = lfwft.Point2D(
            goal_handle.request.target_position.x,
            goal_handle.request.target_position.y)
        # Used for feedback purposes
        feedback = Move2Pos.Feedback()

        while rclpy.ok():
            # Wait for new information to arrive
            if trigger_event.wait(5.0) is False:
                self.get_logger().warn(f'{ACTION_NAME} is still running')
                # If, for some reason, we are not subscribed eyet, subscribe
                with self.goal_lock:
                    if self.sub_pose is None:
                        self.sub_pose = self.create_subscription(
                                PoseWithCovarianceStamped,
                                myglobals.robot_name + '/pose',
                                functools.partial(self.robotPoseCallback,
                                                  goal_handle=goal_handle,
                                                  trigger_event=trigger_event),
                                1,
                                callback_group=ReentrantCallbackGroup())
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
                    if self.sub_pose is not None:
                        self.destroy_subscription(self.sub_pose)
                        self.sub_pose = None
                    # Return whatever result we have so far
                    return Move2Pos.Result(final_pose=self.curr_pose)

                ''' Control de robot velocity to reach the desired goal '''

                # The angular velocity will be proportional to the angle of the
                # target as seen by the robot.
                target_local_pos = lfwft.world2Local(self.curr_pose,
                                                     target_position)
                angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
                ang_vel = self.Kp_ang_vel * angle_to_target

                # We will not update the linear velocity if the robot is not
                # facing the target enough. If it is, then the linear velocity
                # will be proportional to the distance, increased with the
                # target velocity. We actually use the squared distance just
                # for performance reasons.
                distance = sqrt((self.curr_pose.x-target_position.x)**2 +
                                (self.curr_pose.y-target_position.y)**2)
                if abs(angle_to_target) < self.max_angle_to_target:
                    lin_vel = self.Kp_lin_vel * distance + \
                              self.velocity_at_target
                else:
                    lin_vel = 0.0

                # Limit maximum velocities
                lin_vel = clipValue(
                    lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
                ang_vel = clipValue(
                    ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

                # Did we reach the goal?
                if distance < self.min_distance:
                    # No need for the callback anymore
                    if self.sub_pose is not None:
                        self.destroy_subscription(self.sub_pose)
                        self.sub_pose = None
                    # Stop the robot
                    self.vel_cmd.angular.z = 0.0
                    self.vel_cmd.linear.x = 0.0
                    self.vel_pub.publish(self.vel_cmd)
                    # We are done!
                    goal_handle.succeed()
                    self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                    return Move2Pos.Result(final_pose=self.curr_pose)
                else:
                    # Send velocity commands
                    self.vel_cmd.angular.z = ang_vel
                    self.vel_cmd.linear.x = lin_vel
                    self.vel_pub.publish(self.vel_cmd)

                    # Publish feedback (current pose)
                    feedback.base_pose = self.curr_pose
                    goal_handle.publish_feedback(feedback)

    def robotPoseCallback(self, msg: PoseWithCovarianceStamped, goal_handle,
                          trigger_event):
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
            self.curr_pose = Pose2D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                theta=quaternionToYaw(msg.pose.pose.orientation))

            # Trigger execute_cb to continue
            trigger_event.set()


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    move2pos_action_server = Move2PosActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(move2pos_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
