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

'''@package docstring
Implementation of the Rotate2Angle action: given a orientation goal, rotate
the robot until that orientation is reached.
'''

# ROS related modules
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from action_msgs.msg import GoalStatus

# Our modules
import tw10.myglobals as myglobals
from ar_msgs_srvs_actions_interfaces.action import Rotate2Angle
from tw07.utils import quaternionToYaw, clipValue

# Other modules
from math import radians
import os
from threading import Lock, Event

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Rotate2AngleActionServer(Node):
    '''
        Given a desired orientation, rotate the robot until that orientation is
        reached
    '''
    def __init__(self):
        super().__init__('action_rotate2angle')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_lock = Lock()
        self.trigger_event = Event()  # Flag is intiallt set to False
        self.goal_reached = False  # Keep track of the current goal status

        ''' Initialize members for navigation control '''
        self.curr_orientation = 0.0  # Hold the last/current robot orientation

        # Robot navigation/motion related constants and variables
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_ang_vel = 3.0  # Proportional gain for the angular vel. control
        self.max_angle_error = radians(5.0)  # Maximum angle error

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist, f'{myglobals.robot_name}/cmd_vel', 1)

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Rotate2Angle,
            f'/{myglobals.robot_name}/{ACTION_NAME}',
            self.execute_cb,
            cancel_callback=self.cancel_cb)

    def cancel_cb(self):
        ''' Callback to call when the action is cancelled '''
        self.get_logger().info(f'{ACTION_NAME} was cancelled!')
        # Stop the robotPoseCallback callback
        self.sub_pose.destroy()
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
        with self.goal_lock:  # Store desired angle
            self.goal_reached = False  # New goal
            self.trigger_event.clear()  # Clear flag
            self.goal = goal  # Store desired goal
            self.get_logger().info(
                f'Executing action {ACTION_NAME} with goal angle ' +
                f'{self.goal.request.target_orientation:.2f} [°].')

        # Setup subscriber for pose
        # The majority of the work will be done in the robotPoseCallback
        self.sub_pose = self.create_subscription(
            PoseStamped,
            myglobals.robot_name + '/pose',
            self.robotPoseCallback, 1)

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        self.trigger_event.wait()
        with self.goal_lock:
            if self.goal_reached:
                self.goal.succeed()
                self.get_logger().info(f'{ACTION_NAME} has succeeded!')
            elif self.goal.status != GoalStatus.STATUS_CANCELED:
                self.goal.cancelled()
            return Rotate2Angle.Result(final_orientation=self.curr_orientation)

    def robotPoseCallback(self, msg: PoseStamped):
        '''
        Receive current robot pose and change its velocity according to the
        orientation
        '''
        with self.goal_lock:
            # If the action is not active or a cancel was requested,
            # return immediately
            if (not self.goal.is_active) or self.goal.is_cancel_requested:
                return

            # Else, get the desired orientation
            target_orientation = self.goal.request.target_orientation

            # Store the current orientation
            self.curr_orientation = quaternionToYaw(msg.pose.orientation)

            # Use a P controller based on the desired and current angles
            error = (target_orientation - self.curr_orientation)
            ang_vel = self.Kp_ang_vel * error
            ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

            # Publish feedback (current robot orientation)
            feedback = Rotate2Angle.Feedback()
            feedback.base_orientation = self.curr_orientation
            self.goal.publish_feedback(feedback)

            # Send velocity commands
            self.vel_cmd.angular.z = ang_vel
            self.vel_cmd.linear.x = 0.0
            self.vel_pub.publish(self.vel_cmd)

            # Did we reach the goal?
            if abs(error) < self.max_angle_error:
                # Stop the robot
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0
                self.vel_pub.publish(self.vel_cmd)
                # Stop this callback
                self.sub_pose.destroy()
                # We are done!
                self.goal_reached = True
                self.trigger_event.set()  # Trigger execute_cb to continue


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    rotate2angle_action_server = Rotate2AngleActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(rotate2angle_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
