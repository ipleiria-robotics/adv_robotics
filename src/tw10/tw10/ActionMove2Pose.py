#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
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
Move2Pose action: given a 2D pose (X, Y and Theta), move to that pose using a
NAV2 planner and controller.
'''

# Non-ROS modules
from math import radians, degrees
import os
from threading import Lock

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Pose2D
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Our modules
from tw10.myglobals import execution_rate
from ar_utils.action import Move2Pose
from ar_py_utils.utils import rpyToQuaternion

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Move2PoseActionServer(Node):
    '''
        Given a position goal, move the robot until that position is
        reached, independently of the orientation.
    '''
    def __init__(self):
        super().__init__('action_move2pose')

        # Nav2 initialization
        self.nav2 = BasicNavigator(namespace='/robot_0')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()

        ''' Initialize members for navigation control '''
        self.curr_pose = Pose2D()

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Move2Pose,
            ACTION_NAME,
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
            f'Executing action {ACTION_NAME} with goal pose ' +
            f'[{goal_handle.request.target_pose.x:0.2f} m,' +
            f' {goal_handle.request.target_pose.y:0.2f} m,' +
            f' {degrees(goal_handle.request.target_pose.theta):0.2f} deg]')

        # Used for feedback purposes
        feedback = Move2Pose.Feedback()

        # Compute path to target
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position = Point(x=goal_handle.request.target_pose.x,
                                     y=goal_handle.request.target_pose.y, z=0.)
        target.pose.orientation = rpyToQuaternion(
            0., 0., goal_handle.request.target_pose.theta)
        path = self.nav2.getPath(start=PoseStamped(),  # Not used
                                 goal=target,
                                 planner_id="GridBased",
                                 use_start=False)

        # If no path exists, abort action and report
        if path is None:
            self.get_logger().warn(
                f'{ACTION_NAME}: path not found to target. Abort!')
            goal_handle.abort()
            return Move2Pose.Result(success=False)

        # We have a path, lets follow it
        self.nav2.followPath(path, controller_id='FollowPath',
                             goal_checker_id='general_goal_checker')

        # Monitor the path following
        while rclpy.ok() and (not self.nav2.isTaskComplete()):
            # Publish feedback (distance to tarrget)
            feedback_nav2 = self.nav2.getFeedback()
            feedback.distance_to_goal = feedback_nav2.distance_to_goal
            goal_handle.publish_feedback(feedback)
            self.get_clock().sleep_for(Duration(seconds=1/execution_rate))

        # Do something depending on the return code
        result = self.nav2.getResult()
        if result == TaskResult.SUCCEEDED:
            # We are done!
            goal_handle.succeed()
            self.get_logger().info(f'{ACTION_NAME} has succeeded!')
            return Move2Pose.Result(success=True)
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(
                f'{ACTION_NAME}: path following canceled. Abort!')
            goal_handle.abort()
            return Move2Pose.Result(success=False)
        elif result == TaskResult.FAILED:
            self.get_logger().warn(
                f'{ACTION_NAME}: path following failed. Abort!')
            goal_handle.abort()
            return Move2Pose.Result(success=False)
        else:
            self.get_logger().warn(
                f'{ACTION_NAME}: invalid return statud. Abort!')
            goal_handle.abort()
            return Move2Pose.Result(success=False)


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    move2pose_action_server = Move2PoseActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(move2pose_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
