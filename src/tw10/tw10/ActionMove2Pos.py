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
Move2Pos action: given a 2D position (X and Y), move to that position.
'''

# ROS related modules
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist, PoseStamped
from action_msgs.msg import GoalStatus

# Our modules
import tw10.myglobals as myglobals
from ar_utils.action import Move2Pos
from ar_utils.utils import quaternionToYaw, clipValue
import ar_utils.LocalFrameWorldFrameTransformations as lfwft

# Other modules
from math import radians, atan2, sqrt
import os
from threading import Lock, Event

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
        self.goal_lock = Lock()
        self.trigger_event = Event()  # Flag is intially set to False
        self.goal_reached = False  # Keep track of the current goal status

        ''' Initialize members for navigation control '''
        self.curr_pose = Pose2D()

        # Robot navigation/motion related constants and variables
        self.MAX_LIN_VEL = 1.0  # Maximum linear speed [m/s]
        self.MAX_ANG_VEL = 1.57  # Maximu angular speed (90°/s) [rad/s]

        # Navigation variables
        self.Kp_lin_vel = 1.0  # Proportional gain for the linear vel. control
        self.Kp_ang_vel = 3.0  # Propostional gain for the angular vel. control
        self.min_distance = 0.1  # Minimum accepted distance to target [m]
        self.max_angle_to_target = radians(30.0)
        self.velocity_at_target = 0.0

        ''' ROS related code '''
        self.vel_cmd = Twist()  # Velocity commands

        # Setup publisher for velocity commands
        self.vel_pub = self.create_publisher(
            Twist, f'/{myglobals.robot_name}/cmd_vel', 1)

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            Move2Pos,
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
        ''' Callback to execute when the action has a new goal '''
        with self.goal_lock:
            self.goal_reached = False  # New goal
            self.trigger_event.clear()  # Clear flag
            self.goal = goal  # Store desired goal
        self.get_logger().info(
            f'Executing action {ACTION_NAME} with goal position ' +
            f'[{self.goal.request.target_position.x:0.2f},' +
            f' {self.goal.request.target_position.y:0.2f}]. [m]')

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
            return Move2Pos.Result(final_pose=self.curr_pose)

    def robotPoseCallback(self, msg: PoseStamped):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        with self.goal_lock:
            # If the action is not active or a cancel was requested,
            # return immediately
            if (not self.goal.is_active) or self.goal.is_cancel_requested:
                return

            # Else, get the desired orientation
            target_position = lfwft.Point2D(
                self.goal.request.target_position.x,
                self.goal.request.target_position.y)
            # Store current pose
            self.curr_pose = Pose2D(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                theta=quaternionToYaw(msg.pose.orientation))

        # The angular velocity will be proportional to the angle of the target
        # as seen by the robot.
        target_local_pos = lfwft.world2Localp(self.curr_pose, target_position)
        angle_to_target = atan2(target_local_pos.y, target_local_pos.x)
        ang_vel = self.Kp_ang_vel * angle_to_target

        #  We will not update the linear velocity if the robot is not facing
        # the target enough. If it is, then the linear velocity will be
        # proportional to the distance, increased with the target velocity. We
        # actually use the squared distance just for performance reasons.
        distance = sqrt((self.curr_pose.x-target_position.x)**2 +
                        (self.curr_pose.y-target_position.y)**2)
        if abs(angle_to_target) < self.max_angle_to_target:
            lin_vel = self.Kp_lin_vel * distance + self.velocity_at_target
        else:
            lin_vel = 0.0

        # Limit maximum velocities
        lin_vel = clipValue(lin_vel, -self.MAX_LIN_VEL, self.MAX_LIN_VEL)
        ang_vel = clipValue(ang_vel, -self.MAX_ANG_VEL, self.MAX_ANG_VEL)

        # Publish feedback (current pose)
        feedback = Move2Pos.Feedback()
        feedback.base_pose = self.curr_pose
        self.goal.publish_feedback(feedback)

        # Send velocity commands
        self.vel_cmd.angular.z = ang_vel
        self.vel_cmd.linear.x = lin_vel
        self.vel_pub.publish(self.vel_cmd)

        # Did we reach the goal?
        if distance < self.min_distance:
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
    move2pos_action_server = Move2PosActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(move2pos_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
