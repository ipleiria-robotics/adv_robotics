#!/usr/bin/env python3

# Copyright (c) 2025, Hugo Costelha
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


"""Simple robotic task implementaion using ROS2 actions.

Execute a simple task, formed by the following sequential execution:
  - Action Move2Pose to pose (0.0 m, -2.0 m, -90 deg)
  - Action Stop
  - Action Recharge to 90%.
"""

# Our libraries and functions
import ar_utils.action as action
import tw10.myglobals as myglobals

# ROS API
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

# Other libraries
from enum import Enum


class States(Enum):
    STATE_MOTION = 'STATE_MOTION'
    STATE_STOP = 'STATE_STOP'
    STATE_RECHARGING = 'STATE_RECHARGING'
    STATE_FAILURE = 'STATE_FAILURE'
    STATE_SUCCESS = 'STATE_SUCCESS'


class SimpleTask(Node):
    def __init__(self) -> None:
        super().__init__('tw10_simple_task')
        # Set the initial state
        self.state = States.STATE_MOTION
        # Build internal states structure for calling the correspond
        # methods
        self.state_call = {
            'STATE_MOTION': self.STATE_MOTION,
            'STATE_STOP': self.STATE_STOP,
            'STATE_RECHARGING': self.STATE_RECHARGING,
            'STATE_FAILURE': self.STATE_FAILURE,
            'STATE_SUCCESS': self.STATE_SUCCESS
        }

    def createActionClients(self):
        ''' Create all the action clients. It will wait for each action server
        to be available.
        '''

        # Create the client for each action
        self.clientMove2Pose = ActionClient(
            self, action.Move2Pose, 'ActionMove2Pose')
        self.clientSpeakText = ActionClient(
            self, action.SpeakText, 'ActionSpeakText')
        self.clientRecharge = ActionClient(
            self, action.Recharge, 'ActionRecharge')
        self.clientStop = ActionClient(
            self, action.Stop, 'ActionStop')
        # Wait for each action server to be up and running
        self.clientMove2Pose.wait_for_server()
        self.clientSpeakText.wait_for_server()
        self.clientRecharge.wait_for_server()
        self.clientStop.wait_for_server()

    '''
    State methods
    '''

    def STATE_MOTION(self):
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal position and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Move2Pose.Goal(target_position=Pose2D(
            x=myglobals.recharge_targets_wpose.x,
            y=myglobals.recharge_targets_wpose.y,
            theta=myglobals.recharge_targets_wpose.theta))
        result_response = self.clientMove2Pose.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('Action Move2Pose failed with result:' +
                                   result_response.result.success.__str__())
            # Select nest state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info('Action Move2Pose suceeded with result:' +
                                   result_response.result.success.__str__())
            # Select next state
            self.state = States.STATE_STOP

    def STATE_STOP(self):
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal stop (empty) send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Stop.Goal()  # Nothing specific
        result_response = self.clientStop.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action Stop failed with result:' +
                                   result_response.result.is_stopped.__str__())
            # Select nest state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info('Action Stop suceeded with result:' +
                                   result_response.result.is_stopped.__str__())
            # Select next state
            self.state = States.STATE_RECHARGING

    def STATE_RECHARGING(self):
        self.get_logger().info(f'Running state {self.state.value}...')

        # Create the goal battery level and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = action.Recharge.Goal(target_battery_level=1.0)
        result_response = self.clientRecharge.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Action Recharge failed with result:' +
                result_response.result.battery_level.__str__())
            # Select nest state
            self.state = States.STATE_FAILURE
        else:
            self.get_logger().info(
                'Action Recharge suceeded with result:' +
                result_response.result.battery_level.__str__())

            # In this case, run the speak text action
            goal = action.SpeakText.Goal(
                text_to_speak='Battery is now fully charged!')
            result_response = self.clientSpeakText.send_goal(goal)
            if result_response.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Action SpeakText failed!')
            else:
                self.get_logger().info('Action SpeakText suceeded!')

            # Select next state
            self.state = States.STATE_SUCCESS

    def STATE_FAILURE(self):
        self.get_logger().info(f'Running state {self.state}...')

        # In this case, shutdown
        rclpy.shutdown()

    def STATE_SUCCESS(self):
        self.get_logger().info(f'Running state {self.state}...')

        # In this case, shutdown
        rclpy.shutdown()

    def runTask(self):
        ''' Execute a sequence of pre-defined actions'''
        while rclpy.ok():
            # Execute the current state
            self.state_call[self.state.value]()

        self.get_logger().info('Done...')


def main(args=None):
    ''' Main function - runs a simple sequential task with few actions.
    '''

    rclpy.init(args=args)

    # Initializes a node so that the action client can publish and subscribe
    # over ROS.
    simple_task = SimpleTask()

    # Create the actions clients (wait for the action servers to be available)
    simple_task.createActionClients()

    # We will use several threads to make sure that the callbacks are spinned
    # while we are waiting for the goal requests to finish, i.e., while we wait
    # for the action servers responses
    executor = MultiThreadedExecutor(num_threads=2)
    # Include our Node callbacks
    executor.add_node(simple_task)
    # Include our main task function
    executor.create_task(simple_task.runTask)
    # Spin both the Node and the function runTask
    executor.spin()

    print('Done...')


if __name__ == '__main__':
    main()
