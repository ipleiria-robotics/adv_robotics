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
Execute a simple task, formed by the following sequential execution:
  - Action Move2Pos to position (0, -2) [m]
  - Action Rotate2Angle to orientation -90 deg
  - Action Stop
  - Action Recharge to 90%
'''

# Our libraries and functions
from ar_utils.action import Move2Pos, Recharge, Rotate2Angle, Stop
import tw10.myglobals as myglobals

# ROS API
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus

# Other libraries
from numpy import pi, degrees


class SimpleTask(Node):
    def __init__(self) -> None:
        super().__init__('tw10_simple_task')

    def createActionClients(self):
        ''' Create all the action clients. It will wait for each action server
        to be available.
        '''

        # Create the client for each action
        self.clientMove2Pos = ActionClient(
            self, Move2Pos, f'/{myglobals.robot_name}/ActionMove2Pos')
        self.clientRotate2Angle = ActionClient(
            self, Rotate2Angle, f'/{myglobals.robot_name}/ActionRotate2Angle')
        self.clientRecharge = ActionClient(
            self, Recharge, f'/{myglobals.robot_name}/ActionRecharge')
        self.clientStop = ActionClient(
            self, Stop, f'/{myglobals.robot_name}/ActionStop')
        # Wait for each action server to be up and running
        self.clientMove2Pos.wait_for_server()
        self.clientRotate2Angle.wait_for_server()
        self.clientRecharge.wait_for_server()
        self.clientStop.wait_for_server()

    def runTask(self):
        ''' Execute a sequence of pre-defined actions'''

        # 1st action
        # Create the goal position and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = Move2Pos.Goal(target_position=Point(
            x=myglobals.recharge_targets_wpose.x,
            y=myglobals.recharge_targets_wpose.y,
            z=myglobals.recharge_targets_wpose.theta))
        result_response = self.clientMove2Pos.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('Action Move2Pos failed with result:' +
                                   result_response.result.final_pose.__str__())
        else:
            self.get_logger().info('Action Move2Pos suceeded with result:' +
                                   result_response.result.final_pose.__str__())

        # 2nd action
        # Create the goal angle and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = Rotate2Angle.Goal(target_orientation=-pi/2)
        result_response = self.clientRotate2Angle.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Action Rotate2Angle failed with result:' +
                f'{degrees(result_response.result.final_orientation):.2f}')
        else:
            self.get_logger().info(
                'Action Rotate2Angle suceeded with result:' +
                f'{degrees(result_response.result.final_orientation):.2f}')

        # 3rd action
        # Create the goal angle and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = Stop.Goal()  # Nothing specific
        result_response = self.clientStop.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action Stop failed with result:' +
                                   result_response.result.is_stopped.__str__())
        else:
            self.get_logger().info('Action Stop suceeded with result:' +
                                   result_response.result.is_stopped.__str__())

        # 4th action
        # Create the goal position and send it to the action server.
        # We will wait for the action to finish and return a result.
        goal = Recharge.Goal(target_battery_level=1.0)
        result_response = self.clientRecharge.send_goal(goal)
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Action Recharge failed with result:' +
                result_response.result.battery_level.__str__())
        else:
            self.get_logger().info(
                'Action Recharge suceeded with result:' +
                result_response.result.battery_level.__str__())

        self.get_logger().info('Done...')
        # Shutdown ROS
        rclpy.shutdown()


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
