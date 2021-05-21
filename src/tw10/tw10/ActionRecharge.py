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
#
# Revision $Id$

'''@package docstring
Recharge action: Request charging.
'''

# ROS related modules
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import BatteryState

# Our modules
import tw10.myglobals as myglobals
from ar_utils.action import Recharge
from ar_utils.srv import StartCharging  # Access to the battery manager service
from action_msgs.msg import GoalStatus

# Other modules
import os
from threading import Lock, Event

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class RechargeActionServer(Node):
    '''
        Charge the robot battery, if the robot is in a charging location.
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    def __init__(self):
        super().__init__('action_recharge')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_lock = Lock()
        self.battery_level = -1.0
        self.trigger_event = Event()

        # Enable access to the battery charging service
        self.charge_battery_svc = self.create_client(
            StartCharging,
            f'{myglobals.robot_name}/battery/charge')

        # Wait for the service fo bt available
        while self.charge_battery_svc.wait_for_service(2.0) is False:
            self.get_logger().info(
                f'Executing action {ACTION_NAME}: still waiting for the ' +
                f'battery service at {myglobals.robot_name}/battery/charge...')

        # Start the action server
        self.action_server = ActionServer(
            self,
            Recharge,
            f'/{myglobals.robot_name}/{ACTION_NAME}',
            self.execute_cb,
            cancel_callback=self.cancel_cb)

    def cancel_cb(self):
        ''' Callback to call when the action is cancelled '''
        self.get_logger().info(f'{ACTION_NAME} was cancelled!')
        # Stop the robotPoseCallback callback
        self.sub_batt.destroy()
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
            self.get_logger().info(
                f'Executing action {ACTION_NAME} with battery-level ' +
                f'goal {goal.request.target_battery_level:2.2f}')
            self.goal = goal
            self.trigger_event.clear()  # Clear flag

        # Request charging to start
        svc_req = StartCharging.Request()
        svc_req.charge = True
        resp = self.charge_battery_svc.call(svc_req)
        if resp.charging is False:
            # If it fails, abort action
            goal.canceled()
            self.get_logger().warn(f'{ACTION_NAME} aborted!')
            return Recharge.Result(battery_level=self.battery_level)

        # Setup subscriber for the battery level
        self.sub_batt = self.create_subscription(
            BatteryState,
            myglobals.robot_name + '/battery/state',
            self.batteryStateCallback, 1)

        feedback = Recharge.Feedback()

        while rclpy.ok():
            # Wait for a confimation (trigger), either due to the goal having
            # succeeded, or the goal having been cancelled.
            self.trigger_event.wait()
            self.trigger_event.clear()  # Clear flag
            with self.goal_lock:
                # Check if we got a preempt request
                if self.goal.status != GoalStatus.STATUS_EXECUTING:
                    self.get_logger().warn(
                        f'{ACTION_NAME} is no longer running!')
                    # Stop the batteryStateCallback callback
                    self.sub_batt.destroy()
                    return Recharge.Result(battery_level=self.battery_level)

                # Do nothing until we have an update
                if self.battery_level >= \
                   self.goal.request.target_battery_level:
                    # Stop the batteryStateCallback callback
                    self.sub_batt.destroy()
                    # Store final result and trigger SUCCEED
                    self.goal.succeed()
                    self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                    return Recharge.Result(battery_level=self.battery_level)
                else:
                    # Publish feedback (current battery level)
                    feedback.battery_level = self.battery_level
                    self.goal.publish_feedback(feedback)

    def batteryStateCallback(self, msg: BatteryState):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        with self.goal_lock:
            # If the action is not active or a preemption was requested,
            # return immediately
            if self.goal.is_active and \
               (self.goal.status == GoalStatus.STATUS_EXECUTING):
                # Store the robot pose
                self.battery_level = msg.percentage
            else:
                return

        # Trigger update
        self.trigger_event.set()  # Trigger execute_cb to continue


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    recharge_action_server = RechargeActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(recharge_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
