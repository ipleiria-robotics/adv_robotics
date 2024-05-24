#!/usr/bin/env python3

# Copyright (c) 2024, Hugo Costelha
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
from sensor_msgs.msg import BatteryState

# Our modules
from ar_utils.action import Recharge
from ar_utils.srv import StartCharging  # Access to the battery manager service


# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class RechargeActionServer(Node):
    '''
        Charge the robot battery, if the robot is in a charging location.
        This action accepts a single goal at a time so, requesting a new goal
        while a previous one was alread running, cancels the previous goal.
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    def __init__(self):
        '''Constructor'''
        super().__init__('action_recharge')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()
        self.battery_level = -1.0

        # Enable access to the battery charging service
        self.charge_battery_svc = self.create_client(
            StartCharging,
            'battery/charge')

        # Wait for the service fo bt available
        while self.charge_battery_svc.wait_for_service(2.0) is False:
            self.get_logger().info(
                f'Executing action {ACTION_NAME}: still waiting for the ' +
                'battery service at battery/charge...')

        # Battery state subscriber (to be used later on)
        self.sub_batt = None

        # Start the action server.
        # The option ReentrantCallbackGroup allows callbacks to be run in
        # parallel without restrictions
        self.action_server = ActionServer(
            self,
            Recharge,
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
        ''' Callback that's called when an action cancellation is requested '''
        self.get_logger().info(f'{ACTION_NAME} received a cancel request!')
        # The cancel request was accepted
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        ''' Callback to execute when the action has a new goal '''
        self.get_logger().info(
            f'Executing action {ACTION_NAME} with battery-level ' +
            f'goal {goal_handle.request.target_battery_level:2.2f}')

        # Wait for a confimation (trigger), either due to the goal having
        # succeeded, or the goal having been cancelled.
        trigger_event = Event()  # Flag is intially set to False

        # Request charging to start
        svc_req = StartCharging.Request()
        svc_req.charge = True
        resp = self.charge_battery_svc.call(svc_req)
        if resp.charging is False:
            # If it fails, abort action
            goal_handle.abort()
            self.get_logger().warn(
                f'{ACTION_NAME}  - unable to start charging!')
            return Recharge.Result(battery_level=self.battery_level)

        # Setup subscriber for the battery level
        with self.goal_lock:
            if self.sub_batt is None:
                self.sub_batt = self.create_subscription(
                    BatteryState,
                    'battery/state',
                    functools.partial(self.batteryStateCb,
                                      goal_handle=goal_handle,
                                      trigger_event=trigger_event),
                    1,
                    callback_group=ReentrantCallbackGroup())

        # Used for feedback purposes
        feedback = Recharge.Feedback()

        while rclpy.ok():
            # Wait for new information to arrive
            if trigger_event.wait(5.0) is False:
                self.get_logger().warn(f'{ACTION_NAME} is still running')
                with self.goal_lock:
                    if self.sub_batt is None:
                        self.sub_batt = self.create_subscription(
                            BatteryState,
                            'battery/state',
                            functools.partial(self.batteryStateCb,
                                              goal_handle=goal_handle,
                                              trigger_event=trigger_event),
                            1,
                            callback_group=ReentrantCallbackGroup())
            else:
                # If the event was triggered, clear it
                trigger_event.clear()

            with self.goal_lock:
                # Only continue if the goal is active and a cancel was not
                # requested
                if (not goal_handle.is_active) or \
                   (goal_handle.is_cancel_requested):
                    if not goal_handle.is_active:
                        self.get_logger().info(f'{ACTION_NAME}: goal aborted')
                    else:  # goal_handle.is_cancel_requested
                        goal_handle.canceled()  # Confirm goal is canceled
                        self.get_logger().info(f'{ACTION_NAME}: goal canceled')
                    # No need for the callback anymore
                    if self.sub_batt is not None:
                        self.destroy_subscription(self.sub_batt)
                        self.sub_batt = None
                    # Cancel the recharging and return
                    svc_req = StartCharging.Request()
                    svc_req.charge = False
                    resp = self.charge_battery_svc.call(svc_req)
                    return Recharge.Result(battery_level=self.battery_level)

                # Do nothing until we have an update
                if self.battery_level >= \
                   goal_handle.request.target_battery_level:
                    # We are done, no need for the callback anymore
                    if self.sub_batt is not None:
                        self.destroy_subscription(self.sub_batt)
                        self.sub_batt = None
                    # Trigger SUCCEED
                    goal_handle.succeed()
                    self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                    # Cancel the recharging and return
                    svc_req = StartCharging.Request()
                    svc_req.charge = False
                    resp = self.charge_battery_svc.call(svc_req)
                    # Return last reported battery level
                    return Recharge.Result(battery_level=self.battery_level)
                else:
                    # Publish feedback (current battery level)
                    feedback.battery_level = self.battery_level
                    goal_handle.publish_feedback(feedback)

    def batteryStateCb(self, msg: BatteryState, goal_handle, trigger_event):
        '''
        Receive current robot battery charge
        '''
        with self.goal_lock:
            # Check if we are still "in business"
            if not goal_handle.is_active:
                self.get_logger().warn(
                    f'{ACTION_NAME} callback called without active goal!')
                return

            # Store the robot pose
            self.battery_level = msg.percentage

            # Trigger execute_cb to continue
            trigger_event.set()


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    recharge_action_server = RechargeActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(recharge_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
