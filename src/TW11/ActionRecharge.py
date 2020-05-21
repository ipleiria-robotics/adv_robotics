#!/usr/bin/env python3

# Copyright (c) 2020, Hugo Costelha
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
import rospy
import actionlib
from std_msgs.msg import UInt8

# Our modules
import myglobals
import tw11.msg  # Import our (action generated) messages
import tw11.srv  # Access to the battery manager service

# Other modules
import os
from threading import Lock

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class Recharge:
    '''
        Recharge action

        Charge the robot battery, if the robot is in a charging location.
    '''
    # Store the action feecback and result functions as class attributes,
    # i.e., they are the same for all instances of this class
    feedback = tw11.msg.RechargeFeedback()
    result = tw11.msg.RechargeResult()

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            name=ACTION_NAME, ActionSpec=tw11.msg.RechargeAction,
            execute_cb=self.execute_cb, auto_start=False)

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.battery_level_lock = Lock()
        self.battery_level = 0

        # Start the action server
        self.action_server.start()

    def execute_cb(self, goal):
        rospy.loginfo(
            f'Executing action {ACTION_NAME}' +
            f' with battery-level goal {goal.target_battery_level} %')
        self.goal = goal

        # Enable access to the battery charging service
        charge_battery = rospy.ServiceProxy(
            myglobals.robot_name + '/battery/charge',
            tw11.srv.StartCharging)

        # Request charging to start
        resp = charge_battery(True)
        if resp.charging is False:
            # If it fails, abort action
            self.action_server.set_aborted(
                text='Unable to start charging!')
            rospy.loginfo(f'{ACTION_NAME} has aborted!')
            return

        # Otherwise the robot is charging
        self.battery_level_updated = False

        # Setup subscriber for the battery level
        self.sub_batt = rospy.Subscriber(
            myglobals.robot_name + '/battery/level', UInt8,
            self.batteryLevelCallback, queue_size=1)

        rate = rospy.Rate(myglobals.execution_rate)
        while not rospy.is_shutdown():
            # Check if we got a preempt request
            if self.action_server.is_preempt_requested():
                rospy.loginfo(f'{ACTION_NAME} was Preempted!')
                # Stop the batteryLevelCallback callback
                self.sub_batt.unregister()
                self.action_server.set_preempted()
                return

            with self.battery_level_lock:
                # Do nothing until we have an update
                if self.battery_level_updated:
                    if self.battery_level >= self.goal.target_battery_level:
                        # Stop the batteryLevelCallback callback
                        self.sub_batt.unregister()
                        # Store final result and trigger SUCCEED
                        self.result.battery_level = self.battery_level
                        self.action_server.set_succeeded(self.result)
                        rospy.loginfo(f'{ACTION_NAME} has succeeded!')
                        return
                    else:
                        # Publish feedback (current battery level)
                        self.feedback.battery_level = self.battery_level
                        self.action_server.publish_feedback(self.feedback)

            rate.sleep()

    def batteryLevelCallback(self, msg: UInt8):
        '''
        Receive current robot pose and change its velocity accordingly
        '''
        # If the action is not active or a preemption was requested,
        # return immediately
        if (not self.action_server.is_active()) or \
           self.action_server.is_preempt_requested():
            return

        # Store the robot pose
        with self.battery_level_lock:
            self.battery_level_updated = True
            self.battery_level = msg.data


if __name__ == '__main__':
    # Init ROS node
    rospy.init_node(ACTION_NAME)
    # Create and start the action server
    server = Recharge()
    # Spin off all callbacks
    rospy.spin()
