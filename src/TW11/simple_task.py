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
Execute a simple task, formed by the following sequential execution:
  - Action Move2Pos to position (0, -2) [m]
  - Action Rotate2Angle to orientation -90 deg
  - Action Stop
  - Action Recharge to 90%
'''

# Our libraries and functions
import tw11.msg
import myglobals

# ROS API
import rospy
import actionlib
from geometry_msgs.msg import Point

# Other libraries
from numpy import pi


if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('simple_task')

    # Create the client for each action
    clientMove2Pos = actionlib.SimpleActionClient(
        myglobals.robot_name + '/ActionMove2Pos', tw11.msg.Move2PosAction)
    clientRotate2Angle = actionlib.SimpleActionClient(
        myglobals.robot_name + '/ActionRotate2Angle', tw11.msg.Rotate2AngleAction)
    clientRecharge = actionlib.SimpleActionClient(
        myglobals.robot_name + '/ActionRecharge', tw11.msg.RechargeAction)
    clientStop = actionlib.SimpleActionClient(
        myglobals.robot_name + '/ActionStop', tw11.msg.StopAction)

    # Wait for each action server to be up and running
    clientMove2Pos.wait_for_server()
    clientRotate2Angle.wait_for_server()
    clientMove2Pos.wait_for_server()
    clientStop.wait_for_server()

    # 1st action
    # Create the goal position and send it to the action server
    goal = tw11.msg.Move2PosGoal(target_position=Point(0.0, -2.0, 0.0))
    clientMove2Pos.send_goal(goal)
    clientMove2Pos.wait_for_result()
    result = clientMove2Pos.get_result()
    print('Action Move2Pos ended with result:' +
          result.final_position.__str__())

    # 2nd action
    # Create the goal angle and send it to the action server
    goal = tw11.msg.Rotate2AngleGoal(target_orientation=-pi/2)
    clientRotate2Angle.send_goal(goal)
    clientRotate2Angle.wait_for_result()
    result = clientRotate2Angle.get_result()
    print('Action Rotate2Angle ended with result:' +
          result.final_orientation.__str__())

    # 3rd action
    # Create the goal angle and send it to the action server
    goal = tw11.msg.StopGoal()  # Nothing specific
    clientStop.send_goal(goal)
    clientStop.wait_for_result()
    result = clientStop.get_result()
    print('Action Stop ended with result:' +
          result.is_stopped.__str__())

    # 4th action
    # Create the goal position and send it to the action server
    goal = tw11.msg.RechargeGoal(target_battery_level=100)
    clientRecharge.send_goal(goal)
    clientRecharge.wait_for_result()
    result = clientRecharge.get_result()
    print('Action Recharge ended with result:' +
          result.battery_level.__str__())

    print('Done...')
