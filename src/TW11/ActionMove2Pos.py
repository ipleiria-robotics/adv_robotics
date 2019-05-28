#!/usr/bin/env python3
'''
Copyright (c) 2019, Hugo Costelha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
* Neither the name of the Player Project nor the names of its contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 4
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Revision $Id$

@package docstring
Application of a State Machine for high-level robotic tasks control.
'''

# ROS related modules
import rospy
import smach
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovarianceStamped

# Other libraries
from threading import Lock
from math import sin, cos

# Our modules
import myglobals
from utils import quaternion2yaw
from Navigation import Navigation
import LocalFrameWorldFrameTransformations as ft
from utils import quaternion2yaw


class Move2Pos(smach.State):
    ''' Define state MoveForward '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'discharged',
                                       'aborted'],
                             input_keys=['change_target'],
                             output_keys=['robot_pose'])
        # Navigation control
        self.nav = Navigation(myglobals.robot_name)
        self.curr_target = -1  # No target being followed yet
        self.num_targets = len(myglobals.targets_wpos)

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveForward...')

        # Select the next target to move to
        if userdata.change_target:
            self.curr_target = (self.curr_target + 1) % self.num_targets
        userdata.curr_target = self.curr_target
        # Start navigation to target
        self.nav.start(myglobals.targets_wpos[self.curr_target])

        # Execution loop
        rate = rospy.Rate(myglobals.execution_rate)
        while not rospy.is_shutdown():
            # Consume power
            myglobals.power_status -= 5
            
            # Check the power level
            if myglobals.power_status < myglobals.MIN_POWER_LEVEL:
                self.nav.stop()  # Stop navigation loop
                return 'discharged'
            elif self.nav.goal_reached:
                self.nav.stop()  # Stop navigation loop
                userdata.robot_pose = self.nav.robot_pose
                return 'succeeded'
            rate.sleep()

        return 'aborted'

