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


class SelectNextPose(smach.State):
    ''' Define state MoveForward '''
    def __init__(self, targets_wposes):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['change_target'],
                             output_keys=['target_pose'])
        self.targe_wposes = targets_wposes
        self.num_targets = len(targets_wposes)
        # No target selected yet
        self.curr_target_num = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state SelectNextPose...')

        # Select next target, if asked for it.
        # If nothing is said, we assume we should select a new target.
        if ('change_target' not in userdata) or userdata.change_target:
            self.curr_target_num = \
                (self.curr_target_num + 1) % self.num_targets
        userdata.target_pose = self.targe_wposes[self.curr_target_num]
        return 'succeeded'
