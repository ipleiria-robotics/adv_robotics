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
import smach_ros

# Our modules
import LocalFrameWorldFrameTransformations as ft

# Our actions
from ActionSelectNextPose import SelectNextPose
from ActionMove2Pos import Move2Pos
from ActionRecharge import Recharge
from ActionRotate2Angle import Rotate2Angle

# Other modules
from math import pi

# main
if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create the SMACH state machine
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted'])

    # Open the state machine container
    with sm:
        ''' Create a new state machine for moving to given poses '''
        sm_move2pose = smach.StateMachine(outcomes=['succeeded', 'discharged',
                                                    'aborted'],
                                          input_keys=['change_target'],
                                          output_keys=['change_target'])
        with sm_move2pose:
            ''' Add states to sm_move2pose '''
            # Targets pose to be followed.
            # Each line represents a target pose in world coordinates, for
            # [X, Y, Theta], with X and Y in [m] and Theta in [rad].
            # Targets positions to be followed (in world coordinates [m])
            targets_wpose = [ft.Pose2D(-2.5, -1.5, 0.0),  # 1
                             ft.Pose2D(2.5, -1.5, pi)]  # 2
            smach.StateMachine.add('SelectNextPose',
                                   SelectNextPose(targets_wpose),
                                   transitions={'succeeded': 'Move2Pos',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('Move2Pos', Move2Pos(),
                                   transitions={'succeeded': 'Rotate2Angle',
                                                'discharged': 'discharged',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('Rotate2Angle', Rotate2Angle(),
                                   transitions={'succeeded': 'succeeded',
                                                'discharged': 'discharged',
                                                'aborted': 'aborted'})

        ''' Create a new state machine for recharging '''
        sm_recharge = smach.StateMachine(outcomes=['succeeded', 'aborted'],
                                         output_keys=['change_target'])
        # Recharge target pose.
        recharge_targets_wpose = [ft.Pose2D(0.0, -2.0, -pi/2)]
        with sm_recharge:
            # Add states to sm_recharge
            smach.StateMachine.add(
                'SelectRechargePose', SelectNextPose(recharge_targets_wpose),
                transitions={'succeeded': 'Move2RechargePos',
                             'aborted': 'aborted'})
            smach.StateMachine.add(
                'Move2RechargePos', Move2Pos(True),
                transitions={'succeeded': 'Rotate2RechargeAngle',
                             'aborted': 'aborted'})
            smach.StateMachine.add('Rotate2RechargeAngle', Rotate2Angle(True),
                                   transitions={'succeeded': 'Recharge',
                                                'aborted': 'aborted'})
            smach.StateMachine.add('Recharge', Recharge(),
                                   transitions={'succeeded': 'succeeded',
                                                'aborted': 'aborted'})

        # Add states to sm, using the state machines created above.
        # We are using hierarchical state machines to make things more clean.
        smach.StateMachine.add('MOVE2POSE', sm_move2pose,
                               transitions={'discharged': 'RECHARGE',
                                            'succeeded': 'MOVE2POSE',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('RECHARGE', sm_recharge,
                               transitions={'succeeded': 'MOVE2POSE',
                                            'aborted': 'aborted'})

    # Force first Move2Pos to go to target 0
    sm.userdata.change_target = False

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server',
                                        sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
