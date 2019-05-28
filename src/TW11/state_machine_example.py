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
import myglobals

# Our actions
from ActionMove2Pos import Move2Pos
from ActionRecharge import Recharge
from ActionRotate180 import Rotate180

# main
if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create the SMACH state machine
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted'])
    # Force first Move2Pos to go to a new target
    sm.userdata.change_target = True

    # Open the sub container
    with sm:
        # Add states to the sub container
        smach.StateMachine.add('Move2Pos', Move2Pos(),
                               transitions={'succeeded': 'Rotate180',
                                            'discharged': 'RechargeM',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('Rotate180', Rotate180(),
                               transitions={'succeeded': 'MoveForward',
                                            'discharged': 'RechargeR',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('RechargeM', Recharge(),
                               transitions={'succeeded': 'MoveForward',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('RechargeR', Recharge(),
                               transitions={'succeeded': 'Rotate180',
                                            'aborted': 'aborted'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server',
                                        sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
