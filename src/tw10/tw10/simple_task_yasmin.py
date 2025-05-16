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


'''@package docstring
Execute a simple task, formed by the following sequential execution:
  - Action Move2Pos to position (0, -2) [m]
  - Action Rotate2Angle to orientation -90 deg
  - Action Stop
  - Action Recharge to 90%
  Use yasmin to implement the FSM: https://github.com/uleroboticsgroup/yasmin
'''

# ROS API
from geometry_msgs.msg import Point
import rclpy

# Yasmin FSM execution
import yasmin
from yasmin import Blackboard
from yasmin import State
from yasmin import StateMachine
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_viewer import YasminViewerPub

# Our libraries and functions
from ar_utils import action
import tw10.myglobals as myglobals


class SuccessState(State):
    """Represents the success state."""

    def __init__(self) -> None:
        # List of possible outcomes
        super().__init__([SUCCEED])

    def execute(self, blackboard: Blackboard) -> str:
        # We are simply returning a fixed outcome, but the idea is to run
        # the code corresponding to this state and, depending how thing run,
        # return the corresponding outcome
        return SUCCEED


class FailureState(State):
    """Represents the failure state."""

    def __init__(self) -> None:
        # List of possible outcomes
        super().__init__(['failure'])

    def execute(self, blackboard: Blackboard) -> str:
        # We are simply returning a fixed outcome, but the idea is to run
        # the code corresponding to this state and, depending how thing run,
        # return the corresponding outcome
        return 'failure'


def main(args=None):
    """
    @brief  Main function for the simple task example using YASMIN.

    Main function - runs a simple sequential task with few actions, implemented
    as a FSM using YASMIN.
    """
    rclpy.init(args=args)

    # Set ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["success", "failure"])

    # Create the blackboard
    blackboard = Blackboard()
    # Initialize the message to be spoken as an empty string
    blackboard['message2speak'] = ''

    #
    # Add each state to the FSM
    #

    #############################################################
    # Motion state
    def create_goal_cb_motion(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Move2Pos.Goal(target_position=Point(
            x=myglobals.recharge_targets_wpose.x,
            y=myglobals.recharge_targets_wpose.y,
            z=0.0))
        return goal

    def response_handler_cb_motion(blackboard: Blackboard,
                                   response: action.Move2Pos.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Move2Pos finished at: ' +
                               response.final_pose.__str__())
        return SUCCEED
    sm.add_state(
        'MOTION',
        ActionState(
            action_type=action.Move2Pos,
            action_name='ActionMove2Pos',
            create_goal_handler=create_goal_cb_motion,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=response_handler_cb_motion
        ),
        transitions={
            SUCCEED: 'ROTATION',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Rotation state
    def create_goal_cb_rotation(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Rotate2Angle.Goal(
            target_orientation=myglobals.recharge_targets_wpose.theta)
        return goal

    def feedback_handler_cb_rotation(blackboard: Blackboard,
                                     response: action.Rotate2Angle.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Rotate2Angle finished at: ' +
                               response.final_orientation.__str__())
        return SUCCEED
    sm.add_state(
        'ROTATION',
        ActionState(
            action_type=action.Rotate2Angle,
            action_name='ActionRotate2Angle',
            create_goal_handler=create_goal_cb_rotation,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=feedback_handler_cb_rotation
        ),
        transitions={
            SUCCEED: 'STOP',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Stop state
    def create_goal_cb_stop(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Stop.Goal()
        return goal

    def feedback_handler_cb_stop(blackboard: Blackboard,
                                 response: action.Stop.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Stop finished with robot stopped: ' +
                               str(response.is_stopped))
        return SUCCEED
    sm.add_state(
        'STOP',
        ActionState(
            action_type=action.Stop,
            action_name='ActionStop',
            create_goal_handler=create_goal_cb_stop,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=feedback_handler_cb_stop
        ),
        transitions={
            SUCCEED: 'RECHARGING',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Recharging state
    def create_goal_cb_recharging(blackboard: Blackboard):
        # Create the desired goal
        goal = action.Recharge.Goal(target_battery_level=1.0)
        return goal

    def feedback_handler_cb_recharging(blackboard: Blackboard,
                                       response: action.Recharge.Result):
        # Store the message with the battery level in the blackboard
        blackboard['message2speak'] = \
            f'Battery recharged up to {response.battery_level*100:.0f}%'
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action Recharging finished with battery at: ' +
                               f'{response.battery_level*100:.0f}%')
        return SUCCEED
    sm.add_state(
        'RECHARGING',
        ActionState(
            action_type=action.Recharge,
            action_name='ActionRecharge',
            create_goal_handler=create_goal_cb_recharging,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=feedback_handler_cb_recharging
        ),
        transitions={
            SUCCEED: 'SPEAK',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Speak state
    def create_goal_cb_speak(blackboard: Blackboard):
        # Create the desired goal
        goal = action.SpeakText.Goal(text_to_speak=blackboard['message2speak'])
        return goal

    def feedback_handler_cb_speak(blackboard: Blackboard,
                                  response: action.SpeakText.Result):
        # Print the result
        yasmin.YASMIN_LOG_INFO('Action SpeakText succeeded: ' +
                               str(response.text_spoken))
        return SUCCEED
    sm.add_state(
        'SPEAK',
        ActionState(
            action_type=action.SpeakText,
            action_name='ActionSpeakText',
            create_goal_handler=create_goal_cb_speak,
            outcomes=[ABORT, CANCEL, SUCCEED],
            result_handler=feedback_handler_cb_speak
        ),
        transitions={
            SUCCEED: 'SUCCESS',
            ABORT: 'FAILURE',
            CANCEL: 'FAILURE',
        },
    )

    #############################################################
    # Success state
    sm.add_state(
        'SUCCESS',
        SuccessState(),
        transitions={
            SUCCEED: 'success'
        },
    )

    #############################################################
    # Failure state
    sm.add_state(
        'FAILURE',
        FailureState(),
        transitions={
            'failure': 'failure'
        },
    )

    # Publish FSM information for visualization
    YasminViewerPub('TW10_YASMIN', sm)

    #############################################################
    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()

    print('Done...')


if __name__ == '__main__':
    main()
