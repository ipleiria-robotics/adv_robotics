#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
# All rights reserved
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
Sound action: play a sound file.
'''

# ROS related modules
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Our modules
import lw2.myglobals as myglobals
from ar_utils.action import PlaySound
from ar_utils.utils import play_sound, stop_all_sounds

# Other modules
import os
from threading import Lock

# This action name (strip the '.py' preffix)
ACTION_NAME = os.path.basename(__file__)[:-3]


class PlaySoundActionServer(Node):
    '''
        Play a sound from a file.
    '''
    def __init__(self):
        super().__init__('action_play_sound')

        # Create condition to manage access to the goal variable, wich will be
        # accessed in multiple callbacks
        self.goal_handle = None
        self.goal_lock = Lock()

        # Start the actual action server
        self.action_server = ActionServer(
            self,
            PlaySound,
            f'/{myglobals.robot_name}/{ACTION_NAME}',
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
                # Allow the trigger callback to finish (it might be blocked)
            self.goal_handle = goal_handle
        # Start runing the execute callback
        goal_handle.execute()

    def cancel_cb(self, goal_handle):
        ''' Callback to call when the action is cancelled '''
        self.get_logger().info(f'{ACTION_NAME} was cancelled!')
        # Update internal information
        with self.goal_lock:
            stop_all_sounds()
        # Change the action status to cancelled
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        ''' Callback to call when the action as a new goal '''
        self.get_logger().info(f'Executing action {ACTION_NAME} with ' +
                               f'file {self.goal_handle.request.sound_file}')

        # Play sound
        with self.goal_lock:
            play_succeeded = play_sound(self.goal_handle.request.sound_file,
                                        False)
            if play_succeeded:
                self.goal_handle.succeed()
                self.get_logger().info(f'{ACTION_NAME} has succeeded!')
                return PlaySound.Result(sound_played=True)
            else:
                if not goal_handle.is_active:
                    self.get_logger().info(f'{ACTION_NAME}: goal aborted')
                elif goal_handle.is_cancel_requested:
                    goal_handle.canceled()  # Confirm goal is canceled
                    self.get_logger().info(f'{ACTION_NAME}: goal canceled')
                else:
                    self.get_logger().info(f'{ACTION_NAME} failed!')
                return PlaySound.Result(sound_played=False)


def main(args=None):
    ''' Main function - start the action server.
    '''
    rclpy.init(args=args)
    play_sound_action_server = PlaySoundActionServer()

    # Use 2 threads to make sure callbacks can run in parallel and the action
    # does not block.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(play_sound_action_server)
    executor.spin()


if __name__ == '__main__':
    main()
