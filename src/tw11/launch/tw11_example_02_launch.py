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
# POSSIBILITY OF SUCH DAMAGE

from simple_launch import SimpleLauncher


def addActionServer(sl, package, executable, namespace, use_sim_time):
    # Action server for given action
    sl.node(package=package,
            executable=executable,
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])


def generate_launch_description():
    # Parameters
    use_sim_time = True

    sl = SimpleLauncher(use_sim_time=use_sim_time)

    # Declare arguments
    sl.declare_arg('namespace', 'robot_0',
                   description='Top-level robot namespace.')
    namespace = sl.arg('namespace')

    # Stage simulator
    sl.declare_arg('run-stage', True,
                   description='If True, the Stage simulator is started')
    with sl.group(if_arg='run-stage'):
        sl.node(package='stage_ros',
                executable='stageros',
                name='stageros',
                output='screen',
                arguments=sl.find('worlds', 'map_laser_with_landmarks2.world',
                                  'stage_worlds'),
                parameters={'use_sim_time': use_sim_time})

    # Start the battery manager
    sl.node(
        package='ar_cpp_utils',
        executable='battery_manager',
        name='battery_manager',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters={'use_sim_time': use_sim_time})

    # Add action servers
    addActionServer(sl, 'tw10', 'action_speak_text', namespace, use_sim_time)

    # Tutorial example
    sl.declare_arg('run-example-02', True,
                   description='If True, run the 2nd BT')
    with sl.group(if_arg='run-example-02'):
        sl.node(package='tw11',
                executable='battery-check',
                namespace=namespace,
                name='battery_check',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time}])

    return sl.launch_description()
