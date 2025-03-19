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


def generate_launch_description():
    lifecycle_nodes = ['map_server']

    # Create the launch description. It will be filled below, and returned in
    # the end
    sl = SimpleLauncher(use_sim_time=True)

    # Declare arguments
    sl.declare_arg('namespace', 'robot_0',
                   description='Top-level robot namespace.')
    namespace = sl.arg('namespace')

    # Map server
    sl.declare_arg('run-map-server', True,
                   description='If True, run the map_server node')
    with sl.group(if_arg='run-map-server'):
        sl.node(package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[
                    {'yaml_filename': sl.find('lw1', 'map.yaml', 'config')}])

        # Start lifecycle node manager
        sl.node(package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])

    # Localization node
    sl.declare_arg('run-localization', True,
                   description='If True, run the localization node')
    with sl.group(if_arg='run-localization'):
        sl.node(package='lw1',
                executable='localization',
                name='lw1_localization',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                arguments=['--ros-args', '--log-level', 'warn'])

    # Planner node
    sl.declare_arg('run-planner', True,
                   description='If True, run the localization node')
    with sl.group(if_arg='run-planner'):
        sl.node(package='lw1',
                executable='planner',
                name='lw1_planner',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                arguments=['--ros-args', '--log-level', 'warn'])

    # Navigation node
    sl.declare_arg('run-navigation', True,
                   description='If True, run the navigation node')
    with sl.group(if_arg='run-navigation'):
        sl.node(package='lw1',
                executable='navigation',
                name='lw1_navigation',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                arguments=['--ros-args', '--log-level', 'warn'])

    return sl.launch_description()
