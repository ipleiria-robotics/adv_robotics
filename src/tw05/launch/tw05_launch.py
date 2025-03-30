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

    # Stage simulator
    sl.declare_arg('run-stage', True,
                   description='If True, the Stage simulator is started')
    with sl.group(if_arg='run-stage'):
        sl.node(package='stage_ros',
                executable='stageros',
                name='stageros',
                output='screen',
                arguments=sl.find('worlds',
                                  'factory_laser_with_landmarks.world',
                                  'stage_worlds'))
        # Static tf publisher, until we learn more in TF2
        sl.node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='screen',
                arguments='0 0 0 0 0 0 map robot_0/odom')

    # Map server (from tw04)
    sl.declare_arg('map_config', sl.find('tw04', 'map.yaml', 'config'),
                   description='(Optional) Path to the map config file')
    map_config_file = sl.arg('map_config')  # Get the YAML configuration file
    sl.node(package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[
                {'yaml_filename': map_config_file}])

    # RViz
    sl.declare_arg('run-rviz', True,
                   description='Set to True to start RViz.')
    with sl.group(if_arg='run-rviz'):
        sl.rviz(sl.find('tw05', 'config.rviz', 'config'),
                warnings=True)

    # Localization node (from TW04)
    sl.node(package='tw04',
            executable='localization',
            name='tw04_localization',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            arguments=['--ros-args', '--log-level', 'warn'])

    # Potential fields-based planner
    # The potential-based planner is not started by default. If the user wants,
    # it can pass the "start-planner:=true" argument to start it
    sl.declare_arg('start-planner', False,
                   description='Set to True to auto-start the planner.')
    with sl.group(if_arg='start-planner'):
        sl.node(package='tw05',
                executable='potential_fields_planner',
                name='tw05_potential_fields_planner',
                namespace=namespace,
                output='screen',
                emulate_tty=True)  # https://github.com/ros2/launch/issues/188

    # Path navigation node (from TW04)
    sl.node(package='tw04',
            executable='path_navigation',
            name='tw04_path_navigation',
            namespace=namespace,
            output='screen',
            emulate_tty=True)  # https://github.com/ros2/launch/issues/188

    # Start lifecycle node manager
    sl.node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return sl.launch_description()
