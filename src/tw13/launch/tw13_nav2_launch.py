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
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    #lifecycle_nodes = ['slam_toolbox']
    use_sim_time = True

    sl = SimpleLauncher(use_sim_time=use_sim_time)

    # Declare arguments
    sl.declare_arg('namespace', 'robot_0',
                   description='Top-level robot namespace.')
    namespace = sl.arg('namespace')
    sl.declare_arg(
        'params_file', sl.find('tw13', 'params_nav2.yaml', 'config'),
        description='(Optional) Complete path to the parameters file')

    # Get the YAML configuration file
    org_param_file = sl.arg('params_file')
    # configured_params = params
    rewritten_params_file = ParameterFile(
        RewrittenYaml(
            source_file=org_param_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

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

    # SLAM from NAV2
    sl.node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        remappings={'/map': 'map'},
        parameters=[rewritten_params_file,
                    {'use_lifecycle_manager': False,
                     'use_sim_time': use_sim_time}])

    # RViz
    sl.declare_arg('run-rviz', True,
                   description='If True, RViz is started')
    with sl.group(if_arg='run-rviz'):
        sl.rviz(sl.find('tw13', 'config.rviz', 'config'), warnings=True)

    # Ground truth republisher
    sl.node(package='ar_py_utils',
            executable='ground_truth_republisher',
            name='tw07_ground_truth_republisher',
            namespace=namespace,
            output='screen',
            parameters={'use_sim_time': use_sim_time})

    # Path navigation node (from TW03)
    sl.declare_arg('run-navigation', True,
                   description='Whether to run the navigation (from TW03)')
    with sl.group(if_arg='run-navigation'):
        sl.node(package='tw03',
                executable='navigation',
                name='tw03_navigation',
                namespace=namespace,
                output='screen',
                parameters=[rewritten_params_file,
                            {'use_sim_time': use_sim_time}])

    ## Start lifecycle node manager
    #sl.node(package='nav2_lifecycle_manager',
    #        executable='lifecycle_manager',
    #        name='lifecycle_manager',
    #        namespace=namespace,
    #        output='screen',
    #        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #        parameters={'autostart': True,
    #                    'node_names': lifecycle_nodes,
    #                    'use_sim_time': use_sim_time})

    return sl.launch_description()
