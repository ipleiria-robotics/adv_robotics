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


def addActionServer(sl, package, executable, namespace, params_file,
                    use_sim_time):
    # Action server for given action
    sl.node(package=package,
            executable=executable,
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}])


def generate_launch_description():
    lifecycle_nodes = ['map_server']
    use_sim_time = True

    sl = SimpleLauncher(use_sim_time=use_sim_time)

    # Declare arguments
    sl.declare_arg('namespace', 'robot_0',
                   description='Top-level robot namespace.')
    namespace = sl.arg('namespace')
    sl.declare_arg(
        'params_file', sl.find('tw10', 'params.yaml', 'config'),
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
                arguments=sl.find('worlds', 'lw2.world', 'stage_worlds'),
                parameters={'use_sim_time': use_sim_time})

    # Start the simulation control
    sl.node(
        package='ar_cpp_utils',
        executable='sim_control',
        name='sim_control',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters={'use_sim_time': use_sim_time})

    # Include the map server launch
    # The map server is needed only while the ICP is not changed, so it it does
    # not run by default.
    sl.declare_arg('run-map-server', False,
                   description='If True, RViz is started')
    with sl.group(if_arg='run-map-server'):
        sl.node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters={'yaml_filename': sl.find('tw04', 'map.yaml', 'config'),
                        'use_sim_time': use_sim_time})

        # If we are using the map server, run the lifecycle node manager
        sl.node(package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters={'autostart': True,
                            'node_names': lifecycle_nodes,
                            'use_sim_time': use_sim_time})

    # RViz
    sl.declare_arg('run-rviz', True,
                   description='If True, RViz is started')
    with sl.group(if_arg='run-rviz'):
        sl.rviz(sl.find('lw2', 'config.rviz', 'config'), warnings=True)

    # The localization is started by default. If the user wants, it can pass
    # the "run-ekf-localization:=False" argument to avoid starting it
    sl.declare_arg(
        'run-ekf-localization', True,
        description='If True, run the EKF-based localization node.')
    with sl.group(if_arg='run-ekf-localization'):
        sl.node(package='lw2',
                executable='ekf_icp_localization',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[rewritten_params_file,
                            {'use_sim_time': use_sim_time}])

    # Add action servers
    addActionServer(sl, 'lw2', 'action_play_sound',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_move2pos',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_move_forklift',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_move_visual_servoing',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_rotate2angle',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_recharge',
                    namespace, rewritten_params_file, use_sim_time)
    addActionServer(sl, 'lw2', 'action_stop',
                    namespace, rewritten_params_file, use_sim_time)

    # Path navigation node
    sl.declare_arg(
        'run-navigation', True,
        description='If True, run the path navigation node')
    with sl.group(if_arg='run-navigation'):
        sl.node(package='lw2',
                executable='path_navigation',
                name='lw2_path_navigation',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[rewritten_params_file,
                            {'use_sim_time': use_sim_time}])

    # Task node
    sl.declare_arg(
        'run-task', True,
        description='If True, run the main task.')
    with sl.group(if_arg='run-navigation'):
        sl.node(package='lw2',
                executable='task',
                name='lw2_task',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[rewritten_params_file,
                            {'use_sim_time': use_sim_time}])

    return sl.launch_description()
