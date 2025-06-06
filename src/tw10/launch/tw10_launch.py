#!/usr/bin/env python3

"""Launch file for tw10 robotic task implementation.

This module sets up the launch description for the tw10 package, including
simulation, localization, planning, navigation, and visualization nodes.
"""

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
# POSSIBILITY OF SUCH DAMAGE

from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from simple_launch import SimpleLauncher


def addActionServer(sl, package, executable, namespace, params_file,
                    use_sim_time):
    """Add an action server node to the launch description.

    Args:
        sl: The SimpleLauncher instance.
        package: The ROS package containing the action server.
        executable: The executable name of the action server.
        namespace: The namespace for the node.
        params_file: The parameters file to use.
        use_sim_time: Boolean indicating whether to use simulation time.
    """
    # Action server for given action
    sl.node(package=package,
            executable=executable,
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}])


def generate_launch_description():
    """Generate the launch description."""
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
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=org_param_file,
            root_key=namespace,
            param_rewrites={
                # 'odom_topic': '/' + sl.arg(namespace)+'/base_footprint',
                # 'local_costmap.local_costmap.ros__parameters.global_frame': sl.arg(namespace)+'/odom',
                # 'robot_base_frame': sl.arg(namespace)+'/base_footprint',
                # 'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': '/' + sl.arg(namespace)+'/base_scan'
                },
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

    # Map server (use map from TW07)
    sl.node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters={'yaml_filename': sl.find('tw07', 'map.yaml', 'config'),
                    'use_sim_time': use_sim_time})

    # RViz
    sl.declare_arg('run-rviz', True,
                   description='If True, RViz is started')
    with sl.group(if_arg='run-rviz'):
        sl.rviz(sl.find('tw10', 'config.rviz', 'config'), warnings=True)

    # Ground truth republisher
    sl.node(package='ar_py_utils',
            executable='ground_truth_republisher',
            name='tw07_ground_truth_republisher',
            namespace=namespace,
            output='screen',
            parameters={'use_sim_time': use_sim_time})

    # EKF localization node started by default
    sl.declare_arg(
        'run-ekf-localization', True,
        description='If True, run the EKF-based localization node from TW08.')
    with sl.group(if_arg='run-ekf-localization'):
        sl.node(package='tw08',
                executable='ekf_localization',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time}])

    # Start the battery manager
    sl.node(package='ar_cpp_utils',
            executable='battery_manager',
            namespace=namespace,
            name='battery_manager',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params,
                        {'use_sim_time': use_sim_time}])

    # Add action servers
    addActionServer(sl, 'tw10', 'action_move2pos',
                    namespace, configured_params, use_sim_time)
    addActionServer(sl, 'tw10', 'action_speak_text',
                    namespace, configured_params, use_sim_time)
    addActionServer(sl, 'tw10', 'action_rotate2angle',
                    namespace, configured_params, use_sim_time)
    addActionServer(sl, 'tw10', 'action_recharge',
                    namespace, configured_params, use_sim_time)
    addActionServer(sl, 'tw10', 'action_stop',
                    namespace, configured_params, use_sim_time)

    # Main task node argument
    sl.declare_arg('run-simple-task', False,
                   description='If True, run the main task node.')

    # Nav2 planner and controller
    sl.declare_arg('run-nav2-planner-and-controller', False,
                   description='If True, run the nav2 path planner node.')
    with sl.group(if_arg='run-nav2-planner-and-controller'):
        full_lifecycle_nodes = lifecycle_nodes.copy()
        # Planner
        full_lifecycle_nodes.append('planner_server')
        sl.node(package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time}])

        # Controller
        full_lifecycle_nodes.append('controller_server')
        sl.node(package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time}])

        # Main task node (with nav2)
        with sl.group(if_arg='run-simple-task'):
            sl.node(package='tw10',
                    executable='simple_task_nav2',
                    namespace=namespace,
                    name='tw10_simple_task',
                    output='screen',
                    emulate_tty=True,
                    parameters=[configured_params,
                                {'use_sim_time': use_sim_time}])

        # Start lifecycle node manager
        sl.node(package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters={'autostart': True,
                            'node_names': full_lifecycle_nodes,
                            'use_sim_time': use_sim_time})

    # Nodes that run only when not using nav2
    with sl.group(unless_arg='run-nav2-planner-and-controller'):
        # Non-nav2 main simple task
        with sl.group(if_arg='run-simple-task'):
            sl.node(package='tw10',
                    executable='simple_task',
                    namespace=namespace,
                    name='tw10_simple_task',
                    output='screen',
                    emulate_tty=True,
                    parameters=[configured_params,
                                {'use_sim_time': use_sim_time}])

        # If we are not running the nav2 planner and controller, use the
        # lifecycle node manager just for the map_server
        sl.node(package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                namespace=namespace,
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters={'autostart': True,
                            'node_names': lifecycle_nodes,
                            'use_sim_time': use_sim_time})

    return sl.launch_description()
