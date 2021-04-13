#!/usr/bin/env python3

# Copyright (c) 2021, Hugo Costelha
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Parameters
    use_sim_time = True

    # Create the launch description. It will be filled below, and returned in
    # the end
    ld = LaunchDescription()

    # Start the stage simulator
    start_stage_cmd = DeclareLaunchArgument(
        'start_stage',
        default_value='True',
        description='Whether to start the stage simulator')
    ld.add_action(start_stage_cmd)
    stage_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('worlds'), 'launch',
                'worlds lw1_simulator_launch. py')),
        condition=IfCondition(LaunchConfiguration('start_stage')))
    ld.add_action(stage_cmd)

    # Include the map server launch (similar to the one used in the tw04)
    included_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lw1'),
                         'launch', 'map_server_launch.py')))
    ld.add_action(included_map_server_launch)

    # RViz node
    rviz_config = os.path.join(get_package_share_directory('lw1'),
                               'config', 'config.rviz')
    start_localization_cmd = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config])
    ld.add_action(start_localization_cmd)

    # Localization node
    start_localization_cmd = launch_ros.actions.Node(
            package='lw1',
            executable='localization',
            name='lw1_localization',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'warn'])
    ld.add_action(start_localization_cmd)

    # Planner node
    start_planner_cmd = launch_ros.actions.Node(
            package='lw1',
            executable='planner',
            name='lw1_planner',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_planner_cmd)

    # Navigation node
    start_navigation_cmd = launch_ros.actions.Node(
            package='lw1',
            executable='navigation',
            name='lw1_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_navigation_cmd)

    return ld
