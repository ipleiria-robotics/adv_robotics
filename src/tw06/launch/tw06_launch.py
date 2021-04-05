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

    # Include the map server launch used in the tw04
    included_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tw04'),
                         'launch', 'map_server_launch.py')))
    ld.add_action(included_map_server_launch)

    # RViz node
    rviz_config = os.path.join(get_package_share_directory('tw06'),
                               'config', 'config.rviz')
    start_rviz_cmd = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config])
    ld.add_action(start_rviz_cmd)

    # Localization node (from TW04)
    start_localization_cmd = launch_ros.actions.Node(
            package='tw04',
            executable='localization',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'warn'])
    ld.add_action(start_localization_cmd)

    # The potential-based planner is not started by default. If the user wants,
    # it can pass the "start_planner:=true" argument to start it
    auto_start_planner_cmd = DeclareLaunchArgument(
        'start_planner',
        default_value='false',
        description='Whether to auto-start the planner')
    ld.add_action(auto_start_planner_cmd)
    start_planner_cmd = launch_ros.actions.Node(
            package='tw06',
            executable='search_and_planning',
            name='tw06_search_and_planning',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('start_planner')))
    ld.add_action(start_planner_cmd)

    # Path navigation node (from TW05)
    start_navigation_cmd = launch_ros.actions.Node(
            package='tw05',
            executable='path_navigation',
            name='tw05_path_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_navigation_cmd)

    return ld
