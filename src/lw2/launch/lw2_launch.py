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
from math import radians


def addActionServer(ld, package, executable, use_sim_time):
    # Action server for Rotate2Angle
    start_action_server_cmd = launch_ros.actions.Node(
            package=package,
            executable=executable,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_action_server_cmd)


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
                'lw2_simulator_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_stage')))
    ld.add_action(stage_cmd)

    # Start the simulation control
    start_sim_control_cmd = launch_ros.actions.Node(
            package='ar_utils',
            executable='sim_control',
            namespace='robot_0',  # TODO: Make this a (launch) parameter
            name='sim_control',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_sim_control_cmd)

    # Include the map server launch
    # The map server is needed only while the ICP is not changed, so it it does
    # not run by default.
    auto_run_map_server_cmd = DeclareLaunchArgument(
        'run-map-server',
        default_value='False',
        description='Whether to run the map server (it should not be needed)')
    ld.add_action(auto_run_map_server_cmd)
    map_config = os.path.join(
        get_package_share_directory('tw04'), 'config', 'map.yaml')
    included_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tw07'),
                         'launch', 'map_server_launch.py')),
        launch_arguments={'map_yaml_file': map_config}.items(),
        condition=IfCondition(
                LaunchConfiguration('run-map-server')))
    ld.add_action(included_map_server_launch)

    # The localization is started by default. If the user wants, it can pass
    # the "run-ekf-localization:=False" argument to avoid starting it
    auto_run_ekf_cmd = DeclareLaunchArgument(
        'run-ekf-icp-localization',
        default_value='True',
        description='Whether to run the ICP-based EKF localization')
    ld.add_action(auto_run_ekf_cmd)
    start_ekf_cmd = launch_ros.actions.Node(
            package='lw2',
            executable='ekf_icp_localization',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(
                LaunchConfiguration('run-ekf-icp-localization')))
    ld.add_action(start_ekf_cmd)

    # RViz node
    auto_run_rviz_cmd = DeclareLaunchArgument(
        'run-rviz',
        default_value='False',
        description='Whether to start RViz')
    ld.add_action(auto_run_rviz_cmd)
    rviz_config = os.path.join(get_package_share_directory('lw2'),
                               'config', 'config.rviz')
    start_rviz_cmd = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('run-rviz')))
    ld.add_action(start_rviz_cmd)

    # Add action servers
    addActionServer(ld, 'lw2', 'action_play_sound', use_sim_time)
    addActionServer(ld, 'lw2', 'action_move2pos', use_sim_time)
    addActionServer(ld, 'lw2', 'action_rotate2angle', use_sim_time)
    addActionServer(ld, 'lw2', 'action_recharge', use_sim_time)

    # Path navigation node
    auto_run_navigation_cmd = DeclareLaunchArgument(
        'run-navigation',
        default_value='False',
        description='Whether to run the navigation')
    ld.add_action(auto_run_navigation_cmd)
    start_navigation_cmd = launch_ros.actions.Node(
            package='lw2',
            executable='path_navigation',
            name='lw2_path_navigation',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time,
                         'ref_lin_vel': 0.5,
                         'ref_ang_vel': radians(60)}],
            condition=IfCondition(LaunchConfiguration('run-navigation')))
    ld.add_action(start_navigation_cmd)

    # Task node
    auto_run_task_cmd = DeclareLaunchArgument(
        'run-task',
        default_value='True',
        description='Whether to run the main task')
    ld.add_action(auto_run_task_cmd)
    start_task_cmd = launch_ros.actions.Node(
            package='lw2',
            executable='task',
            name='lw2_task',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time,
                         'ref_lin_vel': 0.5,
                         'ref_ang_vel': radians(60)}],
            condition=IfCondition(LaunchConfiguration('run-task')))
    ld.add_action(start_task_cmd)

    return ld
