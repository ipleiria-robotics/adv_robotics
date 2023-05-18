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


def add_example(ld, launch_argument, description, package, executable,
                use_sim_time):
    # Tutorial example
    auto_run_example_cmd = DeclareLaunchArgument(
        launch_argument,
        default_value='True',
        description=description)
    ld.add_action(auto_run_example_cmd)
    start_example_task_cmd = launch_ros.actions.Node(
            package=package,
            executable=executable,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration(launch_argument)))
    ld.add_action(start_example_task_cmd)


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
                'map_laser_with_landmarks2_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_stage')))
    ld.add_action(stage_cmd)

    # Start the battery manager
    start_battery_manager_cmd = launch_ros.actions.Node(
            package='ar_cpp_utils',
            executable='battery_manager',
            namespace='robot_0',  # TODO: Make this a (launch) parameter
            name='battery_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_battery_manager_cmd)

    # Add action servers
    addActionServer(ld, 'tw10', 'action_play_sound', use_sim_time)

    # Tutorial example
    add_example(ld, 'run-example-02',
                'Whether to run the BT-based tutorial 02', 'tw11',
                'battery-check', use_sim_time)

    return ld
