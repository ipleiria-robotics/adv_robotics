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
            package='ar_utils',
            executable='battery_manager',
            namespace='robot_0',  # TODO: Make this a (launch) parameter
            name='battery_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_battery_manager_cmd)

    # Ground trugh republisher
    start_republisher_cmd = launch_ros.actions.Node(
            package='tw07',
            executable='ground_truth_republisher',
            name='tw07_ground_truth_republisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_republisher_cmd)

    # Include the map server launch
    map_config = os.path.join(
        get_package_share_directory('tw07'), 'config', 'map.yaml')
    included_map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tw07'),
                         'launch', 'map_server_launch.py')),
        launch_arguments={'map_yaml_file': map_config}.items())
    ld.add_action(included_map_server_launch)

    # RViz node
    auto_run_rviz_cmd = DeclareLaunchArgument(
        'run-rviz',
        default_value='False',
        description='Whether to start RViz')
    ld.add_action(auto_run_rviz_cmd)
    rviz_config = os.path.join(get_package_share_directory('tw10'),
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

    # The localization is not started by default. If the user wants,
    # it can pass the "run-particle-filter:=true" argument to start it
    auto_run_ekf_cmd = DeclareLaunchArgument(
        'run-ekf-localization',
        default_value='True',
        description='Whether to run the Particle Filter-based localization')
    ld.add_action(auto_run_ekf_cmd)
    start_ekf_cmd = launch_ros.actions.Node(
            package='tw08',
            executable='ekf_localization',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('run-ekf-localization')))
    ld.add_action(start_ekf_cmd)

    # Action server for Move2Pos
    start_action_server_move2pos_cmd = launch_ros.actions.Node(
            package='tw10',
            executable='action_move2pos',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_action_server_move2pos_cmd)

    # Action server for Rotate2Angle
    start_action_server_rotate2angle_cmd = launch_ros.actions.Node(
            package='tw10',
            executable='action_rotate2angle',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_action_server_rotate2angle_cmd)

    # Action server for Recharge
    start_action_server_recharge_cmd = launch_ros.actions.Node(
            package='tw10',
            executable='action_recharge',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_action_server_recharge_cmd)

    # Action server for Stop
    start_action_server_stop_cmd = launch_ros.actions.Node(
            package='tw10',
            executable='action_stop',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}])
    ld.add_action(start_action_server_stop_cmd)

    # Main task node
    auto_run_simple_task_cmd = DeclareLaunchArgument(
        'run-simple-task',
        default_value='True',
        description='Whether to run the simple task')
    ld.add_action(auto_run_simple_task_cmd)
    start_simple_task_cmd = launch_ros.actions.Node(
            package='tw10',
            executable='simple_task',
            name='tw10_simple_task',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time,
                         'ref_lin_vel': 0.5,
                         'ref_ang_vel': radians(60)}],
            condition=IfCondition(LaunchConfiguration('run-simple-task')))
    ld.add_action(start_simple_task_cmd)

    return ld
