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

    # Simulator node.
    world_file = os.path.join(get_package_share_directory('worlds'),
                              'stage_worlds',
                              'map_laser_with_landmarks2.world')
    start_simulator_cmd = launch_ros.actions.Node(
            package='stage_ros',
            executable='stageros',
            name='stage',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[world_file])
    ld.add_action(start_simulator_cmd)

    # Static transform publisher between /map and /robot_0/odom.
    #start_static_tf_pub_cmd = launch_ros.actions.Node(
    #        package='tf2_ros',
    #        executable='static_transform_publisher',
    #        output='screen',
    #        emulate_tty=True,
    #        parameters=[{'use_sim_time': use_sim_time}],
    #        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_0/odom'])
    #ld.add_action(start_static_tf_pub_cmd)

    return ld
