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
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
import os


def generate_launch_description():
    # This could be a parameter
    use_sim_time = True
    #world_file = os.path.join(get_package_share_directory('worlds'),
    #                          'stage-worlds', 'map_laser.world')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('base_scan', 'scan')]

    # Create the launch description. It will be filled below, and returned in
    # the end
    ld = LaunchDescription()

    # World file complete path
    world_file = LaunchConfiguration('world_file')
    declare_world_file_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=os.path.join(get_package_share_directory('worlds'),
                                   'stage-worlds', 'map_laser.world'),
        description='Complete path to the stage world file to open'
        )
    ld.add_action(declare_world_file_cmd)

    # Robot namespace
    namespace = LaunchConfiguration('namespace')
    declare_ns_cmd = DeclareLaunchArgument(
        name='namespace', default_value='robot_0',
        description='Robot namespace'
        )
    ld.add_action(declare_ns_cmd)

    # Use static map --> odom TF
    use_static_map_odom_tf = LaunchConfiguration('use_static_map_odom_tf')
    declare_use_static_map_odom_tf_cmd = DeclareLaunchArgument(
        name='use_static_map_odom_tf', default_value='True',
        description='Wether or not to use the static TF from map to odom'
        )
    ld.add_action(declare_use_static_map_odom_tf_cmd)

    ld.add_action(GroupAction([
        # Change namespace
        PushRosNamespace(namespace=namespace),
        # Simulator node.
        launch_ros.actions.Node(
            package='stage_ros',
            executable='stageros',
            name='stage',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[world_file],
            remappings=remappings),
        # Static map --> odom TF publisher. Only if use_static_map_odom_tf is
        # True.
        launch_ros.actions.Node(
            condition=IfCondition(use_static_map_odom_tf),
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            remappings=remappings)
    ]))

    return ld
