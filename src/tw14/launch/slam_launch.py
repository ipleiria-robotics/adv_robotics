# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
                           GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Variables
    lifecycle_nodes = ['map_saver']

    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('tw14')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        }

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/map', 'map'),
                  ('/scan', 'scan'),
                  ('/map_metadata', 'map_metadata'),
                  ('/slam_toolbox', 'slam_toolbox'),
                 ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace ')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Nodes launching commands
    start_sync_slam_toolbox_node = Node(
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        remappings=remappings)

    #start_slam_toolbox_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(slam_launch_file),
    #    launch_arguments={'use_sim_time': use_sim_time}.items(),
    #    remappings=remappings)

    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            parameters=[configured_params])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(GroupAction([
#        PushRosNamespace(
#            condition=IfCondition(use_namespace),
#            namespace=namespace),
        # Running SLAM Toolbox
        start_sync_slam_toolbox_node,
        # Running Map Saver Server
        start_map_saver_server_cmd,
        start_lifecycle_manager_cmd
    ]))

    return ld
