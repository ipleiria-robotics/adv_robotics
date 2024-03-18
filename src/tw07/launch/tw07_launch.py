from simple_launch import SimpleLauncher
import yaml
import os

lifecycle_nodes = ['map_server']

sl = SimpleLauncher(use_sim_time=True)

# Declare arguments
sl.declare_arg('params_file',
               sl.find('tw07', 'params.yaml', 'config'),
               description='(Optional) Complete path to the parameters file')
sl.declare_arg('start_stage', True,
               description='If True, the Stage simulator is started')


def launch_opaque_setup():

    with sl.group(if_arg='start_stage'):
        # Get the YAML configuration file
        params = sl.arg('params_file')
        with open(params, 'r') as file:
            params_raw = yaml.safe_load(file)

        # Get the world file from the parameters file
        world_pkg = params_raw['stage']['world_package']
        world_path = params_raw['stage']['world_file']
        world_dir, world_filename = os.path.split(world_path)

        # Stage simulator
        sl.node('stage_ros', 'stageros',
                arguments=sl.find(world_pkg, world_filename, world_dir))

        # Get the map file from the parameters file
        map_pkg = params_raw['map_server']['map_package']
        map_path = params_raw['map_server']['map_filename']
        map_dir, map_filename = os.path.split(map_path)

        # Map server
        sl.node('nav2_map_server',
                'map_server',
                name='map_server',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[
                    {'yaml_filename': sl.find(map_pkg, map_filename, map_dir)},
                    params_raw['map_server']['ros__parameters']])
        
        # Get the rviz confiuration file
        rviz_cfg_pkg = params_raw['rviz']['cfg_package']
        rviz_cfg_path = params_raw['rviz']['cfg_filename']
        rviz_cfg_dir, rviz_cfg_filename = os.path.split(rviz_cfg_path)

        # Rviz server
        sl.node('rviz2',
                'rviz2',
                name='rviz',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[
                        {'use_sim_time': use_sim_time}],
                arguments=['-d', sl.find(rviz_cfg_pkg, rviz_cfg, rviz_cfg_dir)])
    return sl.launch_description()


generate_launch_description = \
    sl.launch_description(opaque_function=launch_opaque_setup)
