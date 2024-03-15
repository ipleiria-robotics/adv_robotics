from simple_launch import SimpleLauncher
import yaml
import os


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    # Declare arguments
    sl.declare_arg('params_file',
                   sl.find('tw07', 'params.yaml', 'config'),
                   description='(Optional) Complete path to the parameters file')

    # Get the YAML configuration file
    params = sl.arg('params_file')
    #params = sl.find('tw07', 'params.yaml', 'config')

    # TODO: Find a better way to do this
    # We want to pass the world file as an argument to stage, having received
    # it in the YAML file
    with open(params, 'r') as file:
        params_raw = yaml.safe_load(file)
    world_pkg = params_raw['stage']['world_package']
    world_path = params_raw['stage']['world_file']
    world_dir, world_filename = os.path.split(world_path)

    # Stage simulator
    sl.node('stage_ros', 'stageros',
            arguments=[sl.find(world_pkg, world_filename, world_dir)])




    return sl.launch_description()