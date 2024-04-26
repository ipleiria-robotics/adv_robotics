from simple_launch import SimpleLauncher
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lifecycle_nodes = ['map_server', 'amcl', 'planner_server']

    sl = SimpleLauncher(use_sim_time=True)

    # Declare arguments
    sl.declare_arg('namespace', 'robot_0',
                   description='Top-level robot namespace.')
    namespace = sl.arg('namespace')
    sl.declare_arg(
        'params_file', sl.find('tw07', 'params.yaml', 'config'),
        description='(Optional) Complete path to the parameters file')
    # Get the YAML configuration file
    params = sl.arg('params_file')
    # configured_params = params
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            root_key=namespace,
            param_rewrites={
                'base_frame_id': sl.arg(namespace)+'/base_footprint',
                'odom_frame_id': sl.arg(namespace)+'/odom',
                'robot_base_frame': sl.arg(namespace)+'/base_footprint'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Stage simulator
    sl.declare_arg('run-stage', True,
                   description='If True, the Stage simulator is started')
    with sl.group(if_arg='run-stage'):
        sl.node(package='stage_ros',
                executable='stageros',
                name='stageros',
                output='screen',
                arguments=sl.find('worlds', 'map_laser_with_landmarks2.world',
                                  'stage_worlds'))

    # Map server
    sl.node(package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[
                {'yaml_filename': sl.find('tw07', 'map.yaml', 'config')}])

    # RViz
    sl.declare_arg('run-rviz', True,
                   description='If True, RViz is started')
    with sl.group(if_arg='run-rviz'):
        sl.rviz(sl.find('tw07', 'config.rviz', 'config'), warnings=True)

    # Ground truth republisher
    sl.node(package='tw07',
            executable='ground_truth_republisher',
            name='tw07_ground_truth_republisher',
            namespace=namespace,
            output='screen')

    # NAV2-based particle filter
    sl.node(package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace,
            output='screen',
            parameters=[configured_params])

    # NAV2-based path planner
    sl.node(package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            parameters=[configured_params])

    # Path navigation node (from TW03)
    #sl.declare_arg('run-navigation', True,
    #               description='If True, run the navigation node')
    #with sl.group(if_arg='run-navigation'):
    #    sl.node(package='tw03',
    #            executable='navigation',
    #            name='navigation',
    #            namespace=namespace,
    #            output='screen',
    #            emulate_tty=True,
    #            parameters=[configured_params])

    # Start lifecycle node manager
    sl.node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}])

    return sl.launch_description()
