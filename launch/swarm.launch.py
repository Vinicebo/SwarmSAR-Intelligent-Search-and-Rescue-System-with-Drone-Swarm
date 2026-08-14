import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simulator.spawn_helpers import render_bridge_yaml, render_drone_sdf

SPAWN_SPACING_M = 3.0
SPAWN_ROW_LENGTH = 6
COMM_RANGE_M = 50.0


def launch_setup(context, *args, **kwargs):
    simulator_share = get_package_share_directory('simulator')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    world_name = LaunchConfiguration('world').perform(context)

    world_path = os.path.join(simulator_share, 'worlds', f'{world_name}.world')
    sdf_template_path = os.path.join(simulator_share, 'models', 'quadrotor', 'model.sdf.template')
    bridge_template_path = os.path.join(simulator_share, 'config', 'ros_gz_bridge.yaml.template')
    world_bridge_template_path = os.path.join(simulator_share, 'config', 'world_bridge.yaml.template')

    drone_ids = [f'drone_{i + 1}' for i in range(num_drones)]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    bridge_config_path = render_bridge_yaml(
        bridge_template_path, drone_ids, world_bridge_template_path, world_name)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen',
    )

    actions = [gz_sim, bridge]

    for i, drone_id in enumerate(drone_ids):
        model_path = render_drone_sdf(sdf_template_path, drone_id)
        x = (i % SPAWN_ROW_LENGTH) * SPAWN_SPACING_M
        y = (i // SPAWN_ROW_LENGTH) * SPAWN_SPACING_M

        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', drone_id,
                '-file', model_path,
                '-x', str(x), '-y', str(y), '-z', '1',
            ],
            output='screen',
        ))

        # 'force' is remapped to the same absolute topic for every drone —
        # see world_bridge.yaml.template for why per-drone force topics
        # can't each get their own bridge to the same gz wrench topic.
        actions.append(Node(
            package='navigation',
            executable='hover_controller',
            namespace=drone_id,
            parameters=[{'use_sim_time': True, 'drone_id': drone_id}],
            remappings=[('force', '/drone_force'), ('force_clear', '/drone_force_clear')],
            output='screen',
        ))

        # 'swarm_state' is remapped to the same absolute topic for every
        # drone, simulating a shared radio broadcast — each node filters out
        # of out-of-range neighbors on its own, there is no central relay.
        actions.append(Node(
            package='communication',
            executable='swarm_state_node',
            namespace=drone_id,
            parameters=[{
                'use_sim_time': True,
                'drone_id': drone_id,
                'comm_range_m': COMM_RANGE_M,
            }],
            remappings=[('swarm_state', '/swarm_state')],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Defaulted below the project's target swarm size (30): a single
        # parameter_bridge process currently saturates somewhere past ~20
        # drones (see docs/simulation_notes.md), causing some drones to
        # never receive their first sensor reading and never take off.
        # Override with num_drones:=N once that bottleneck is addressed.
        DeclareLaunchArgument('num_drones', default_value='20'),
        DeclareLaunchArgument('world', default_value='earthquake_city'),
        OpaqueFunction(function=launch_setup),
    ])
