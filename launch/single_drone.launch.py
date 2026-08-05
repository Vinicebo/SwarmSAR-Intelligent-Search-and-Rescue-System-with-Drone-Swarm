import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from simulator.spawn_helpers import render_bridge_yaml, render_drone_sdf

DRONE_ID = 'drone_1'
WORLD_NAME = 'earthquake_city'


def generate_launch_description():
    simulator_share = get_package_share_directory('simulator')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(simulator_share, 'worlds', f'{WORLD_NAME}.world')
    sdf_template_path = os.path.join(simulator_share, 'models', 'quadrotor', 'model.sdf.template')
    bridge_template_path = os.path.join(simulator_share, 'config', 'ros_gz_bridge.yaml.template')
    world_bridge_template_path = os.path.join(simulator_share, 'config', 'world_bridge.yaml.template')

    model_path = render_drone_sdf(sdf_template_path, DRONE_ID)
    bridge_config_path = render_bridge_yaml(
        bridge_template_path, [DRONE_ID], world_bridge_template_path, WORLD_NAME)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', DRONE_ID,
            '-file', model_path,
            '-x', '0', '-y', '0', '-z', '1',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        output='screen',
    )

    hover_controller = Node(
        package='navigation',
        executable='hover_controller',
        namespace=DRONE_ID,
        parameters=[{'use_sim_time': True, 'drone_id': DRONE_ID}],
        remappings=[('force', '/drone_force'), ('force_clear', '/drone_force_clear')],
        output='screen',
    )

    swarm_state_node = Node(
        package='communication',
        executable='swarm_state_node',
        namespace=DRONE_ID,
        parameters=[{'use_sim_time': True, 'drone_id': DRONE_ID}],
        remappings=[('swarm_state', '/swarm_state')],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_drone,
        bridge,
        hover_controller,
        swarm_state_node,
    ])
