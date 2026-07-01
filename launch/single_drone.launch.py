import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulator_share = get_package_share_directory('simulator')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(simulator_share, 'worlds', 'earthquake_city.world')
    model_path = os.path.join(simulator_share, 'models', 'quadrotor', 'model.sdf')
    bridge_config_path = os.path.join(simulator_share, 'config', 'ros_gz_bridge.yaml')

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
            '-name', 'drone_1',
            '-file', model_path,
            '-x', '0', '-y', '0', '-z', '0.5',
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
        namespace='drone_1',
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_drone,
        bridge,
        hover_controller,
    ])
