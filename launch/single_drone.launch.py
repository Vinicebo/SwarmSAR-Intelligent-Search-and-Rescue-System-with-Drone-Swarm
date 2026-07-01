import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulator_share = get_package_share_directory('simulator')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(simulator_share, 'worlds', 'earthquake_city.world')
    model_path = os.path.join(simulator_share, 'models', 'quadrotor', 'model.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )

    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'drone_1',
            '-file', model_path,
            '-x', '0', '-y', '0', '-z', '0.5',
        ],
        output='screen',
    )

    hover_controller = Node(
        package='navigation',
        executable='hover_controller',
        namespace='drone_1',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        spawn_drone,
        hover_controller,
    ])
