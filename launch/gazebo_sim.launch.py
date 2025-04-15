from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('firefighting_drone')
    
    # Gazebo launch with our world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'verbose': 'true',
            'world': os.path.join(pkg_share, 'worlds', 'city_world.world'),
            'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        }.items()
    )

    # Spawn city model
    spawn_city = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'city',
                  '-file', os.path.join(pkg_share, 'models', 'map', 'model.sdf'),
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.0'],
        output='screen'
    )

    # Spawn drone model
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'drone',
                  '-file', os.path.join(pkg_share, 'models', 'drone', 'model.sdf'),
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '2.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_city,
        spawn_drone
    ]) 