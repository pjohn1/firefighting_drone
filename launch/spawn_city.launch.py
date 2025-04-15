from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('firefighting_drone')
    
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

    return LaunchDescription([
        spawn_city
    ]) 