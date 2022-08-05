import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('cfe_plugin'),
        'config',
        'cfe_config.yaml'
        )

    node = Node(
        package='fsw_ros2_bridge',
        name='cfe_bridge',
        executable='fsw_ros2_bridge',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
