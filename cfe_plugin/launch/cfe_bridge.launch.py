import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    # Argument to set cfe_config file to use, dependent on your specific setup
    # default: cfe_config.yaml is good for single-host setups 
    # alternative: cfe_config_multihost.yaml is good for multihost setups, such as the docker demo 
    cfe_config = LaunchConfiguration('cfe_config')
    cfe_config_launch_arg = DeclareLaunchArgument('cfe_config', default_value='cfe_config.yaml',
                            description='cfe_config yaml file')
        
    config = PathJoinSubstitution([
        get_package_share_directory('cfe_plugin'),
        'config',
        cfe_config
        ])

    juicer_config = PathJoinSubstitution([
        get_package_share_directory('juicer_util'),
        'config',
        'cfe_plugin_config.yaml'
        ])

    node = Node(
        package='fsw_ros2_bridge',
        name='cfe_bridge',
        executable='fsw_ros2_bridge',
        parameters=[config, juicer_config]
    )

    return LaunchDescription([
      cfe_config_launch_arg,
      node
    ])

