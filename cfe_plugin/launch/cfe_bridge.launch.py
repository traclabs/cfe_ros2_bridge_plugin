import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()
    juicer_db = LaunchConfiguration("juicer_db")
    commands = LaunchConfiguration("commands")
    telemetry = LaunchConfiguration("telemetry")
    command_data = LaunchConfiguration("command_data")
    telemetry_data = LaunchConfiguration("telemetry_data")
    juicer_package = LaunchConfiguration("juicer_package")

    config = os.path.join(
        get_package_share_directory('cfe_plugin'),
        'config',
        'cfe_config.yaml'
        )

    juicer_config = os.path.join(
        get_package_share_directory('juicer_util'),
        'config',
        'cfe_cmd_tlm_config.yaml'
        )


    node = Node(
        package='fsw_ros2_bridge',
        name='cfe_bridge',
        executable='fsw_ros2_bridge',
        parameters=[config, juicer_config]
    )
    ld.add_action(node)
    return ld
