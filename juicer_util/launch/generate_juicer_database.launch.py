import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

#####################################
def generate_launch_description():
    
  cfs_path_arg = DeclareLaunchArgument(
    'cfs_path',
    default_value='~/code/cFS')
  juicer_path_arg = DeclareLaunchArgument(
    'juicer_path',
    default_value='~/code/juicer')
  output_db_arg = DeclareLaunchArgument(
    'output_db',
    default_value='combined.sqlite')

  cfs_path = LaunchConfiguration('cfs_path')
  juicer_path = LaunchConfiguration('juicer_path')
  output_db = LaunchConfiguration('output_db')


  input_list = ['core-cpu1', 'cf/cfe_assert.so', 'cf/ci_lab.so', 
    'cf/ros_app.so', 'cf/sample_app.so', 'cf/sample_lib.so', 
    'cf/sbn_f_remap.so', 'cf/sbn.so', 'cf/sbn_udp.so', 'cf/sch_lab.so', 
                'cf/to_lab.so', 'cf/robot_sim.so', 'cf/cf.so', 'cf/rover_app.so', 'cf/sntp.so']
  
  # Generate Message node
  generate_juicer_db = Node(
        package='juicer_util',
        executable='generate_juicer_database',
        output='screen',
        parameters=[
            {"cfs_path": cfs_path},
            {"juicer_path": juicer_path},
            {"output": output_db},
            {"files": input_list}
        ]
  )    


  return LaunchDescription([
      # Arguments
      cfs_path_arg,
      juicer_path_arg,
      output_db_arg,
      # Juicer message generator
      generate_juicer_db
    ])

   
