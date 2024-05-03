import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
   
    ns_arg = DeclareLaunchArgument(
        name='ns',
        default_value='/',
        description='Namespace'
    )

    robot_description_file_arg = DeclareLaunchArgument(
       name='robot_description_file',
       default_value=os.path.join(get_package_share_directory('aura_description'),
                         'launch/aura_upload.launch.py'),
       description='Robot description launch file.', 
    )

    use_mimic_arg = DeclareLaunchArgument(
        name='use_mimic',
        default_value='false',
        description='Loads urdf with mimic joint if true.'
    )
  
    # Include comau_common.launch.py  
    comau_common_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('comau_bringup'),
                        'launch/common/comau_common.launch.py')),
        launch_arguments={
            'robot_description_file': LaunchConfiguration('robot_description_file'), 
            'use_mimic':LaunchConfiguration('use_mimic'),
            'ns': LaunchConfiguration('ns')
        }.items(),   
    )

    config = os.path.join(
        get_package_share_directory('comau_bringup'),
        'config',
        'transforms',
        'parallel_joint_fix_value.yaml',
        )
    
    return LaunchDescription([
      ns_arg,
      robot_description_file_arg,
      use_mimic_arg,
      comau_common_launch, 
        
      Node(
        package='comau_bringup',
        executable='load_param_node',
        name='param_value',              
        parameters=[config],                               
      )
    ])

