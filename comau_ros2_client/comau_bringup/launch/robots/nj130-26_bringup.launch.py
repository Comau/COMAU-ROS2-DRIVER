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
       default_value=os.path.join(get_package_share_directory('nj130-26_description'),
                         'launch/nj130-26_upload.launch.py'),
       description='Robot description launch file.'.format('/'), 
    )

    # Include comau_common.launch.py  
    comau_common_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('comau_bringup'),
                        'launch/common/comau_common.launch.py')),
        launch_arguments={
            'robot_description_file': LaunchConfiguration('robot_description_file'), 
            'ns': LaunchConfiguration('ns')
        }.items(),   
    )

    return LaunchDescription([
     ns_arg,
     robot_description_file_arg,
     comau_common_launch,   
      Node(
        package='comau_bringup',
        executable='load_yaml_param',
        parameters=[os.path.join(get_package_share_directory('comau_bringup'),
                        'config','transforms','parallel_joint_fix_value.yaml')],
        namespace=LaunchConfiguration('ns'),                
        output='screen',
      )
    ])

