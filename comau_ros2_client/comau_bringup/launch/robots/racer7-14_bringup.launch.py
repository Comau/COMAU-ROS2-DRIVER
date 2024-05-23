import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Load robot_description
    xacro_file_path = os.path.join(get_package_share_directory('racer7-14_description'), 'robots', 'racer7-14_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    xml = robot_description_config.toxml()  
       
    robot_description_file_arg = DeclareLaunchArgument(
       name='robot_description_file',
       default_value=os.path.join(get_package_share_directory('racer7-14_description'),
                         'launch/racer7-14_upload.launch.py'),
       description='Robot description launch file.', 
    )
  
    # Include comau_common.launch.py  
    comau_common_launch = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('comau_bringup'),
                        'launch/common/comau_common.launch.py')),
        launch_arguments={
            'robot_description_file': LaunchConfiguration('robot_description_file')
        }.items(),   
    )

    return LaunchDescription([
      robot_description_file_arg,
      comau_common_launch,

      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':xml}]
      ),
    
      Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description':xml}]
      )        
    ])

