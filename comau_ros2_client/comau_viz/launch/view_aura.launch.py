import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pos_x_name = 'pos_x'
    pos_y_name = 'pos_y'
    pos_z_name = 'pos_z'
    roll_name  = 'roll'
    pitch_name = 'pitch'
    yaw_name   = 'yaw'

    pos_x  = LaunchConfiguration(pos_x_name)
    pos_y  = LaunchConfiguration(pos_y_name)
    pos_z  = LaunchConfiguration(pos_z_name)
    roll   = LaunchConfiguration(roll_name )
    pitch  = LaunchConfiguration(pitch_name)
    yaw    = LaunchConfiguration(yaw_name  )
    
    xacro_file_path= os.path.join(get_package_share_directory('aura_description'), 'robots', 'aura_robot.urdf.xacro')  
    robot_description_config = xacro.process_file(xacro_file_path)
    xml = robot_description_config.toxml()

    
    return LaunchDescription([
        DeclareLaunchArgument(
                pos_x_name,
                default_value='0.0',
        ) ,             
        DeclareLaunchArgument(
                pos_y_name,
                default_value='0.0',
        ) ,
        DeclareLaunchArgument(
                pos_z_name,
                default_value='0.0',
        ) ,
        DeclareLaunchArgument(
                roll_name,
                default_value='0.0',
        ), 
        DeclareLaunchArgument(
                pitch_name,
                default_value='0.0',
        ) ,
        DeclareLaunchArgument(
                yaw_name,
                default_value='0.0',
        ) ,
              
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':xml }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': xml}],
        ) ,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
      
    ])