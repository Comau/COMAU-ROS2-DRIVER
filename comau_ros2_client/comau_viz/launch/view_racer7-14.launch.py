# Copyright 2024 Comau Robotics S.p.A.

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable,LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # Path to the .xacro file
  xacro_file_path = os.path.join(
    get_package_share_directory('racer7-14_description'),
    'robots',
    'racer7-14_robot.urdf.xacro'
  )

   ## If you change, from racer7-14_description/racer7-14_upload.launch.py the values ​​of:
   # 'pos_x'       # 'pitch'
   # 'pos_y'       # 'yaw'
   # 'pos_z'       # 'robot_name'
   # 'roll'        # 'use_mimic'
   
   ## To apply the change (i.e. pos_z) uncomment the following lines:
    # pos_z_arg = 'pos_z'    
    # declare_pos_z_cmd = DeclareLaunchArgument(
    # 'pos_z_arg',
    # default_value='',
    # )
    
  robot_description = Command([
    FindExecutable(name='xacro'), ' ',
    xacro_file_path, ' ', 
  # 'pos_z:=', LaunchConfiguration(pos_z_arg)     
  ])

  # Command to interprete robot_description as a string
  robot_description_param = ParameterValue(robot_description, value_type=str)

  # Include the launch file 
  robot_upload_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('racer7-14_description'),'launch/racer7-14_upload.launch.py')
    ),
  )

  # Robot state publisher
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description_param}]
  )

  # Joint state publisher
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'robot_description': robot_description_param}]
  )

  # RViz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen'
  )

  return LaunchDescription([
    robot_upload_launch,
    robot_state_publisher_node,
    joint_state_publisher_node,
    rviz_node       
  ])