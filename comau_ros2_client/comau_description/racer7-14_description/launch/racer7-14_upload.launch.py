# Copyright 2024 Comau Robotics S.p.A.

import os
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    

  # Path to the .xacro file
  xacro_file_path = os.path.join(
    get_package_share_directory('racer7-14_description'),
    'robots',
    'racer7-14_robot.urdf.xacro'
  )

  # Define the launch argument for pos_z
  pos_x_arg = 'pos_x'
  pos_y_arg = 'pos_y'
  pos_z_arg = 'pos_z'
  roll_arg  = 'roll'
  pitch_arg = 'pitch'
  yaw_arg   = 'yaw'

  robot_name_arg = 'robot_name'

  declare_pos_x_cmd = DeclareLaunchArgument(
    pos_x_arg,
    default_value='0.0',
    description='The x position of the robot'
  )

  declare_pos_y_cmd = DeclareLaunchArgument(
    pos_y_arg,
    default_value='0.0',
    description='The y position of the robot'
  )        

  declare_pos_z_cmd = DeclareLaunchArgument(
    pos_z_arg,
    default_value='0.0',
    description='The z position of the robot'
  )

  declare_roll_cmd = DeclareLaunchArgument(
    roll_arg,
    default_value='0.0',
    description='The roll of the robot'
  )

  declare_pitch_cmd = DeclareLaunchArgument(
    pitch_arg,
    default_value='0.0',
    description='The pitch of the robot'
  )        

  declare_yaw_cmd = DeclareLaunchArgument(
    yaw_arg,
    default_value='0.0',
    description='The yaw of the robot'
  )

  declare_robot_name_cmd = DeclareLaunchArgument(
    robot_name_arg,
    default_value='racer7-14',
  )

  #    ***********     TO DO    ******************* 
  # transmission_hw_interface_arg = DeclareLaunchArgument(
  # name='transmission_hw_interface',
  # default_value='',  
  # default="hardware_interface/PositionJointInterface"  
  #)

  # Get the values from the launch argument
  pos_x = LaunchConfiguration(pos_x_arg)
  pos_y = LaunchConfiguration(pos_y_arg)
  pos_z = LaunchConfiguration(pos_z_arg)
  roll  = LaunchConfiguration(roll_arg)
  pitch = LaunchConfiguration(pitch_arg)
  yaw   = LaunchConfiguration(yaw_arg)

  robot_name = LaunchConfiguration(robot_name_arg)

  robot_description = Command([
    FindExecutable(name='xacro'), ' ',
    xacro_file_path, ' ', 
  ])

  # Command to interprete robot_description as a string
  robot_description_param = ParameterValue(robot_description, value_type=str)

  return LaunchDescription([
    declare_pos_x_cmd,
    declare_pos_y_cmd,
    declare_pos_z_cmd,
    declare_roll_cmd,
    declare_pitch_cmd,
    declare_yaw_cmd,
    declare_robot_name_cmd
  ])
