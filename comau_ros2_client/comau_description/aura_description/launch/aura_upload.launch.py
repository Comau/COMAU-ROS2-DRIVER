import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
      name='robot/name',
      default_value='aura',
    )
        
    pos_x_arg = DeclareLaunchArgument(
      name='pos_x',
      default_value='0.0',
    )
        
    pos_y_arg = DeclareLaunchArgument(
      name='pos_y',
      default_value='0.0',
    )

    pos_z_arg = DeclareLaunchArgument(
      name='pos_z',
      default_value='0.0',
    )

    roll_arg = DeclareLaunchArgument(
      name='roll',
      default_value='0.0',
    )
        
    pitch_arg = DeclareLaunchArgument(
      name='pitch',
      default_value='0.0',
    )

    yaw_arg = DeclareLaunchArgument(
      name='yaw',
      default_value='0.0',
    )

#      ***********     To Do ******************* 
    #transmission_hw_interface_arg = DeclareLaunchArgument(
    #  name='transmission_hw_interface',
    #  default_value='',  #TO-DO 
    # #default="hardware_interface/PositionJointInterface"  ros1 
    #)

    use_mimic_arg = DeclareLaunchArgument(
      name='use_mimic',
      default_value='false',
      description='Loads urdf with mimic joint if true',
    )
    
    #xacro_file_path= os.path.join(get_package_share_directory('aura_description'),'robots/aura_robot.urdf.xacro'),
        
    xacro_file = os.path.join(get_package_share_directory('aura_description'), 'robots', 'aura_robot.urdf.xacro')    
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    #print(robot_description)
    


    return LaunchDescription([
      robot_name_arg,  
      pos_x_arg,
      pos_y_arg,
      pos_z_arg,
      roll_arg,
      pitch_arg,
      yaw_arg,
      use_mimic_arg
    ])
