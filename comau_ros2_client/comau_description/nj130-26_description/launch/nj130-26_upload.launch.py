import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
      name='robot/name',
      default_value='nj130-26',
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

# to do
    #transmission_hw_interface_arg = DeclareLaunchArgument(
    #  name='transmission_hw_interface',
    #  default_value='',  #TO-DO 
    # #default="hardware_interface/PositionJointInterface"  ros1 
    #)
    
    xacro_file_path= os.path.join(get_package_share_directory('nj130-26_description'),'robots/nj130-26_robot.urdf.xacro'),
           
    #robot_description_config = xacro.process_file(xacro_file)
    #robot_description = robot_description_config.toxml()
#
    #print(robot_description)

    return LaunchDescription([
      robot_name_arg,  
      pos_x_arg,
      pos_y_arg,
      pos_z_arg,
      roll_arg,
      pitch_arg,
      yaw_arg
    ])
