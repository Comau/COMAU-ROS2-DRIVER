import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

   # Path to the .xacro file    
    xacro_file_path = os.path.join(
      get_package_share_directory('aura_description'),
      'robots',
      'aura_robot.urdf.xacro'
    )

    robot_description = Command([
      FindExecutable(name='xacro'), ' ',
      xacro_file_path, ' ', 
    ])

    # Command to interprete robot_description as a string
    robot_description_param = ParameterValue(robot_description, value_type=str)

    use_mimic_arg = DeclareLaunchArgument(
        'use_mimic',
        default_value='false',
        description='Loads urdf with mimic joint if true'
    )
   
    use_mimic_value = LaunchConfiguration('use_mimic')

    mimic_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description),
        condition=IfCondition(use_mimic_value),
    )

    #no_mimic_include = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(robot_description),
    #    condition=UnlessCondition(use_mimic_value),
    #)
#
   # Include comau_control.launch.py  
    comau_control_launch = IncludeLaunchDescription( 
       PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('comau_bringup'),
                       'launch/common/comau_control.launch.py')), 
    )

   #  **********************         TO DO   **********************************************   
   #  Include comau_controller_wrapper.launch.py 
    
   # comau_controller_wrapper_launch = IncludeLaunchDescription( 
   #     PythonLaunchDescriptionSource(
   #         os.path.join(get_package_share_directory('comau_controller_wrapper'),
   #                     'launch/comau_controller_wrapper.launch.py')),
   # )
   # ******************************************** 
 
    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_description_param}]
    )

    config =  os.path.join(
       get_package_share_directory('comau_bringup'), 
       'config',
       'net',
       'roboshop_net_config.yaml'
    )    

    load_param = Node(
        package='comau_bringup',
        executable='load_param_node',
        name='comau_driver', 
        parameters=[config],   
    )

    return LaunchDescription([
      use_mimic_arg,
      mimic_include,
    #  no_mimic_include,
      comau_control_launch,
      robot_state_publisher_node,
      load_param
    ])