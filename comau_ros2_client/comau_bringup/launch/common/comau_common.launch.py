import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
         default_value='',
        description='Robot description launch file.'
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
         default_value='',
        description='tf_prefix used for the robot.'
    )

    use_mimic_arg = DeclareLaunchArgument(
        'use_mimic',
        default_value='false',
        description='Loads urdf with mimic joint if true'
    )

    robot_description_file_value = LaunchConfiguration('robot_description_file')
   
    use_mimic_value = LaunchConfiguration('use_mimic')


    mimic_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_file_value),
        condition=IfCondition(use_mimic_value),
    )

    no_mimic_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_file_value),
        condition=UnlessCondition(use_mimic_value),
    )

   # Include comau_control.launch.py  
    comau_control_launch = IncludeLaunchDescription( 
       PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('comau_bringup'),
                       'launch/common/comau_control.launch.py')), 
    )

   #  **********************         TO DO   ***********************************************   
   #  Include comau_controller_wrapper.launch.py 
    
   # comau_controller_wrapper_launch = IncludeLaunchDescription( 
   #     PythonLaunchDescriptionSource(
   #         os.path.join(get_package_share_directory('comau_controller_wrapper'),
   #                     'launch/comau_controller_wrapper.launch.py')),
   # )
   # *************************************************************************************
    xacro_file_path = os.path.join(get_package_share_directory('aura_description'), 'robots', 'aura_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    xml = robot_description_config.toxml()

    config =  os.path.join(
       get_package_share_directory('comau_bringup'), 
       'config',
       'net',
       'roboshop_net_config.yaml'
    )    

    return LaunchDescription([
        robot_description_file_arg,
        tf_prefix_arg,
        use_mimic_arg,
        mimic_include,
        no_mimic_include,
        comau_control_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xml}]
        ),

        Node(
        package='comau_bringup',
        executable='load_param_node',
        name='comau_driver', 
        parameters=[config],   
    )

    ])