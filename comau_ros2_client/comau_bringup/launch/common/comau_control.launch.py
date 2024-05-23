import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import yaml

def generate_launch_description():

    controllers_arg = DeclareLaunchArgument(
      name='controllers',
      default_value='joint_state_controller',
      description='Controllers that are activated by default.'
    )
    controllers_value = LaunchConfiguration('controllers')

    config = os.path.join(
      get_package_share_directory('comau_bringup'),
      'config',
      'driver',
      'controllers.yaml',
    )
   
## ******   comau_hardware_interface_node   TO DO  *****************

 #   comau_hardware_interface_node = Node(
 #      package='comau_hardware_interface',
 #      executable='comau_hardware_interface_node',
 #      name='comau_hardware_interface',
 #      output='screen'
 #   )

    comau_hardware_control_loop_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='comau_hardware_control_loop',    
      parameters=[config],                 
    )

    comau_hardware_interface_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='comau_hardware_interface',                
      parameters=[config],              
    )

    comau_driver_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='comau_driver',                
      parameters=[config],                              
    ) 
    comau_point_follower_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='comau_point_follower',                
      parameters=[config],                              
    )
   
    joint_state_controller_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='joint_state_controller',                
      parameters=[config],                              
    ) 
    sensor_tracking_relative_controller_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='sensor_tracking_relative_controller',                
      parameters=[config],                              
     )          
    sensor_tracking_absolute_controller_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='sensor_tracking_absolute_controller',                
      parameters=[config],                              
     )   
    pos_joint_traj_controller_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='pos_joint_traj_controller',                
      parameters=[config],                              
     )          
    arm_jog_controller_node= Node(
      package='comau_bringup',
      executable='load_param_node',
      name='arm_jog_controller',                
      parameters=[config],                              
     )     
 
    return LaunchDescription([
      controllers_arg,
      comau_hardware_control_loop_node,
      comau_hardware_interface_node,
      comau_driver_node,
      comau_point_follower_node,
      joint_state_controller_node,
      sensor_tracking_relative_controller_node,
      sensor_tracking_absolute_controller_node,
      pos_joint_traj_controller_node,
      arm_jog_controller_node
     
#       Node(
#           package='controller_manager',
#           executable='spawner',
#           name='controller_spawner',
#           output='screen',
#         #  parameters=['controllers_value'],
#        )
#    
#        Node(
#           package='controller_manager',
#           executable='controller_manager',
#           name='controller_loader',
#           output='screen',
#   #        arguments=['load', 'sensor_tracking_relative_controller', 'sensor_tracking_absolute_controller', 'pos_joint_traj_controller', 'arm_jog_controller']
#            arguments=['load', 'pos_joint_traj_controller', 'arm_jog_controller']
#
#        )
    ])
    
