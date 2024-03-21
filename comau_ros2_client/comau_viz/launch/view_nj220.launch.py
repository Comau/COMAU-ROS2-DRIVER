import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('nj220_description'), 'robots', 'nj220_robot.urdf.xacro')    
    assert os.path.exists(xacro_file), "The nj220_robot.urdf.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    #print(robot_desc)

    return LaunchDescription([
      Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc}],
        output="screen"),
          
      Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
          {"robot_description": robot_desc}]),
      Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')
        
    #    Node(
    #        package='joint_state_publisher_gui',
    #        executable='joint_state_publisher_gui',
    #        name='joint_state_publisher_gui',
    #      )
  ])
  
