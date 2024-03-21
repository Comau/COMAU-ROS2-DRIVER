import os
import xacro
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('racer7-14_description'), 'robots', 'racer7-14_robot.urdf.xacro')    
    assert os.path.exists(xacro_file), "The racer7-14.urdf.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    #print(robot_desc)

    return LaunchDescription([

  ])
