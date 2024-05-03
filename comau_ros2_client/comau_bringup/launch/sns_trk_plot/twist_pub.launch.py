from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      twist_sin_pub_node = Node(
        package='comau_demo',
        executable='twist_sin_pub',
        name='twist_sin_pub',
        output='screen',
      )

      rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
      )

    return LaunchDescription([
       twist_sin_pub_node,
       rqt_reconfigure_node
      ])
