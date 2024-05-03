from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      rqt_multiplot_x_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_x',
        output='screen',
        arguments=['--multiplot-config', '$(find comau_bringup)/config/rqt_config/rqt_multiplot_x.xml']
      )

      rqt_multiplot_y_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_y',
        output='screen',
        arguments=['--multiplot-config', '$(find comau_bringup)/config/rqt_config/rqt_multiplot_y.xml']
      )
      rqt_multiplot_z_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_z',
        output='screen',
        arguments=['-multiplot-config', '$(find comau_bringup)/config/rqt_config/rqt_multiplot_z.xml']
      )
 
      rqt_multiplot_roll_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_roll',
        output='screen',
        arguments=['--multiplot-config',' $(find comau_bringup)/config/rqt_config/rqt_multiplot_roll.xml']
      )

      rqt_multiplot_pitch_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_pitch',
        output='screen',
        arguments=['--multiplot-config',' $(find comau_bringup)/config/rqt_config/rqt_multiplot_pitch.xml']
      )

      rqt_multiplot_yaw_node = Node(
        package='rqt_multiplot',
        executable='rqt_multiplot',
        name='rqt_multiplot_yaw',
        output='screen',
        arguments=['--multiplot-config', '$(find comau_bringup)/config/rqt_config/rqt_multiplot_yaw.xml']
      )

    return LaunchDescription([
       rqt_multiplot_x_node,
       rqt_multiplot_y_node,
       rqt_multiplot_z_node,
       rqt_multiplot_roll_node,
       rqt_multiplot_pitch_node,
       rqt_multiplot_yaw_node
      ])

#   return LaunchDescription([
#       Node(
#           package='rqt_multiplot',
#           executable='rqt_multiplot',
#           name='rqt_multiplot_x',
#           output='screen',
#           arguments=['--multiplot-config', '$(find comau_bringup)/config/rqt_config/rqt_multiplot_x.xml']
#       )
 #   ])