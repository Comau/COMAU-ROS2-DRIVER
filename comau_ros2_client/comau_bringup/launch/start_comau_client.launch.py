from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchService
import sys
import launch.actions
import launch.events

def generate_launch_description():
    """Main."""
    
    robot_type = ""
    
    for arg in sys.argv:
        if arg.startswith("robot_type:="):
            robot_type = str(arg.split(":=")[1])
    
    colors = {
            'background_r': '200'
        }
    
    match robot_type:
        case "aura":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'aura_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "aura-mimic":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'aura-mimic_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "nj4-110":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'nj4-110_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "nj4-170-29":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'nj4-170-29_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "nj130-26":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'nj130-26_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "nj220":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'nj220_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "racer5-0-80":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'racer5-0-80_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "racer5-cobot":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('racer5-cobot_description'),
                        'launch/',
                        'view_racer5_cobot.launch.py'
                    ])
                ]),
            )
        ])
        case "racer5-cobot-rail":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'racer5-cobot-rail_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case "racer7-14":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros2_comau_client'),
                        'comau_bringup/launch/robots/',
                        'racer7-14_bringup.launch.py'
                    ])
                ]),
            )
        ])
        case _:
            print("Client closed. Please choose a robot with robot_type:= \n aura \n aura-mimic \n nj4-110 \n nj4-170-29 \n nj130-26 \n nj220 \n racer5-0-80 \n racer5-cobot \n racer5-cobot-rail \n racer7-14")
            sys.exit()
    
    return
