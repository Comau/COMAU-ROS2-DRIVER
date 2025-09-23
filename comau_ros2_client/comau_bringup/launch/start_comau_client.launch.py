# Copyright 2024 Comau Robotics S.p.A.

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
    
    match robot_type:
        case "aura":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('aura_description'),
                        'launch/',
                        'view_aura.launch.py'
                    ])
                ]),
            )
        ])
        case "aura-mimic":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('aura-mimic_description'),
                        'launch/',
                        'view_aura-mimic.launch.py'
                    ])
                ]),
            )
        ])
        case "edo":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('edo_description'),
                        'launch/',
                        'view_edo.launch.py'
                    ])
                ]),
            )
        ])
        case "nj4-110":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj4-110_description'),
                        'launch/',
                        'view_nj4-110.launch.py'
                    ])
                ]),
            )
        ])
        case "nj-165-30":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj-165-30_description'),
                        'launch/',
                        'view_nj-165-30.launch.py'
                    ])
                ]),
            )
        ])
        case "nj4-170-29":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj4-170-29_description'),
                        'launch/',
                        'view_nj4-170-29.launch.py'
                    ])
                ]),
            )
        ])
        case "nj130-26":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj130-26_description'),
                        'launch/',
                        'view_nj130-26.launch.py'
                    ])
                ]),
            )
        ])
        case "nj220":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj220_description'),
                        'launch/',
                        'view_nj220.launch.py'
                    ])
                ]),
            )
        ])
        case "racer3":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('racer3_0_63_description'),
                        'launch/',
                        'view_racer3_0_63.launch.py'
                    ])
                ]),
            )
        ])
        case "racer3-rl":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('racer3-robolab_description'),
                        'launch/',
                        'view_racer3-robolab.launch.py'
                    ])
                ]),
            )
        ])
        case "racer5-0-80":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('racer5-0-80_description'),
                        'launch/',
                        'view_racer5-0-80.launch.py'
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
                        FindPackageShare('racer5-cobot-rail_description'),
                        'launch/',
                        'view_racer5-cobot-rail.launch.py'
                    ])
                ]),
            )
        ])
        case "racer7-14":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('racer7-14_description'),
                        'launch/',
                        'view_racer7-14.launch.py'
                    ])
                ]),
            )
        ])
        case "nj60":
            return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nj60_description'),
                        'launch/',
                        'view_nj60.launch.py'
                    ])
                ]),
            )
        ])
        case _:
            print("Client closed. Please choose a robot_type:= \n aura \n aura-mimic \n edo \n nj4-110 \n nj-165-30 \n nj4-170-29 \n nj130-26 \n nj220 \n nj60 \n racer3 \n racer3-rl \n racer5-0-80 \n racer5-cobot \n racer5-cobot-rail \n racer7-14")
            sys.exit()
    
    return
