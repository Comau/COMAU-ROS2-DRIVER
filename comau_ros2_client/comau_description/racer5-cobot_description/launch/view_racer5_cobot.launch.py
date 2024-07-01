# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Load yaml:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("racer5-cobot_description"),
                    "robots",
                    "racer5-cobot_robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    config_param = os.path.join(
      get_package_share_directory('comau_bringup'),
      'config',
      'general_config.yaml',
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("comau_hardware_interface"),
            "config",
            "robot_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("racer5-cobot_description"), "launch", "view_cobot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(gui),
    )

    joint_state_publisher_node_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "-c", "/controller_manager"],
    )

    send_trajectory_node = Node(
        package="comau_handlers",
        executable="execute_trajectory_handler_node",
        name="execute_trajectory_handler_node",
        parameters=[robot_description,config_param],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes_to_start = [
        #joint_state_publisher_node_gui,                                       # gui
        #joint_state_publisher_node,                                           # sim
        control_node,                                                          # both
        robot_state_publisher_node,                                            # both
        #joint_state_broadcaster_spawner,                                      # ROS COMAU CONTROL
        #delay_rviz_after_joint_state_broadcaster_spawner,                     # ROS COMAU CONTROL
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner, # ROS COMAU CONTROL
        send_trajectory_node,                                                  # both
        rviz_node,                                                             # Update URDF
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
