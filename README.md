## Overview
This repository contains all the required ROS2 packages to work with Comau robots through ROS2.

The Comau Experimental package has been tested under ROS2 Humble and Ubuntu 22.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Acknowledgment
Developed in collaboration between:

[<img height="60" alt="COMAU" src="comau_ros2_client/doc/COMAU-logo.png">](https://www.comau.com/en)  &nbsp; and &nbsp;
[<img height="60" alt="LMS" src="comau_ros2_client/doc/LMS-logo.jpg">](http://lms.mech.upatras.gr)

## Short scheme

The library is based on TCP/IP communication between the ROS2 module running on an external PC and the PDL module running on the cabinet IPC. The following figure reports the set-up.

[<img height="60" alt="LMS" src="comau_ros2_client/doc/LMS-logo.jpg">](http://lms.mech.upatras.gr)

It is also possible to use the library in virtual mode. In this case, the user only needs a laptop where both Windows and Linux run. The former is necessary to support RoboShop Comau software tool; the latter is used for ROS2.

[<img height="60" alt="LMS" src="comau_ros2_client/doc/LMS-logo.jpg">](http://lms.mech.upatras.gr)

## How to Start the Communication on Robot Side

This repository contains the PDL code used for the COMAU ROS2 interface.

Translate all four PDL files into cod files and upload them on the C5GPlus controller using the FTP within the folder "UD:\USR\comau_ros2_server".
“pdl_tcp_functions” - NO HOLD PDL program with utility functions for the TCP/IP communication
“state_server` - NO HOLD PDL program that contains a TCP server for publishing robot's state
“robot_server” - NO HOLD PDL program that contains a TCP server for receiving driver management commands
“motion_server” - NO HOLD PDL program that contains a TCP server for receiving motion commands
“arm1_handler” - HOLD PDL program that executes the motion commands

Load and activate only the pdl_tcp_functions.cod file which will automatically load and activate the other files. It is necessary to set the robot in Drive-On state and press the green Start button before launching the client.
The Robot Controller should have the following software options:
TCP/IP

 [<img height="60" alt="LMS" src="comau_ros2_client/doc/LMS-logo.jpg">](http://lms.mech.upatras.gr)
