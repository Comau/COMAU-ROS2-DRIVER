# Comau Experimental

[![catkin_build](https://github.com/LMS-Robotics-VR/comau_experimental/actions/workflows/main.yml/badge.svg)](https://github.com/LMS-Robotics-VR/comau_experimental/actions/workflows/main.yml)

## Overview

This repository contains all the required ROS packages to work with Comau robots through ROS.

The Comau Experimental package has been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Acknowledgment

Developed in collaboration between:

[<img height="60" alt="LMS" src="doc/LMS-logo.jpg">](http://lms.mech.upatras.gr) &nbsp; and &nbsp;
[<img height="60" alt="COMAU" src="doc/COMAU-logo.png">](https://www.comau.com/en)
 
This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 820689

<img src="doc/EU-flag.jpg" alt="eu_flag" height="45">

## Installation

### Building from Source

#### Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

It is recommended to use **Ubuntu 18.04 with ROS melodic**, however using Ubuntu 16.04 with ROS kinetic should also work.

#### Building procedure

```bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone the latest version of this repository into your catkin workspace *src* folder.
git clone <repository link>

# Install dependencies of all packages.
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin_make

# activate the workspace
source devel/setup.bash

```

## How to use the COMAU ROS

### Simulation

Follow the instructions at 

[comau_sim README](comau_sim/README.md)

## Real Robot

To start the driver follow the instructions at 

[comau_driver README](comau_driver/comau_hardware_interface/README.md)


### After that you are ready to start interfacing with the robot through ros 
1. How to use Comau controller wrapper
    
    [comau_controller_wrapper README](comau_tools/comau_controller_wrapper/README.md)

2. How to use Asynchronous Joint/Cartesian feature
    
    [comau_handlers README](comau_driver/comau_handlers/README.md)

3. How to use Sensor Tracking feature 

    [comau_controllers README](comau_driver/comau_controllers/README.md)

4. How to use Moveit
    
    [comau_moveit_handlers README](comau_moveit/comau_moveit_handlers/README.md)

5. **For examples on how to use the c++ and python api see the comau_demo package**
