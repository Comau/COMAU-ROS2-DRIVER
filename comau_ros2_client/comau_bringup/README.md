# comau_action_handlers

Here is a component diagram of the comau action servers

![](doc/comau_action_handlers_component_diagram.svg)

## Prerequisites

The asynchronous action servers require **no controller running** on the hardware interface, except the **joint_state_controller**. This is the **default** state of the hardware interface.

**Furthermore** they start directly with the hardware interface because they are included as libraries

To check if the asynchronous feature is enabled

```bash
rostopic echo /async_enable # If true no "write" controller is running
rostopic echo /robot_status # The status of the robot
```

**If the robot is in a ready state and the async_enable topic is true, the action servers will send the goal for execution on the real robot otherwise they will abort.**

**If sensor tracking controller is running disable it with a service call to the controller manager**

```bash
rosservice call /comau_controller_manager "Mode: 1"
```



**ATTENTION!!!**\
Always check the robot surroundings. Make sure that no one is near the robot.

## Send a asynchronous trajectory execution command

Start the `arm1_handler` PDL program.

Now you are ready to send a trajectory from an ROS action client to execute joint / cartesian trajectory  action server. You may find the definition of this ROS action at `comau_msgs` package.

For simple tests you can use the test [GUI action client](https://github.com/ros/actionlib/tree/noetic-devel/actionlib_tools) with the following command:

```bash
rosrun actionlib axclient.py /execute_joint_trajectory_handler
rosrun actionlib axclient.py /execute_cartesian_trajectory_handler
```
### Joint trajectory

At the *Goal* area of the window you should place a trajectory of multiple joint positions. For valid goals, you should follow the format of the example bellow :

```yaml
trajectory: [
positions: [0.436332, 0.0, -1.5708, 0.0, 0.0, 0.0],
positions: [0.872665, 0.0, -1.5708, 0.0, 0.0, 0.0],
positions: [1.22173, 0.0, -1.5708, 0.0, 0.0, 0.0],
#
# other joint positions with angles in rad
#
positions: [0.872665, 0.0, -1.0472, 0.0, 0.0, 0.0]
]
```

### Cartesian trajectory

At the *Goal* area of the window you should place a trajectory of multiple cartesian poses. For valid goals, you should follow the format of the example bellow :

```yaml
trajectory: [
# relative to tool frame tool_controller or ee_link
{header: {frame_id: tool_controller}, x: 0.0, y: 0.0, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 0.0},
# relative to base link
{header: {frame_id: base_link}, x: 0.9339, y: 0.0, z: 1.1506, roll: 0.0, pitch: 1.5707, yaw: 0.0},
#
# other cartesian poses with angles in rad
#
{header: {frame_id: base_link}, x: 0.9339, y: 0.0, z: 1.1506, roll: 0.0, pitch: 1.5707, yaw: 0.0}
]
```

When you click send goal the robot should start to move along the trajectory that you have send.

## Authors
LMS - Laboratory for Manufacturing Systems & Automation University of Patras, Greece 
[<img height="60" alt="LMS" src="../../doc/LMS-logo.jpg">](http://lms.mech.upatras.gr)