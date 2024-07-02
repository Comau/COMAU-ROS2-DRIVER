# comau_example

## Prerequisites

The asynchronous action servers require **no controller running** on the hardware interface, except the **joint_state_controller**. This is the **default** state of the hardware interface.

**Furthermore** they start directly with the hardware interface because they are included as libraries

To check if the asynchronous feature is enabled

```bash
rostopic echo /async_enable # If true no "write" controller is running
rostopic echo /robot_status # The status of the robot
```

**If the robot is in a ready state and the async_enable topic is true, the action servers will send the goal for execution on the real robot otherwise they will abort.**


**ATTENTION!!!**\
Always check the robot surroundings. Make sure that no one is near the robot.

## Send a asynchronous trajectory execution command

Start the `arm1_handler` PDL program.

Now you are ready to send a trajectory from an ROS action client to execute joint / cartesian trajectory  action server. You may find the definition of this ROS action at `comau_msgs` package.

For simple tests you can use the test with the following command:

```bash

ros2 run comau_example execute_example_node

```
### Joint trajectory

At the *Goal* area of the window you should place a trajectory of multiple joint positions.

```bash

ros2 param set /execute_example_node joints_demo_example 1

```

### Cartesian trajectory

At the *Goal* area of the window you should place a trajectory of multiple cartesian poses.

```bash

ros2 param set /execute_example_node cart_demo_example 1

```
Other Trajectory examples and specifications:

Joint Trajectory

At the Goal area of the window you should place a trajectory of multiple joint positions. 

```bash

trajectory: [
{positions: [0.436332, 0.0, -1.5708, 0.0, 0.0, 0.0], seg_ovr: 50, move_type: joint},
{positions: [0.872665, 0.0, -1.5708, 0.0, 0.0, 0.0], seg_ovr: 80, move_type: joint},
{positions: [1.22173, 0.0, -1.5708, 0.0, 0.0, 0.0]},
#
# other joint positions with angles in rad
#
{positions: [0.872665, 0.0, -1.0472, 0.0, 0.0, 0.0]}
]

```


At the Goal area of the window you should place a trajectory of multiple cartesian poses. 

```bash

trajectory: [
# relative to tool frame tool_controller or ee_link
{header: {frame_id: tool_controller}, x: 0.0, y: 0.0, z: 0.1, roll: 0.0, pitch: 0.0, yaw: 0.0, lin_vel: 0.5, move_type: linear},
# relative to base link
{header: {frame_id: base_link}, x: 0.9339, y: 0.0, z: 1.1506, roll: 0.0, pitch: 1.5707, yaw: 0.0, lin_vel: 0.1, seg_ovr: 50, move_type: joint},
#

# other cartesian poses with angles in rad
#
{header: {frame_id: base_link}, x: 0.9339, y: 0.0, z: 1.1506, roll: 0.0, pitch: 1.5707, yaw: 0.0}
]

```