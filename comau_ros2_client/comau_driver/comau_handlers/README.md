# comau_handlers

## How to use Asynchronous Joint/Cartesian feature

## Prerequisites

The asynchronous action servers require no controller running on the hardware interface, except the joint_state_controller. This is the default state of the hardware interface.
To check if the asynchronous feature is enabled you can echo the following topics:

```bash
ros2 topic echo /async_enable # If true no "write" controller is running
ros2 topic echo /robot_status # The status of the robot
```

**If the robot is in a ready state and the async_enable topic is true, the action servers will send the goal for execution on the real robot otherwise they will abort.**

## Send a asynchronous trajectory execution command

Start the `arm1_handler` PDL program.

Now you are ready to send a trajectory from an ROS action client to execute joint / cartesian trajectory  action server. You may find the definition of this ROS action at `comau_msgs` package.

As soon as you start the arm1_handler PDL program, you are ready to send a trajectory from an ROS action client to execute joint/cartesian trajectory action server. You may find the definition of this ROS action at comau_msgs package. 
The cartesian trajectory is a comau_msgs/msg/CartesianPoseStamped array which has the following fields:

```bash
## Definition: A euler pose with a tf frame to transform the pose relative from.
Header header
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw


float64 lin_vel
uint64  seg_ovr
string  move_type
```
where:
header is the reference frame,
{x, y, z, roll, pitch, yaw} is the goal pose of the node in [m, rad],
lin_vel is the maximum linear velocity that the robot can reach during the execution of the node in [m/s] (default default_linear_velocity m/s in config.yaml file. Always check the limit value $LIN_SPD_LIM), 
seg_ovr is an integer value and it represents the override of the node (default 100),
move_type is a case-insensitive string which defines the type of robot movement to reach the node. It can be JOINT, LINEAR, SEG_VIA, CIRCULAR (default value is joint). Notice that you always need a SEG_VIA node before a CIRCULAR one. Check the Comau manual for more information.

The cartesian trajectory is a comau_msgs/msg/JointPose array which has the following fields:

```bash
## Definition: A euler pose with a tf frame to transform the pose relative from.
float64[] positions

uint64  seg_ovr
string  move_type

```
where:
positions is an array of float which contains the joints position in radiants.
seg_ovr is an integer value and it represents the override of the node (default 100)
move_type is a case-insensitive string which defines the type of robot movement to reach the node. It can be JOINT, LINEAR, SEG_VIA, CIRCULAR (default value is joint). Notice that you always need a SEG_VIA node before a CIRCULAR one. Check the Comau manual for more information.

### For simple tests

You can use the test comau_example package running the node example with the following command:
```bash
ros2 run comau_example execute_example_node
```
to run the cartesian example:

```bash
ros2 param set /execute_example_node cart_demo_example 1
```
to run the joints example:

```bash
ros2 param set /execute_example_node joints_demo_example 1
```

### Joint trajectory

At the Goal area of the window you should place a trajectory of multiple joint positions. For valid goals, you should follow the format of the example below:

```yaml
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

### Cartesian trajectory

At the Goal area of the window you should place a trajectory of multiple cartesian poses. For valid goals, you should follow the format of the example below:

```yaml
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