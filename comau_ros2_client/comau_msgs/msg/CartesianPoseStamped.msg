##Usage: In ExecuteCartesianTrajectory.action

##Definition: A euler pose with a tf frame to transform the pose relative from.
##If the frame is "" will not transform the pose
std_msgs/Header header
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw

float64 lin_vel
uint64  seg_ovr
string  move_type
