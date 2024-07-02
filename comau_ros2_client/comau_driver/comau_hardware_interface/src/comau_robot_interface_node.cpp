/**
 * @file comau_robot_interface_node.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes control loop and hw interface
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <thread>
#include <memory>

#include <csignal>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("comau_robot_interface_node");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/robot_controller/joint_trajectory", 10);

  // get robot description
  auto robot_param = rclcpp::Parameter();
  node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  node->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  int startROS2Traj;
  startROS2Traj = 0;
  node->declare_parameter("control_traj",0);

  // create kinematic chain
  KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "ee_link", chain);

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();
  // create KDL solvers
  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  for (size_t i = 0; i < chain.getNrOfSegments(); i++)
  {
    auto joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
    {
      trajectory_msg.joint_names.push_back(joint.getName());
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(chain.getNrOfJoints());
  trajectory_point_msg.velocities.resize(chain.getNrOfJoints());

  double total_time = 3.0;
  int trajectory_len = 200;
  int loop_r = trajectory_len / total_time;
  double dt = 1.0 / loop_r;
  rclcpp::Rate loop_rate(loop_r);
  for (int i = 0; i < trajectory_len; i++)
  {
    // set endpoint twist
    double t = i;
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t / trajectory_len));
    twist.vel.y(-0.3 * sin(2 * M_PI * t / trajectory_len));

    // convert cart to joint velocities
    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

    // copy to trajectory_point_msg
    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
    std::memcpy(
      trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      trajectory_point_msg.velocities.size() * sizeof(double));

    // integrate joint velocities
    joint_positions.data += joint_velocities.data * dt;

    // set timing information
    trajectory_point_msg.time_from_start.sec = i / loop_r;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
      1E9 / loop_r *
      static_cast<double>(t - loop_r * (i / loop_r)));  // implicit integer division

    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  while (rclcpp::ok())
  {
    startROS2Traj = node->get_parameter("control_traj").as_int();
    if(startROS2Traj == 1)
    {
      startROS2Traj = 0;
      rclcpp::Parameter set_startROS2Traj("control_traj", startROS2Traj);
      node->set_parameter(set_startROS2Traj);

      pub->publish(trajectory_msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}