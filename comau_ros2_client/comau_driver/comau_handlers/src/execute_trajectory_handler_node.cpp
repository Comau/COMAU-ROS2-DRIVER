/**
 * @file execute_trajectory_handler_node.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the trajectory action
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
#include "rclcpp/rclcpp.hpp"
#include "comau_handlers/trajectory_handler.hpp"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;
using namespace comau_action_handlers;

boost::shared_ptr<trajectory_handler::TrajectoryHandler> c_exec_handler_ptr;

std::shared_ptr<rclcpp::Node> nh;

int startTrajAction = 0;

void signalHandler(int signum) 
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("execute_trajectory_handler_node")," Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  nh = std::make_shared<rclcpp::Node>("execute_trajectory_handler_node");

  nh->declare_parameter("loop_hz", 500.0);
  nh->declare_parameter("cycle_time_error_threshold", 0.0025);
  
  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Create the interface
  c_exec_handler_ptr.reset(new trajectory_handler::TrajectoryHandler(nh));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), "New Session:");
  c_exec_handler_ptr->loop_hz_ = nh->get_parameter("loop_hz").as_double();
  rclcpp::Rate loop_rate(c_exec_handler_ptr->loop_hz_);
  c_exec_handler_ptr->cycle_time_error_threshold_ = nh->get_parameter("cycle_time_error_threshold").as_double();
  if (!c_exec_handler_ptr->init()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("trajectory_handler"), "Could not correctly initialize robot. Exiting");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), "HW interface initialized");
  RCLCPP_WARN_STREAM(rclcpp::get_logger("trajectory_handler"), "loop_hz: " << c_exec_handler_ptr->loop_hz_);

  /* Control ex*/
  auto pub = nh->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/robot_controller/joint_trajectory", 10);

  // get robot description
  auto robot_param = rclcpp::Parameter();
  nh->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  nh->get_parameter("robot_description", robot_param);
  auto robot_description = robot_param.as_string();

  //UPDATE URDF POS THROUGH CONTROLLER
  // create kinematic chain
  /*KDL::Tree robot_tree;
  KDL::Chain chain;
  kdl_parser::treeFromString(robot_description, robot_tree);
  robot_tree.getChain("base_link", "ee_link", chain);

  auto joint_positions = KDL::JntArray(chain.getNrOfJoints());
  auto joint_velocities = KDL::JntArray(chain.getNrOfJoints());
  auto twist = KDL::Twist();

  auto ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = nh->now();
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
  rclcpp::Rate l_rate(loop_r);
  for (int i = 0; i < trajectory_len; i++)
  {
    
    double t = i;
    twist.vel.x(2 * 0.3 * cos(2 * M_PI * t / trajectory_len));
    twist.vel.y(-0.3 * sin(2 * M_PI * t / trajectory_len));


    ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);


    std::memcpy(
      trajectory_point_msg.positions.data(), joint_positions.data.data(),
      trajectory_point_msg.positions.size() * sizeof(double));
    std::memcpy(
      trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      trajectory_point_msg.velocities.size() * sizeof(double));


    joint_positions.data += joint_velocities.data * dt;


    trajectory_point_msg.time_from_start.sec = i / loop_r;
    trajectory_point_msg.time_from_start.nanosec = static_cast<int>(
      1E9 / loop_r *
      static_cast<double>(t - loop_r * (i / loop_r)));  

    trajectory_msg.points.push_back(trajectory_point_msg);
  }*/ /* Control ex*/

  while (rclcpp::ok())
  {
    c_exec_handler_ptr->update();

    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }
  return 0;
}
