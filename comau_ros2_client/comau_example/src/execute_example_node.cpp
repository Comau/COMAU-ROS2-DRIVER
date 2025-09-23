/**
 * @file execute_example_node.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes example of trajectories
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
#include "comau_example/ex_trj_handler.hpp"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;
using namespace comau_action_handlers;

boost::shared_ptr<ex_trj_handler::TrajectoryHandler> c_exec_handler_ptr;

std::shared_ptr<rclcpp::Node> nh;

int startTrajAction = 0;

void signalHandler(int signum) 
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("execute_example_node")," Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  nh = std::make_shared<rclcpp::Node>("execute_example_node");

  nh->declare_parameter("cart_demo_example",0);
  
  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Create the interface
  c_exec_handler_ptr.reset(new ex_trj_handler::TrajectoryHandler(nh));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ex_trj_handler"), "New Session:");

  rclcpp::Rate loop_rate(1);
  if (!c_exec_handler_ptr->init()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ex_trj_handler"), "Could not correctly initialize robot. Exiting");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ex_trj_handler"), "Example interface initialized");

  int startROS2Traj;
  startROS2Traj = 0;
  nh->declare_parameter("joints_demo_example",0);

  while (rclcpp::ok())
  {
    startTrajAction = nh->get_parameter("cart_demo_example").as_int();

    startROS2Traj = nh->get_parameter("joints_demo_example").as_int();
    if(startROS2Traj == 1)
    {
      startROS2Traj = 0;
      rclcpp::Parameter set_startROS2Traj("joints_demo_example", startROS2Traj);
      nh->set_parameter(set_startROS2Traj);

      //pub->publish(trajectory_msg);
      c_exec_handler_ptr->sendJntTraj();
    }

    if(startTrajAction == 1)
    {
      startTrajAction = 0;
      rclcpp::Parameter set_startTrajAction("cart_demo_example", startTrajAction);
      nh->set_parameter(set_startTrajAction);

      c_exec_handler_ptr->sendCartTraj();
    }

    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }
  return 0;
}
