/**
 * @file execute_trajectory_handler_node.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "comau_handlers/trajectory_handler.hpp"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;
using namespace comau_action_handlers;

boost::shared_ptr<trajectory_handler::TrajectoryHandler> c_exec_handler_ptr;

std::shared_ptr<rclcpp::Node> nh;

int startTrajAction = 0;

/*TO DO closeComauDriver ANDY */

void signalHandler(int signum) 
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("execute_trajectory_handler_node")," Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  nh = std::make_shared<rclcpp::Node>("execute_trajectory_handler_node");

  nh->declare_parameter("start_traj_action",0);
  nh->declare_parameter("loop_hz", 500.0);
  nh->declare_parameter("cycle_time_error_threshold", 0.025);
  
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

  while (rclcpp::ok())
  {
    startTrajAction = nh->get_parameter("start_traj_action").as_int();

    c_exec_handler_ptr->update();

    if(startTrajAction == 1)
    {
      startTrajAction = 0;
      rclcpp::Parameter set_startTrajAction("start_traj_action", startTrajAction);
      nh->set_parameter(set_startTrajAction);

      c_exec_handler_ptr->sendCartTraj();
    }

    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }
  return 0;
}
