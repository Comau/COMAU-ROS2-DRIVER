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
int exitParam;
char robot_status_;

void signalHandler(int signum) 
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("execute_trajectory_handler_node")," Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  bool connectionEstablished = 0;

  //boost::shared_ptr<action_cpp::JointTrajectoryActionClient> c_exec_traj_handler_ptr;
  nh = std::make_shared<rclcpp::Node>("execute_trajectory_handler_node");
  nh->declare_parameter("exit",0);
  nh->declare_parameter("start_traj_action",0);
  rclcpp::Rate loop_rate(500);
  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);
  // Create the hardware interface
  c_exec_handler_ptr.reset(new trajectory_handler::TrajectoryHandler(nh));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), " New Session:");
  if (!c_exec_handler_ptr->init()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("trajectory_handler"), " Could not correctly initialize robot. Exiting");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), " HW interface initialized");
  /*auto trajectory_goal = comau_msgs::action::ExecuteJointTrajectory::Goal();
  trajectory_goal.trajectory = {0,0,-1.57,0,0,0};*/
  while (rclcpp::ok())
  {
    exitParam = nh->get_parameter("exit").as_int();
    startTrajAction = nh->get_parameter("start_traj_action").as_int();

    if(connectionEstablished == 0)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), " Connection Established...");
      connectionEstablished = 1;
    }
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
    c_exec_handler_ptr->read();
    robot_status_ = c_exec_handler_ptr->publishRobotStatus();
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
    if ((robot_status_ == 'T') || (robot_status_ == 'C') || (robot_status_ == 'R') || (robot_status_ == 'M') ||
        (robot_status_ == 'I') || (robot_status_ == 'P') || (robot_status_ == 'S') || (robot_status_ == 'E') )
    {
      c_exec_handler_ptr->execute_joints_handler_ptr->set_status(robot_status_);
      c_exec_handler_ptr->execute_joints_handler_ptr->set_allow_async(true);
      c_exec_handler_ptr->execute_cartesian_handler_ptr->set_status(robot_status_);
      c_exec_handler_ptr->execute_cartesian_handler_ptr->set_allow_async(true);

      c_exec_handler_ptr->update();
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("trajectory_handler"), "Invalid state msg: " << robot_status_);
    }
    /* TEST RCLCPP_INFO_STREAM(rclcpp::get_logger("trajectory_handler"), " RobotStatus: [%s]" << c_exec_handler_ptr->robot_status_);*/
    if(startTrajAction == 1)
    {
      /*c_exec_traj_handler_ptr.reset(new action_cpp::JointTrajectoryActionClient());*/
      startTrajAction = 0;
      rclcpp::Parameter set_startTrajAction("start_traj_action", startTrajAction);
      nh->set_parameter(set_startTrajAction);

      c_exec_handler_ptr->sendCartTraj();
    }

    if(exitParam == 1) {
      
      rclcpp::shutdown();
    }
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }
  return 0;
}
