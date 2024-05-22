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

int startTrajAction = 0;
int exitParam;
char robot_status_;

//namespace action_cpp
//{
//class JointTrajectoryActionClient : public rclcpp::Node
//{
//public:
//  using ExecuteJointTrajectory           = comau_msgs::action::ExecuteJointTrajectory;              //Fibonacci
//  using GoalHandleExecuteJointTrajectory = rclcpp_action::ClientGoalHandle<ExecuteJointTrajectory>; //GoalHandleFibonacci
//
//  explicit JointTrajectoryActionClient()
//  : Node("jnt_trj_action_client_node")
//  {
//    this->client_ptr_ = rclcpp_action::create_client<ExecuteJointTrajectory>(this, "execute_joint_trajectory_handler");
//    RCLCPP_INFO(this->get_logger(), "Action server waiting...");
//    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&JointTrajectoryActionClient::send_goal, this));
//  }
//
//  void send_goal()
//  {
//    using namespace std::placeholders;
//
//    this->timer_->cancel();
//
//    if (!this->client_ptr_->wait_for_action_server()) {
//      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//      rclcpp::shutdown();
//    }
//
//    auto goal_msg = ExecuteJointTrajectory::Goal();
//    comau_msgs::msg::JointPose joint_pos;
//    joint_pos.positions = {0.0, 0.0, -1.57, 0.0, 1.5707, 0.0};
//
//    // populate the goal
//    goal_msg.trajectory.push_back(joint_pos);
//
//    RCLCPP_INFO(this->get_logger(), "Sending goal");
//
//    auto send_goal_options = rclcpp_action::Client<ExecuteJointTrajectory>::SendGoalOptions();
//    send_goal_options.goal_response_callback =
//      std::bind(&JointTrajectoryActionClient::goal_response_callback, this, _1);
//    send_goal_options.feedback_callback =
//      std::bind(&JointTrajectoryActionClient::feedback_callback, this, _1, _2);
//    send_goal_options.result_callback =
//      std::bind(&JointTrajectoryActionClient::result_callback, this, _1);
//    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//  }
//
//private:
//  rclcpp_action::Client<ExecuteJointTrajectory>::SharedPtr client_ptr_;
//  rclcpp::TimerBase::SharedPtr timer_;
//
//  void goal_response_callback(const GoalHandleExecuteJointTrajectory::SharedPtr & goal_handle)
//  {
//    if (!goal_handle) {
//      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//    } else {
//      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//    }
//  }
//
//  void feedback_callback(
//    GoalHandleExecuteJointTrajectory::SharedPtr,
//    const std::shared_ptr<const ExecuteJointTrajectory::Feedback> feedback)
//  {
//    /*bool success;
//    success = feedback->action_feedback.success;
//    RCLCPP_INFO(this->get_logger(), "Feedback ", success.c_str);*/
//  }
//
//  void result_callback(const GoalHandleExecuteJointTrajectory::WrappedResult & result)
//  {
//    switch (result.code) {
//      case rclcpp_action::ResultCode::SUCCEEDED:
//        break;
//      case rclcpp_action::ResultCode::ABORTED:
//        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//        return;
//      case rclcpp_action::ResultCode::CANCELED:
//        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//        return;
//      default:
//        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//        return;
//    }
//    //RCLCPP_INFO(this->get_logger(), result.result->action_result.status);
//    rclcpp::shutdown();
//  }
//};  // class JointTrajectoryActionClient
//
//}

void signalHandler(int signum) {

  RCLCPP_WARN_STREAM(rclcpp::get_logger("execute_trajectory_handler_node")," Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  bool connectionEstablished = 0;

  //boost::shared_ptr<action_cpp::JointTrajectoryActionClient> c_exec_traj_handler_ptr;
  std::shared_ptr<rclcpp::NodeOptions> options;
  std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("execute_trajectory_handler_node");
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
      /*c_exec_traj_handler_ptr.reset(new action_cpp::JointTrajectoryActionClient());
      startTrajAction = 0;*/
      rclcpp::Parameter set_startTrajAction("start_traj_action", startTrajAction);
      nh->set_parameter(set_startTrajAction);
    }

    if(exitParam == 1) {
      
      rclcpp::shutdown();
    }
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }
  return 0;
}
