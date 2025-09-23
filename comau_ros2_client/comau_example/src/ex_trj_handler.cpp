/**
 * @file ex_trj_handler.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes example of trajectories
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_example/ex_trj_handler.hpp"

using namespace std::chrono_literals;

namespace ex_trj_handler 
{

using JntTraj           = comau_msgs::action::ExecuteJointTrajectory;
using GoalHandleJntTraj = rclcpp_action::ClientGoalHandle<JntTraj>;
using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;

// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

TrajectoryHandler::TrajectoryHandler(rclcpp::Node::SharedPtr &nh) : name_("ex_trj_handler"), nh_(nh) 
{
  
}

bool TrajectoryHandler::init() {
  
  std::shared_ptr<rclcpp::Node> nh_priv_ = std::make_shared<rclcpp::Node>(nh_->get_name(), name_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "" << name_);
  sleep(1);

  return true;
}

void TrajectoryHandler::sendCartTraj()
{
  RCLCPP_WARN_STREAM(nh_->get_logger(),"Sending Cartesian Trajectory...");
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  client_cb_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->client_ptr_ = rclcpp_action::create_client<comau_msgs::action::ExecuteCartesianTrajectory>(nh_,"execute_cartesian_trajectory_handler");//,client_cb_group_);

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
    this->client_ptr_->async_cancel_all_goals();
  }
  
  TrajectoryHandler::send_goal();
}

void TrajectoryHandler::send_goal()
{
  using namespace std::placeholders;
  RCLCPP_WARN_STREAM(nh_->get_logger(),"send_goal");

  RCLCPP_WARN(nh_->get_logger(), "Action server available");
  comau_msgs::action::ExecuteCartesianTrajectory::Goal goal_msg;
  comau_msgs::msg::CartesianPoseStamped                homeCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                firstCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                secondCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                thirdCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                fourthCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                fifthCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                sixthCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                seventhCartesianPose;
  comau_msgs::msg::CartesianPoseStamped                eighthCartesianPose;

  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  /*[436, 0, 705, 179.909, -0, 179.909]*/
  homeCartesianPose.header           = header;
  homeCartesianPose.x                =  0.436;
  homeCartesianPose.y                =  0.000;
  homeCartesianPose.z                =  0.705;
  homeCartesianPose.roll             =  3.140;
  homeCartesianPose.pitch            =  0.000;
  homeCartesianPose.yaw              =  3.140;
  goal_msg.trajectory.push_back(homeCartesianPose);

  firstCartesianPose.header          = header;
  firstCartesianPose.x               =  0.436;
  firstCartesianPose.y               =  0.143;
  firstCartesianPose.z               =  0.589;
  firstCartesianPose.roll            =  3.140;
  firstCartesianPose.pitch           =  0.000;
  firstCartesianPose.yaw             =  3.140;
  goal_msg.trajectory.push_back(firstCartesianPose);

  secondCartesianPose.header         = header;
  secondCartesianPose.x              =  0.608;
  secondCartesianPose.y              =  0.143;
  secondCartesianPose.z              =  0.589;
  secondCartesianPose.roll           =  3.140;
  secondCartesianPose.pitch          =  0.000;
  secondCartesianPose.yaw            =  3.140;
  goal_msg.trajectory.push_back(secondCartesianPose);

  thirdCartesianPose.header          = header;
  thirdCartesianPose.x               =  0.608;
  thirdCartesianPose.y               = -0.143;
  thirdCartesianPose.z               =  0.589;
  thirdCartesianPose.roll            =  3.140;
  thirdCartesianPose.pitch           =  0.000;
  thirdCartesianPose.yaw             =  3.140;
  goal_msg.trajectory.push_back(thirdCartesianPose);

  fourthCartesianPose.header         = header;
  fourthCartesianPose.x              =  0.436;
  fourthCartesianPose.y              = -0.143;
  fourthCartesianPose.z              =  0.585;
  fourthCartesianPose.roll           =  3.140;
  fourthCartesianPose.pitch          =  0.000;
  fourthCartesianPose.yaw            =  3.140;
  goal_msg.trajectory.push_back(fourthCartesianPose);
  goal_msg.trajectory.push_back(firstCartesianPose);
  fifthCartesianPose.header          = header;
  fifthCartesianPose.x               =  0.436;
  fifthCartesianPose.y               =  0.143;
  fifthCartesianPose.z               =  0.400;
  fifthCartesianPose.roll            =  3.140;
  fifthCartesianPose.pitch           =  0.000;
  fifthCartesianPose.yaw             =  3.140;
  goal_msg.trajectory.push_back(fifthCartesianPose);

  sixthCartesianPose.header          = header;
  sixthCartesianPose.x               =  0.608;
  sixthCartesianPose.y               =  0.143;
  sixthCartesianPose.z               =  0.400;
  sixthCartesianPose.roll            =  3.140;
  sixthCartesianPose.pitch           =  0.000;
  sixthCartesianPose.yaw             =  3.140;
  goal_msg.trajectory.push_back(sixthCartesianPose);

  seventhCartesianPose.header        = header;
  seventhCartesianPose.x             =  0.608;
  seventhCartesianPose.y             = -0.143;
  seventhCartesianPose.z             =  0.400;
  seventhCartesianPose.roll          =  3.140;
  seventhCartesianPose.pitch         =  0.000;
  seventhCartesianPose.yaw           =  3.140;
  goal_msg.trajectory.push_back(seventhCartesianPose);

  eighthCartesianPose.header         = header;
  eighthCartesianPose.x              =  0.436;
  eighthCartesianPose.y              = -0.143;
  eighthCartesianPose.z              =  0.400;
  eighthCartesianPose.roll           =  3.140;
  eighthCartesianPose.pitch          =  0.000;
  eighthCartesianPose.yaw            =  3.140;
  goal_msg.trajectory.push_back(eighthCartesianPose);
  goal_msg.trajectory.push_back(fifthCartesianPose);
  goal_msg.trajectory.push_back(homeCartesianPose);

  RCLCPP_INFO(nh_->get_logger(), "Sending goals");

  auto send_goal_options = rclcpp_action::Client<CartTraj>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TrajectoryHandler::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&TrajectoryHandler::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&TrajectoryHandler::result_callback, this, _1);
  

  auto result_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  /*std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
  if (status == std::future_status::ready) {
    RCLCPP_INFO(nh_->get_logger(), "Received response");
  } else {
    RCLCPP_ERROR(nh_->get_logger(), "Cancel all goals");
    this->client_ptr_->async_cancel_all_goals();
  }*/
}

/* ANDY void TrajectoryHandler::cancel_goal() 
{
  auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
  if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to cancel action server for %s", action_name_.c_str());
  }
}*/

void TrajectoryHandler::goal_response_callback(const GoalHandleCartTraj::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(nh_->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void TrajectoryHandler::feedback_callback(GoalHandleCartTraj::SharedPtr, const std::shared_ptr<const CartTraj::Feedback> feedback)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(),"FEEDBACK:" << feedback->action_feedback.success);
}

void TrajectoryHandler::result_callback(const GoalHandleCartTraj::WrappedResult & result)
{
  result_.code = result.code;
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_WARN_STREAM(nh_->get_logger(),"SUCCEDED!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
      return;
  }
}

void TrajectoryHandler::sendJntTraj()
{
  RCLCPP_WARN_STREAM(nh_->get_logger(),"Sending Joint Trajectory...");

  this->client_jnt_ptr_ = rclcpp_action::create_client<comau_msgs::action::ExecuteJointTrajectory>(nh_,"execute_joint_trajectory_handler");

  if (!this->client_jnt_ptr_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
    this->client_jnt_ptr_->async_cancel_all_goals();
  }
  
  TrajectoryHandler::send_jnt_goal();
}

void TrajectoryHandler::send_jnt_goal()
{
  using namespace std::placeholders;
  RCLCPP_WARN_STREAM(nh_->get_logger(),"send_joint_goal");

  if (!this->client_jnt_ptr_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
  }
  RCLCPP_WARN(nh_->get_logger(), "Action server available");
  comau_msgs::action::ExecuteJointTrajectory::Goal goal_msg;
  comau_msgs::msg::JointPose                       jointPose;

  jointPose.positions.resize(6);
  urdf_command_ = {0.436332, 0.0, -1.5708, 0.0, 1.57, 0.0};
  
  for(size_t si_i = 0; si_i < 6; si_i++)
  {
    if((urdf_command_.at(si_i)*180/M_PI) < 180 && (urdf_command_.at(si_i)*180/M_PI) > -180)
    {
      jointPose.positions[si_i] = urdf_command_.at(si_i);
    }
  }
  goal_msg.trajectory.push_back(jointPose);

  urdf_command_ = {-0.436332, 0.0, -1.5708, 0.0, 1.57, 0.0};
  
  for(size_t si_i = 0; si_i < 6; si_i++)
  {
    if((urdf_command_.at(si_i)*180/M_PI) < 180 && (urdf_command_.at(si_i)*180/M_PI) > -180)
    {
      jointPose.positions[si_i] = urdf_command_.at(si_i);
    }
  }
  goal_msg.trajectory.push_back(jointPose);

  urdf_command_ = {0.0, 0.0, -1.5708, 0.0, 1.57, 0.0};
  
  for(size_t si_i = 0; si_i < 6; si_i++)
  {
    if((urdf_command_.at(si_i)*180/M_PI) < 180 && (urdf_command_.at(si_i)*180/M_PI) > -180)
    {
      jointPose.positions[si_i] = urdf_command_.at(si_i);
    }
  }
  goal_msg.trajectory.push_back(jointPose);

  RCLCPP_INFO(nh_->get_logger(), "Sending joint goal");

  auto send_goal_options = rclcpp_action::Client<JntTraj>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TrajectoryHandler::jnt_goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&TrajectoryHandler::jnt_feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&TrajectoryHandler::jnt_result_callback, this, _1);
  this->client_jnt_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void TrajectoryHandler::jnt_goal_response_callback(const GoalHandleJntTraj::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(nh_->get_logger(), "Joint Goal was rejected by server");
  } else {
    RCLCPP_INFO(nh_->get_logger(), "Joint Goal accepted by server, waiting for result");
  }
}

void TrajectoryHandler::jnt_feedback_callback(GoalHandleJntTraj::SharedPtr, const std::shared_ptr<const JntTraj::Feedback> feedback)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(),"FEEDBACK:" << feedback->action_feedback.success);
}

void TrajectoryHandler::jnt_result_callback(const GoalHandleJntTraj::WrappedResult & result)
{
  result_.code = result.code;
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_WARN_STREAM(nh_->get_logger(),"SUCCEDED!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(nh_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(nh_->get_logger(), "Unknown result code");
      return;
  }
}

} // namespace ex_trj_handler
