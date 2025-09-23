/**
 * @file execute_joint_trajectory_handler.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the joint trajectory action
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_handlers/execute_joint_trajectory_handler.hpp"

using namespace std::chrono_literals;
using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_action_handlers {

ExecuteJointTrajectoryHandler::ExecuteJointTrajectoryHandler(
    const rclcpp::Node::SharedPtr &nh, const rclcpp::Node::SharedPtr &nh_local, const std::string &name,
    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)), robot_ptr_(robot_ptr) {}

ExecuteJointTrajectoryHandler::~ExecuteJointTrajectoryHandler() {
  //as_ptr_.reset();
  action_server_.reset();
  urdf_model_ptr_.reset();
  if (action_active_) {
    result_->action_result.success = false;
    result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
    result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
    //as_ptr_->setPreempted(result_);
    goal_handle_->canceled(result_);
  }
}

bool ExecuteJointTrajectoryHandler::initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server) {
  use_state_server_ = use_state_server;
  use_robot_server_ = use_robot_server;
  use_arm1_server_ = use_arm1_server;

  urdf_number_of_joints_ = 0;

  if (!loadJointLimits(nh_, "robot_description")) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(action_name_), "Error at loadJointLimits function");
    return false;
  } /* ANDY */

  using namespace std::placeholders;
  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), "Starting up the ExecuteJointTrajectoryActionServer ...  ");
  this->action_server_ = rclcpp_action::create_server<comau_msgs::action::ExecuteJointTrajectory>(nh_,
                                                                                                  action_name_,
                                                                                                  std::bind(&ExecuteJointTrajectoryHandler::handle_goal, this, _1, _2),
                                                                                                  std::bind(&ExecuteJointTrajectoryHandler::handle_cancel, this, _1),
                                                                                                  std::bind(&ExecuteJointTrajectoryHandler::handle_accepted, this, _1));

  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), "Ready to receive goals!  ");
  return true;
}

rclcpp_action::GoalResponse ExecuteJointTrajectoryHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const comau_msgs::action::ExecuteJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger(action_name_), "Received goal request with joint trajectory ");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecuteJointTrajectoryHandler::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteJointTrajectory>> &goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger(action_name_), "Received request to cancel goal-trajectory");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecuteJointTrajectoryHandler::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteJointTrajectory>> &goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&ExecuteJointTrajectoryHandler::executeCallback, this, _1), goal_handle}.detach();
}

bool ExecuteJointTrajectoryHandler::loadJointLimits(const rclcpp::Node::SharedPtr &nh, std::string param_name) {
  std::string urdf_string;
  urdf_model_ptr_.reset(new urdf::Model());

  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh, "robot_state_publisher");
  while (!parameters_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger(action_name_), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
      RCLCPP_WARN(rclcpp::get_logger(action_name_), "Service not available, waiting again...");
  }

  // Search and wait for robot_description on param server
  while (urdf_string.empty() && rclcpp::ok()) {

    auto parameters = parameters_client_->get_parameters({ param_name });
    for (auto& parameter : parameters)
    {
      if (parameter.get_name() ==  param_name)
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Found URDF model on the ROS "
                             "param server at location: " << nh->get_namespace() << param_name);
        urdf_string = parameter.value_to_string();
        break;
      } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Waiting for model URDF on the ROS "
                             "param server at location: " << nh->get_namespace() << param_name);
      }
    }
    /*std::string search_param_name;
    urdf_string = nh->get_parameter(param_name).as_string();
    RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Found URDF model on the ROS "
                             "param server at location: "
                          << nh->get_namespace() << search_param_name);
    if (nh->search(param_name,search_param_name)){//nh.searchParam(param_name, search_param_name)) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Found URDF model on the ROS "
                             "param server at location: "
                          << nh->get_namespace() << search_param_name);
      urdf_string = nh->get_parameter(search_param_name).as_string();
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Waiting for model URDF on the ROS "
                             "param server at location: "
                          << nh->get_namespace() << param_name);
    }*/
    usleep(100000);
  }

  if (!urdf_model_ptr_->initString(urdf_string)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(action_name_), " Unable to load URDF model with param string : " << urdf_string);
    return false;
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Received URDF from param server with Name: " << urdf_model_ptr_->getName());
  }

  // Get limits from URDF
  if (!urdf_model_ptr_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(action_name_), " No URDF model loaded, unable to get joint limits");
    return false;
  }

  // Limits datastructures
  urdf::JointLimits joint_limits; // Position
  std::map<std::string, urdf::JointSharedPtr>::iterator jointIter;
  std::map<std::string, urdf::JointLimits> jointLimits;
  for (jointIter = urdf_model_ptr_->joints_.begin();jointIter != urdf_model_ptr_->joints_.end(); jointIter++)
  {
    
    urdf_number_of_joints_ += 1;
    // ANDY RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Joint " << jointLimits.lower);
    /*joint_limits::JointLimits joint_limits;
    if (!joint.second) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(action_name_), " URDF joint not found " << joint.first);
      return false;
    }
    std::string j_limits = joint.first.c_str();
    if (joint_limits::get_joint_limits(j_limits, nh, joint_limits)) 
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " " << joint.first << " has URDF position limits ["
                          << joint_limits.min_position << ", " << joint_limits.max_position << "]");
      joint_names_.push_back(joint.first);
      
      joint_limits.min_position += std::numeric_limits<double>::epsilon();
      joint_limits.max_position -= std::numeric_limits<double>::epsilon();

      joint_position_lower_limits_.push_back(joint_limits.min_position);
      joint_position_upper_limits_.push_back(joint_limits.max_position);
    }
    */
  }

  urdf_number_of_joints_ -= 1;

  return true;
}

joint_trajectoryf_t
ExecuteJointTrajectoryHandler::parseJointTrajectoryGoal(const std::shared_ptr<const comau_msgs::action::ExecuteJointTrajectory::Goal> goal) {
  joint_trajectoryf_t joint_traj;
  RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_), "Joint Trajectory");
  for (comau_msgs::msg::JointPose joints_goal : goal->trajectory)
  {

    /* The goal must have a dimension equal to the configured joints  */
    if (joints_goal.positions.size() < robot_ptr_->num_cmd_joints_)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_),"Some command joint poses are missing, check the goal dimension.");
      valid_goal_ = false;
      return joint_traj;
    }
  
    comau_tcp_interface::utils::joint_traj_node joint_node;
    vector10f_t joint_values_array;
    for (size_t i = 0; i < joints_goal.positions.size(); i++) 
    {
      // Convert RADS to DEGREES
      switch (robot_ptr_->jnt_cmd_type_.at(i))
      {
      case 0:
        //ROS_INFO_STREAM("Joint_"<< i+1 << ": " << robot_ptr_->jnt_cmd_type_[i] << " revolute.");
        std::cout << "joints_goal.positions.at(" << i << ") [deg] = " << joints_goal.positions.at(i) * 180 / M_PI << std::endl;
        joint_values_array.at(i) = (static_cast<float>(joints_goal.positions.at(i) * 180 / M_PI));
        break;
      case 1:
        //ROS_INFO_STREAM("Joint_"<< i+1 << ": " << robot_ptr_->jnt_cmd_type_[i] << " linear.");
        std::cout << "joints_goal.positions.at(" << i << ") [mm] = " << joints_goal.positions.at(i) * 1000 << std::endl;
        joint_values_array.at(i) =(static_cast<float>(joints_goal.positions.at(i) * 1000));
        break;
      default:  /* No error */
        break;
      }
    }

    // Check for joint limits
    if (!validateJointLimits(joint_values_array))
      return joint_traj;
    for(size_t i = 0;i < joint_values_array.size();i++){
      joint_node.pose = joint_values_array;
    }
    
    // Set the segment override
    if (joints_goal.seg_ovr)
      joint_node.seg_ovr   = joints_goal.seg_ovr;
    else
      joint_node.seg_ovr   = 100;
    
    // Set the move type
    std::string type = boost::to_upper_copy<std::string>(joints_goal.move_type);
    if (type.compare("JOINT") == 0)
    {
      joint_node.move_type = comau_driver::MoveType::JOINT;
    }
    else if (type.compare("LINEAR") == 0)
    {
      joint_node.move_type = comau_driver::MoveType::LINEAR;
    }
    else if (type.compare("CIRCULAR") == 0)
    {
      joint_node.move_type = comau_driver::MoveType::CIRCULAR;  
    }
    else if (type.compare("SEG_VIA") == 0)
    {
      joint_node.move_type = comau_driver::MoveType::SEG_VIA;  
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_), " Unknown Move Type: " << type << ". JOINT type is set as default value.");
      joint_node.move_type = comau_driver::MoveType::JOINT;
    }

    joint_traj.push_back(joint_node);
  }
  return joint_traj;
}


bool ExecuteJointTrajectoryHandler::validateJointLimits(vector10f_t joint_values_array) {

  for (size_t joint_id = 0; joint_id < joint_values_array.size(); joint_id++) 
  {
    valid_goal_ = true;

    switch (robot_ptr_->jnt_cmd_type_.at(joint_id))
    {
      case 0:
        if (joint_values_array.at(joint_id) * (M_PI / 180) > 10000 ||/* ANDY joint_position_upper_limits_.at(joint_id) || */
            joint_values_array.at(joint_id) * (M_PI / 180) < -10000) {/* ANDY joint_position_lower_limits_.at(joint_id)) {*/
          valid_goal_ = false;
          RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_)," Trajectory contains an out of limits goal at joint " << joint_id + 1
                              << ". Please check robot limits.");
          RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_)," Joint " << joint_id + 1 << " : " << joint_values_array[joint_id] * (M_PI / 180)
                              << " does not belong to interval [" << joint_position_lower_limits_[joint_id] << ", " << joint_position_upper_limits_[joint_id] << "]");
          return valid_goal_;
        }
        break;
      case 1:
        if (joint_values_array.at(joint_id) * (0.001) > 10000 ||/* ANDY joint_position_upper_limits_.at(joint_id) || */
            joint_values_array.at(joint_id) * (0.001) < -10000) {/* ANDY joint_position_lower_limits_.at(joint_id)) {*/
          valid_goal_ = false;
          RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_)," Trajectory contains an out of limits goal at joint " << joint_id + 1
                              << ". Please check robot limits.");
          RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_)," Joint " << joint_id + 1 << " : " << joint_values_array[joint_id] * (0.001)
                              << " does not belong to interval [" << joint_position_lower_limits_[joint_id] << ", " << joint_position_upper_limits_[joint_id] << "]");
          return valid_goal_;
        }
        break;
      default: /* No error */
        break;
    }
  }
  
  return valid_goal_;
}

void ExecuteJointTrajectoryHandler::executeCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteJointTrajectory>> &goal_handle) {
  RCLCPP_INFO(rclcpp::get_logger(action_name_), "Executing goal");
  double start_time = rclcpp::Clock{RCL_ROS_TIME}.now().nanoseconds();//double start_time = ros::Time::now().toNSec() / 1e-6; // to convert nanoseconds to milliseconds
  action_active_ = false;
  bool pub_moving = false;
  goal_handle_ = goal_handle;
  feedback_ = std::make_shared<comau_msgs::action::ExecuteJointTrajectory::Feedback>();
  result_   = std::make_shared<comau_msgs::action::ExecuteJointTrajectory::Result>();

  if (robot_status_ == RobotStatus::READY && allow_async_) {
    const auto goal = goal_handle->get_goal();
    goal_joint_trajectory_ = parseJointTrajectoryGoal(goal);
    if (valid_goal_) {
      if (use_arm1_server_)
        robot_ptr_->writeJointTrajectoryCommand(goal_joint_trajectory_, comau_driver::ControlMode::MODE_JOINT_TRAJECTORY);
      action_active_ = true;
      RCLCPP_INFO(rclcpp::get_logger(action_name_),"[%s]: Received trajectory sended for execution", action_name_.c_str());
    } else {
      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.success = false;
      goal_handle->abort(result_);
      return;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger(action_name_),": Robot is not in READY status. We are stopping - resetting");
    /*
    if (use_robot_server_ && allow_async_)
      robot_ptr_->resetPDL();
    */
    result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
    result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
    result_->action_result.success = false;

    goal_handle->abort(result_);
    return;
  }

  while (action_active_) {

    if (goal_handle->is_canceling() || !rclcpp::ok()) { // CANCELLED
      RCLCPP_INFO(rclcpp::get_logger(action_name_), "[%s]: Trajectory execution Preempted", action_name_.c_str());
      result_->action_result.success = false;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
      if (use_robot_server_)
        robot_ptr_->cancelMotionPDL();
      goal_handle->canceled(result_);
      pub_moving = false;
      return;
    } else if (robot_status_ == RobotStatus::SUCCEEDED) { // SUCCEEDED
      RCLCPP_INFO(rclcpp::get_logger(action_name_), " [%s]: Trajectory execution Succeeded", action_name_.c_str());

      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::SUCCESS;
      result_->action_result.success = true;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      feedback_->action_feedback.success = result_->action_result.success;
      /* After motion is correctly executed, the server clean the traj then the reset cmd is not necessary
      if (use_robot_server_)
        robot_ptr_->resetPDL();
      */
      
      goal_handle->succeed(result_);
      goal_handle->publish_feedback(feedback_);
      /*while (robot_status_ == RobotStatus::SUCCEEDED)
      {
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.002).to_chrono<std::chrono::nanoseconds>());
      }*/
      
      pub_moving = false;
      return;
    } else if (robot_status_ == RobotStatus::ERROR) {
      // ERROR
      // ROS_ERROR("[%s]: Unexpected error, closing action server", action_name_.c_str());
      // result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      // result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      // result_.action_result.success = false;
      // /*
      // if (use_robot_server_)
      //   robot_ptr_->resetPDL();
      // */
      // as_ptr_->setAborted(result_);
      // return;
    } else if (robot_status_ == RobotStatus::TERMINATE) { // TERMINATE
      RCLCPP_INFO(rclcpp::get_logger(action_name_), "[%s]: Action terminated, canceling Trajectory execution", action_name_.c_str());

      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.success = false;
      goal_handle->abort(result_);
      pub_moving = false;
      return;
    } else if (robot_status_ == RobotStatus::MOVING) { // MOVING
      if(pub_moving == false)
      {
        // MOVING
        RCLCPP_DEBUG(rclcpp::get_logger(action_name_), "[%s]: Trajectory execution is active", action_name_.c_str());
        feedback_->action_feedback.success = false;
        feedback_->action_feedback.millis_passed = uint((rclcpp::Clock{RCL_ROS_TIME}.now().nanoseconds() / 1e-6) - start_time);
        feedback_->action_feedback.status = robot_status_;
        goal_handle->publish_feedback(feedback_);
      }
      pub_moving = true;
    }
    
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.001).to_chrono<std::chrono::nanoseconds>());
  }
  pub_moving = false;
  goal_handle_ = goal_handle;
}

void ExecuteJointTrajectoryHandler::set_status(char &status) {
  robot_status_ = status;
}
void ExecuteJointTrajectoryHandler::set_allow_async(const bool &allow_async) {
  allow_async_ = allow_async;
}

} // namespace comau_action_handlers

//RCLCPP_COMPONENTS_REGISTER_NODE(comau_action_handlers::ExecuteJointTrajectoryHandler)