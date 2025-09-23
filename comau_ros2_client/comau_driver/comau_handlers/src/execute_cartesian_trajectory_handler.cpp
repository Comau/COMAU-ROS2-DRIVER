/**
 * @file execute_cartesian_trajectory_handler.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the cartesian trajectory action
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_handlers/execute_cartesian_trajectory_handler.hpp"

using namespace std::chrono_literals;

namespace comau_action_handlers {

ExecuteCartesianTrajectoryHandler::ExecuteCartesianTrajectoryHandler(
    const rclcpp::Node::SharedPtr &nh, const rclcpp::Node::SharedPtr &nh_local, const std::string &name,
    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)), robot_ptr_(robot_ptr) {}

ExecuteCartesianTrajectoryHandler::~ExecuteCartesianTrajectoryHandler() {
  //as_ptr_.reset();
  action_server_.reset();
  if (action_active_) {
    result_->action_result.success = false;
    result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
    result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
    //as_ptr_->setPreempted(result_);
    goal_handle_->canceled(result_);
  }
}

bool ExecuteCartesianTrajectoryHandler::initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server) {
  use_state_server_ = use_state_server;
  use_robot_server_ = use_robot_server;
  use_arm1_server_ = use_arm1_server;

  using namespace std::placeholders;
  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), "Starting up the ExecuteCartesianTrajectoryActionServer ...  ");
  this->action_server_ = rclcpp_action::create_server<comau_msgs::action::ExecuteCartesianTrajectory>(nh_,
                                                                                                  action_name_,
                                                                                                  std::bind(&ExecuteCartesianTrajectoryHandler::handle_goal, this, _1, _2),
                                                                                                  std::bind(&ExecuteCartesianTrajectoryHandler::handle_cancel, this, _1),
                                                                                                  std::bind(&ExecuteCartesianTrajectoryHandler::handle_accepted, this, _1));
  /*try {
    as_ptr_.reset(new ExecuteCartesianTrajectoryActionServer(
        nh_, action_name_, boost::bind(&ExecuteCartesianTrajectoryHandler::executeCallback, this, _1), false));
    as_ptr_->start();
  } catch (...) {
    ROS_ERROR_STREAM("[" << action_name_ << "]"
                         << "ExecuteCartesianTrajectoryActionServer cannot not start.");
    return false;
  }*/

  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), "Ready to receive goals!  ");
  return true;
}

rclcpp_action::GoalResponse ExecuteCartesianTrajectoryHandler::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const comau_msgs::action::ExecuteCartesianTrajectory::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger(action_name_), "Received goal request with cartesian trajectory ");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecuteCartesianTrajectoryHandler::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger(action_name_), "Received request to cancel goal-trajectory");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExecuteCartesianTrajectoryHandler::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&ExecuteCartesianTrajectoryHandler::executeCallback, this, _1), goal_handle}.detach();
}

geometry_msgs::msg::PoseStamped ExecuteCartesianTrajectoryHandler::changePoseFrame(const std::string &target_frame,
                                                   const geometry_msgs::msg::PoseStamped &goal_pose) {
  //tf2_ros::Buffer br;
  std::shared_ptr<tf2_ros::Buffer> br;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  br = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*br);
  tf2_ros::TransformListener tf2_listener(*br);
  //tf2_ros::TransformListener tf2_listener(br);
  br->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped transformed_pose;

  try {
    rclcpp::Time now = nh_->get_clock()->now();
    transform = br->lookupTransform(target_frame, goal_pose.header.frame_id, now,1s);//, ros::Duration(1.0));
    tf2::doTransform(goal_pose, transformed_pose, transform);
    RCLCPP_WARN_STREAM(rclcpp::get_logger("ChangePoseFrame"),"Pose Transformed!");
    return transformed_pose;
  } catch (const tf2::LookupException &e) {
    RCLCPP_ERROR(rclcpp::get_logger(action_name_),"[%s] %s", action_name_.c_str(), e.what());
    transformed_pose.header.frame_id = "ee_link";//"tool_controller";
    return transformed_pose;
  }
}

trajectoryf_t ExecuteCartesianTrajectoryHandler::parseCartesianTrajectoryGoal(
    const std::shared_ptr<const comau_msgs::action::ExecuteCartesianTrajectory::Goal> goal) {
  trajectoryf_t pose_traj;

  for (comau_msgs::msg::CartesianPoseStamped cart_pose : goal->trajectory)
  {
    comau_tcp_interface::utils::cart_traj_node comau_node;
    /*if (cart_pose.header.frame_id != "") {
      // construct tf pose from cart pose
      geometry_msgs::msg::PoseStamped pose;
      tf2::Quaternion q;
      q.setRPY(cart_pose.roll, cart_pose.pitch, cart_pose.yaw);
      q.normalize();
      pose.header = cart_pose.header;
      pose.pose.position.x = cart_pose.x;
      pose.pose.position.y = cart_pose.y;
      pose.pose.position.z = cart_pose.z;
      pose.pose.orientation.x = q[0];
      pose.pose.orientation.y = q[1];
      pose.pose.orientation.z = q[2];
      pose.pose.orientation.w = q[3];
      // Transform the pose relative on existing TF frame (if frame not equal to pass)
      geometry_msgs::msg::PoseStamped transformed_pose;
      transformed_pose = ExecuteCartesianTrajectoryHandler::changePoseFrame("base_link", pose);
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_),"transformed_pose| "<< "X: " << transformed_pose.pose.position.x * 1000.0 << " Y: " << transformed_pose.pose.position.y * 1000.0 << " Z: " << transformed_pose.pose.position.z * 1000.0);
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = (transformed_pose.pose.position.x * 1000.0);
      pose_values_array.at(1) = (transformed_pose.pose.position.y * 1000.0);
      pose_values_array.at(2) = (transformed_pose.pose.position.z * 1000.0);
      // Revert back to euler
      // POS_SET_RPY IN PDL
      tf2::Quaternion q_transformed;
      q_transformed[0] = transformed_pose.pose.orientation.x;
      q_transformed[1] = transformed_pose.pose.orientation.y;
      q_transformed[2] = transformed_pose.pose.orientation.z;
      q_transformed[3] = transformed_pose.pose.orientation.w;
      q_transformed.normalize();
      double roll, pitch, yaw;
      tf2::Matrix3x3 m(q_transformed);
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_),"(Rad) Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw);
      pose_values_array.at(3) = static_cast<float>(roll * 180.0 / M_PI);
      pose_values_array.at(4) = static_cast<float>(pitch * 180.0 / M_PI);
      pose_values_array.at(5) = static_cast<float>(yaw * 180.0 / M_PI);
      RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_),"(Degree) Roll: " << pose_values_array.at(3) << " Pitch: " << pose_values_array.at(4) << " Yaw: " << pose_values_array.at(5));
      // pose_traj.push_back(pose_values_array);
      comau_node.pose = pose_values_array;
    } else {*/
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = static_cast<float>(cart_pose.x * 1000.0);
      pose_values_array.at(1) = static_cast<float>(cart_pose.y * 1000.0);
      pose_values_array.at(2) = static_cast<float>(cart_pose.z * 1000.0);
      // euler angles
      pose_values_array.at(3) = static_cast<float>(cart_pose.roll * 180.0 / M_PI);
      pose_values_array.at(4) = static_cast<float>(cart_pose.pitch * 180.0 / M_PI);
      pose_values_array.at(5) = static_cast<float>(cart_pose.yaw * 180.0 / M_PI);
      comau_node.pose = pose_values_array;
    //}
    RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_),"comau_node.pose: [x,y,z,R,P,Y] <-> [" << comau_node.pose.at(0) << ", " << comau_node.pose.at(1) << ", " << comau_node.pose.at(2) << ", " << comau_node.pose.at(3) << ", " << comau_node.pose.at(4) << ", " << comau_node.pose.at(5) << "]");
    if (cart_pose.lin_vel)
      comau_node.lin_vel   = cart_pose.lin_vel;
    else
    {
      comau_node.lin_vel   = robot_ptr_->getDefaultLinVel();
      RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_),"Linear velocity is set as default value: " << comau_node.lin_vel);
    }

    if (cart_pose.seg_ovr)
      comau_node.seg_ovr   = cart_pose.seg_ovr;
    else
      comau_node.seg_ovr   = 100;
    
    std::string type = boost::to_upper_copy<std::string>(cart_pose.move_type);
    if (type.compare("JOINT") == 0)
    {
      comau_node.move_type = comau_driver::MoveType::JOINT;
    }
    else if (type.compare("LINEAR") == 0)
    {
      comau_node.move_type = comau_driver::MoveType::LINEAR;
    }
    else if (type.compare("CIRCULAR") == 0)
    {
      comau_node.move_type = comau_driver::MoveType::CIRCULAR;  
    }
    else if (type.compare("SEG_VIA") == 0)
    {
      comau_node.move_type = comau_driver::MoveType::SEG_VIA;  
    }
    else
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(action_name_),"Unknown Move Type: " << type << ". JOINT type is set as default value.");
      comau_node.move_type = comau_driver::MoveType::JOINT;
    }
    pose_traj.push_back(comau_node);
  }

  return pose_traj;
}

void ExecuteCartesianTrajectoryHandler::executeCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle) {
  RCLCPP_INFO(rclcpp::get_logger(action_name_), "Executing goal");
  double start_time = rclcpp::Clock{RCL_ROS_TIME}.now().nanoseconds();//double start_time = ros::Time::now().toNSec() / 1e-6; // to convert nanoseconds to milliseconds
  action_active_ = false;
  bool pub_moving = false;
  goal_handle_ = goal_handle;
  feedback_ = std::make_shared<comau_msgs::action::ExecuteCartesianTrajectory::Feedback>();
  result_   = std::make_shared<comau_msgs::action::ExecuteCartesianTrajectory::Result>();
  if (robot_status_ == RobotStatus::READY && allow_async_) {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(rclcpp::get_logger(action_name_),"[%s]: Parsing Cartesian trajectory", action_name_.c_str());
    goal_cartesian_trajectory_ = ExecuteCartesianTrajectoryHandler::parseCartesianTrajectoryGoal(goal);
    if (use_arm1_server_)
      robot_ptr_->writeTrajectoryCommand(goal_cartesian_trajectory_, comau_driver::ControlMode::MODE_CARTESIAN_TRAJECTORY);
    action_active_ = true;
    RCLCPP_INFO(rclcpp::get_logger(action_name_),"[%s]: Received trajectory sended for execution", action_name_.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger(action_name_),"[%s]: Robot is not in READY status. We are stopping - resetting", action_name_.c_str());
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
      if (use_state_server_)
        result_->action_result.success = false;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
      if (use_robot_server_)
        robot_ptr_->cancelMotionPDL();
      goal_handle->canceled(result_);
      pub_moving = false;
      return;
    } else if (robot_status_ == RobotStatus::SUCCEEDED) { // SUCCEEDED
      RCLCPP_INFO(rclcpp::get_logger(action_name_), "[%s]: Trajectory execution Succeeded", action_name_.c_str());

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
      /*
      while (robot_status_ == RobotStatus::SUCCEEDED)
      {
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.002).to_chrono<std::chrono::nanoseconds>());
      }
      goal_handle->succeed(result_);*/
      pub_moving = false;
      return;
    } else if (robot_status_ == RobotStatus::ERROR) { // ERROR
      //ROS_ERROR("[%s]: Unexpected error, closing action server", action_name_.c_str());
      //
      //result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      //result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      //result_->action_result.success = false;
      ///*
      //if (use_robot_server_)
      //  robot_ptr_->resetPDL();
      //*/
      //goal_handle->abort(result_);
      //
      //return;
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

void ExecuteCartesianTrajectoryHandler::set_status(char &status) {
  robot_status_ = status;
}
void ExecuteCartesianTrajectoryHandler::set_allow_async(const bool &allow_async) {
  allow_async_ = allow_async;
}

} // namespace comau_action_handlers