/**
 * @file execute_cartesian_trajectory_handler.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include "comau_handlers/execute_cartesian_trajectory_handler.hpp"

namespace comau_action_handlers {

ExecuteCartesianTrajectoryHandler::ExecuteCartesianTrajectoryHandler(
    const rclcpp::Node::SharedPtr &nh, const rclcpp::Node::SharedPtr &nh_local, const std::string &name,
    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)), robot_ptr_(robot_ptr) {}

ExecuteCartesianTrajectoryHandler::~ExecuteCartesianTrajectoryHandler() {
  //as_ptr_.reset();
  if (action_active_) {
    result_->action_result.success = false;
    result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
    result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
    goal_handle_->canceled(result_);//as_ptr_->setPreempted(result_);
  }
}

bool ExecuteCartesianTrajectoryHandler::initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server) {
  use_state_server_ = use_state_server;
  use_robot_server_ = use_robot_server;
  use_arm1_server_ = use_arm1_server;

  feedback_ = std::make_shared<ExecuteCartesianTrajectory::Feedback>();
  result_ = std::make_shared<ExecuteCartesianTrajectory::Result>();

  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " tarting up the ExecuteCartesianTrajectoryActionServer ...  ");

  using namespace std::placeholders;
  
  this->ExecuteCartesianTrajectoryActionServer = rclcpp_action::create_server<ExecuteCartesianTrajectory>(nh_,
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger(action_name_), " Ready to receive goals!  ");
  return true;
}

geometry_msgs::msg::PoseStamped
ExecuteCartesianTrajectoryHandler::changePoseFrame(const std::string &target_frame,
                                                   const geometry_msgs::msg::PoseStamped &goal_pose) {
  //tf2_ros::Buffer br;
  std::shared_ptr<tf2_ros::TransformListener> tfl{nullptr};
  std::unique_ptr<tf2_ros::Buffer> br;
  br = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
  //br.setUsingDedicatedThread(true); CHECK
  //tf2_ros::TransformListener tf2_listener(br);
  tfl = std::make_shared<tf2_ros::TransformListener>(*br);
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PoseStamped transformed_pose;

  try {
    transform = br->lookupTransform(target_frame, goal_pose.header.frame_id, nh_->get_clock()->now());
    tf2::doTransform(goal_pose, transformed_pose, transform);
    //RCLCPP_INFO_STREAM(nh_->get_logger(action_name_), "Change transform pose to..   " << transformed_pose); CHECK
    return transformed_pose;
  } catch (tf2::LookupException &e) {
    RCLCPP_ERROR(rclcpp::get_logger(action_name_), e.what());
    transformed_pose.header.frame_id = "tool_controller";
    return transformed_pose;
  }
}

trajectoryf_t ExecuteCartesianTrajectoryHandler::parseCartesianTrajectoryGoal(std::shared_ptr<const comau_msgs::action::ExecuteCartesianTrajectory::Goal> goal) {
  trajectoryf_t pose_traj;

  for (comau_msgs::msg::CartesianPoseStamped cart_pose : goal->trajectory)
  {
    comau_tcp_interface::utils::cart_traj_node comau_node;
    if (cart_pose.header.frame_id != "") {
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
      transformed_pose = changePoseFrame("base_link", pose);
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = static_cast<float>(transformed_pose.pose.position.x * 1000.);
      pose_values_array.at(1) = static_cast<float>(transformed_pose.pose.position.y * 1000.);
      pose_values_array.at(2) = static_cast<float>(transformed_pose.pose.position.z * 1000.);
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
      pose_values_array.at(3) = static_cast<float>(roll * 180. / M_PI);
      pose_values_array.at(4) = static_cast<float>(pitch * 180. / M_PI);
      pose_values_array.at(5) = static_cast<float>(yaw * 180. / M_PI);
      // pose_traj.push_back(pose_values_array);
      comau_node.pose = pose_values_array;
    } else {
      vector6f_t pose_values_array;
      // first 3 points correspond to position - PDL wants millimiters
      pose_values_array.at(0) = static_cast<float>(cart_pose.x * 1000.);
      pose_values_array.at(1) = static_cast<float>(cart_pose.y * 1000.);
      pose_values_array.at(2) = static_cast<float>(cart_pose.z * 1000.);
      // euler angles
      pose_values_array.at(3) = static_cast<float>(cart_pose.roll * 180. / M_PI);
      pose_values_array.at(4) = static_cast<float>(cart_pose.pitch * 180. / M_PI);
      pose_values_array.at(5) = static_cast<float>(cart_pose.yaw * 180. / M_PI);
      comau_node.pose = pose_values_array;
    }
    if (cart_pose.lin_vel)
      comau_node.lin_vel   = cart_pose.lin_vel;
    else
    {
      comau_node.lin_vel   = robot_ptr_->getDefaultLinVel();
      RCLCPP_WARN_STREAM(nh_->get_logger()," Linear velocity is set as default value: " << comau_node.lin_vel);
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
      RCLCPP_WARN_STREAM(nh_->get_logger()," Unknown Move Type: " << type << ". JOINT type is set as default value.");
      comau_node.move_type = comau_driver::MoveType::JOINT;
    }
    pose_traj.push_back(comau_node);
  }

  return pose_traj;
}

rclcpp_action::GoalResponse ExecuteCartesianTrajectoryHandler::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecuteCartesianTrajectory::Goal> goal)
{
  RCLCPP_INFO(nh_->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExecuteCartesianTrajectoryHandler::handle_cancel(
  const std::shared_ptr<GoalHandleExecuteCartesianTrajectory> goal_handle)
{
  RCLCPP_INFO(nh_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
 
void ExecuteCartesianTrajectoryHandler::handle_accepted(const std::shared_ptr<GoalHandleExecuteCartesianTrajectory> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&ExecuteCartesianTrajectoryHandler::executeCallback, this, _1), goal_handle_}.detach();
}

void ExecuteCartesianTrajectoryHandler::executeCallback(const std::shared_ptr<GoalHandleExecuteCartesianTrajectory> goal) {
  double start_time = nh_->now().nanoseconds() / 1e-6; // to convert nanoseconds to milliseconds
  action_active_ = false;
  if (robot_status_ == RobotStatus::READY && allow_async_) {
    RCLCPP_INFO(nh_->get_logger(), ": Parsing Cartesian trajectory %s", action_name_.c_str());
    goal_cartesian_trajectory_ = parseCartesianTrajectoryGoal(goal->get_goal());
    if (use_arm1_server_)
      robot_ptr_->writeTrajectoryCommand(goal_cartesian_trajectory_,
                                         comau_driver::ControlMode::MODE_CARTESIAN_TRAJECTORY);
    action_active_ = true;
    RCLCPP_INFO(nh_->get_logger(), ": Received trajectory sended for execution %s", action_name_.c_str());
  } else {
    RCLCPP_WARN(nh_->get_logger(), ": Robot is not in READY status. We are stopping - resetting %s", action_name_.c_str());
    /*
    if (use_robot_server_ && allow_async_)
      robot_ptr_->resetPDL();
    */
    result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
    result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
    result_->action_result.success = false;

    //as_ptr_->setAborted(result_);
    goal->abort(result_);
    return;
  }

  while (action_active_) {

    if (goal->is_canceling() || !rclcpp::ok()) { // CANCELLED as_ptr_->isPreemptRequested() 
      RCLCPP_INFO(nh_->get_logger(), ": Trajectory execution Preempted %s", action_name_.c_str());
      if (use_state_server_)
        result_->action_result.success = false;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::CANCELLED;
      if (use_robot_server_)
        robot_ptr_->cancelMotionPDL();
      //as_ptr_->setPreempted(result_);
      goal->canceled(result_);
      return;
    } else if (robot_status_ == RobotStatus::SUCCEEDED) { // SUCCEEDED
      RCLCPP_INFO(nh_->get_logger(), ": Trajectory execution Succeeded %s", action_name_.c_str());

      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::SUCCESS;
      result_->action_result.success = true;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      /* After motion is correctly executed, the server clean the traj then the reset cmd is not necessary
      if (use_robot_server_)
        robot_ptr_->resetPDL();
      */
      while (robot_status_ == RobotStatus::SUCCEEDED) /* Wait for the READY status to Ack the motion command */
      {
        //ros::Duration(0.002).sleep();
        rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.002).to_chrono<std::chrono::nanoseconds>());
      }
      //as_ptr_->setSucceeded(result_);
      goal->succeed(result_);
      return;
    } else if (robot_status_ == RobotStatus::ERROR) { // ERROR
      //ROS_ERROR("[%s]: Unexpected error, closing action server", action_name_.c_str());
      //
      //result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      //result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
      //result_.action_result.success = false;
      ///*
      //if (use_robot_server_)
      //  robot_ptr_->resetPDL();
      //*/
      //as_ptr_->setAborted(result_);
      //
      //return;
    } else if (robot_status_ == RobotStatus::TERMINATE) { // TERMINATE
      RCLCPP_INFO(nh_->get_logger(), ": Action terminated, canceling Trajectory execution %s", action_name_.c_str());

      result_->action_result.status = comau_msgs::msg::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
      result_->action_result.millis_passed = feedback_->action_feedback.millis_passed;
      result_->action_result.success = false;
      //as_ptr_->setAborted(result_);
      goal->abort(result_);

      return;
    } else if (robot_status_ == RobotStatus::MOVING) { // MOVING

      RCLCPP_DEBUG(nh_->get_logger(), " Trajectory execution is active %s", action_name_.c_str());
      feedback_->action_feedback.millis_passed = uint((nh_->now().nanoseconds() / 1e-6) - start_time);

      //as_ptr_->publishFeedback(feedback_);
      goal->publish_feedback(feedback_);
    }

    rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.001).to_chrono<std::chrono::nanoseconds>());//check
  }
}

void ExecuteCartesianTrajectoryHandler::set_status(char &status) {
  robot_status_ = status;
}
void ExecuteCartesianTrajectoryHandler::set_allow_async(const bool &allow_async) {
  allow_async_ = allow_async;
}

} // namespace comau_action_handlers