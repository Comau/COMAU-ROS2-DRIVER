/**
 * @file trajectory_handler.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include "comau_handlers/trajectory_handler.hpp"

namespace trajectory_handler {

using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;

TrajectoryHandler::TrajectoryHandler(rclcpp::Node::SharedPtr &nh) : name_("execute_trajectory_handler"), nh_(nh) 
{
  
}

bool TrajectoryHandler::init() {
  
  std::shared_ptr<rclcpp::Node> nh_priv_ = std::make_shared<rclcpp::Node>(nh_->get_name(), name_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), " " << name_);
  sleep(3);
  // Read parameters through rclcpp parameter server
  use_state_server_ = true;
  use_robot_server_ = true;
  use_arm1_server_  = true;
  verbose_          = true;

  /*bool initialization_;
  int32_t data_timestamp_;
  char robot_status_;
  uint32_t error_value_;
  int32_t sns_trk_type_;
  uint32_t num_robot_joints_;*/
  stsSelector_       = 0;
  ee_position_       = {0.0,0.0,0.0,0.0,0.0,0.0};
  pins_in_           = {0,0,0,0,0,0};
  pins_state_in_     = {0,0,0,0,0,0};
  pins_out_          = {0,0,0,0,0,0};
  pins_state_out_    = {0,0,0,0,0,0};
  jnt_type_          = {0,0,0,0,0,0};
  joint_position_ = {0.0,0.0,0.0,0.0,0.0,0.0};

  initialization_ = false;
  num_joints_     = 6;

  //nh_->declare_parameter("robot_description", "COMAU_ROBOT"); /**Da togliere*/

  RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_robot")," Robot driver Initialization...");
  // Initialize Robot driver
  try {
    robot_ptr_.reset(new comau_driver::ComauRobot(nh_));
    if (!robot_ptr_->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot")," Failed to initialize robot driver");
      return false;
    }
    bool state;
    state = false;
    bool robot;
    robot = false;
    bool arm;
    arm = false;
    state = robot_ptr_->state_client_ptr_->openStateThread(true,  state);
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
    robot = robot_ptr_->robot_client_ptr_->openRobotThread(true,  robot);
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
    arm = robot_ptr_->arm1_client_ptr_->openHandlerThread(true, arm);
    rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
    if(robot_ptr_->state_client_ptr_->isConnected() && robot_ptr_->robot_client_ptr_->isConnected() && robot_ptr_->arm1_client_ptr_->isConnected())
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_robot"),"Connected");
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());          
    }
  } catch (...) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot")," Failed to initialize robot driver");
      return false;
  }
  
  // comau action handlers : execute_joint_trajectory_handler
  try {
    execute_joints_handler_ptr.reset(new comau_action_handlers::ExecuteJointTrajectoryHandler(
        nh_, nh_priv_, execute_joint_server_name_, robot_ptr_));
    if (!execute_joints_handler_ptr->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_action_handlers"),"Execute Joint Trajectory Handler could not initialized ");
      return false;
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_action_handlers"),"Execute Joint Trajectory Handler error");
    return false;
  }

  // comau action handlers : execute_cartesian_trajectory_handler
  try {
    execute_cartesian_handler_ptr.reset(new comau_action_handlers::ExecuteCartesianTrajectoryHandler(
        nh_, nh_priv_, execute_cartesian_server_name_, robot_ptr_));
    if (!execute_cartesian_handler_ptr->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_action_handlers"),"Execute Cartesian Trajectory Handler could not initialized ");
      return false;
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_action_handlers"),"Execute Cartesian Trajectory Handler error");
    return false;
  }

  return true;
} // namespace comau_hardware_interface

void TrajectoryHandler::sendCartTraj()
{
  RCLCPP_WARN_STREAM(nh_->get_logger(),"Sending Cartesian Trajectory...");

  this->client_ptr_ = rclcpp_action::create_client<comau_msgs::action::ExecuteCartesianTrajectory>(nh_,"execute_cartesian_trajectory_handler");
  
  TrajectoryHandler::send_goal();
/* 
  auto result = client_ptr_->async_get_result(goal_handle.get());

  if (result.get().code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(),"SUCCEDED!");
  }
*/
}

void TrajectoryHandler::send_goal()
{
  using namespace std::placeholders;
  RCLCPP_WARN_STREAM(nh_->get_logger(),"send_goal");

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) 
  {
    RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
  }
  RCLCPP_WARN(nh_->get_logger(), "Action server available");
  comau_msgs::action::ExecuteCartesianTrajectory::Goal goal_msg;
  comau_msgs::msg::CartesianPoseStamped                homeCartesianPose;

  homeCartesianPose.header.frame_id = "base_link";
  homeCartesianPose.x               =  0.400;
  homeCartesianPose.y               =  0.0;
  homeCartesianPose.z               =  0.700;
  homeCartesianPose.roll            =  0.0;
  homeCartesianPose.pitch           =  1.5707;
  homeCartesianPose.yaw             =  0.0;
  goal_msg.trajectory.push_back(homeCartesianPose);

  RCLCPP_INFO(nh_->get_logger(), "Sending goal");

  /*auto send_goal_options = rclcpp_action::Client<CartTraj>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&result_callback, this, _1);*/
  this->client_ptr_->async_send_goal(goal_msg);//, send_goal_options);
}

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
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
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

void TrajectoryHandler::printVector(const std::vector<double> &vec) {
  RCLCPP_INFO(nh_->get_logger(), "Vector : %f %f %f %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

bool TrajectoryHandler::holdConnection() {
  // TODO add the implementation
  return true;
}

void TrajectoryHandler::read() {
  
  if (robot_ptr_->readMessagePackage())
  {
    RCLCPP_INFO(nh_->get_logger(), "DATA:");
    robot_ptr_->getTimeStamp(data_timestamp_);
    robot_ptr_->getStatus(robot_status_);
    RCLCPP_INFO(nh_->get_logger(), "robot_status_ [%c]", robot_status_);
    robot_ptr_->getSensorType(sns_trk_type_);
    robot_ptr_->getEePosition(ee_position_);
    robot_ptr_->getPinsIN(pins_in_);
    robot_ptr_->getPinsStatesIN(pins_state_in_);
    robot_ptr_->getPinsOUT(pins_out_);
    robot_ptr_->getPinsStatesOUT(pins_state_out_);
    robot_ptr_->getError(error_value_);

    robot_ptr_->getNumJoints(num_robot_joints_);
    RCLCPP_INFO(nh_->get_logger(), "Num_robot_joints_ [%d]", num_robot_joints_);
    num_joints_ = num_robot_joints_;
    robot_ptr_->getStsSelector(stsSelector_);
    RCLCPP_INFO(nh_->get_logger(), "stsSelector_ [%d]", stsSelector_);
    robot_ptr_->getJointType(jnt_type_);            
    RCLCPP_INFO(nh_->get_logger(), "Jnt_type_:");
    std::cout << "[ ";
    for (uint32_t i = 0; i < num_joints_;i++)
    {
      std::cout << jnt_type_.at(i) << " ";
    }
    std::cout << "]" << std::endl;
    RCLCPP_INFO(nh_->get_logger(), "joint_position_:");
    std::cout << "[ ";
    robot_ptr_->getJointPosition(joint_position_, num_robot_joints_, jnt_type_);
    for (uint32_t i = 0; i < num_joints_;i++)
    {
      std::cout << joint_position_.at(i) << " ";
    }
    std::cout << "]" << std::endl;

    if(num_robot_joints_ > 0 && !initialization_)
    {
      if (num_robot_joints_ != num_joints_)
        RCLCPP_WARN_STREAM(nh_->get_logger(),"Warning - mismatch between real robot and urdf number of joints: " << "[" << num_robot_joints_ << ", " << num_joints_ << "]");
      
      robot_ptr_->jnt_cmd_type_   = jnt_type_;
      robot_ptr_->num_cmd_joints_ = num_robot_joints_;
      initialization_ = true;
    }
  }
}

char TrajectoryHandler::publishRobotStatus()
{
  return robot_status_;
}

void TrajectoryHandler::write(double time, double period) {

  if (use_robot_server_ && robot_ptr_->robot_client_ptr_->isConnected())
  {
    if (!robot_ptr_->isCommunicationInit())
    {
      // Initialize PDL through driver
      robot_ptr_->initializePDL(verbose_);
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.3).to_chrono<std::chrono::nanoseconds>());
      // Enables Motion
      robot_ptr_->startPDL();
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.3).to_chrono<std::chrono::nanoseconds>());
      //robot_ptr_->cancelMotionPDL();
      //ros::Duration(0.3).sleep();
      robot_ptr_->setCommunicationInit(true);
    }

    if (robot_status_ == comau_tcp_interface::RobotStatus::ERROR && robot_reset_)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(),"Send reset command after an Error is returned from the server.");
      robot_ptr_->resetPDL();
      robot_reset_ = false;
    }

  }
  
  if (!(use_state_server_ || use_robot_server_ || use_arm1_server_)) {
    if (sensor_tracking_controller_running_)
      printVector(sensor_tracking_command_);
  }
  if (use_arm1_server_) {

    if ((robot_status_ == comau_tcp_interface::RobotStatus::READY ||
         robot_status_ == comau_tcp_interface::RobotStatus::MOVING) &&
        packet_read_) {
      if (sensor_tracking_controller_running_) {
        robot_ptr_->writeCommand(sensor_tracking_command_, time, period,
                                 comau_driver::ControlMode::MODE_SENSOR_TRACKING);
      } else if (position_controller_running_) {
        robot_ptr_->writeCommand(joint_position_command_, time, period,
                                 comau_driver::ControlMode::MODE_POSITION);
      } else {
        holdConnection();
      }
      packet_read_ = false;
    }
  }
}

void TrajectoryHandler::update() {
  // Get change in time
  // ANDY double current_time_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();
  //rclcpp::Duration::from_seconds(0.002).to_chrono<std::chrono::nanoseconds>()
  double elapsed_time_ = 1.0;// ANDY rclcpp::Duration::seconds(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  //last_time_ = current_time_; ANDY
  double now = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();

  // Error check cycle time
  /*const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
  if (cycle_time_error > cycle_time_error_threshold_) {
    RCLCPP_WARN_STREAM_ONCE(nh_->get_logger(),name_, "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time_
                                     << ", threshold: " << cycle_time_error_threshold_);
  }*/ /*ANDY*/

  // Input
  //hardware_interface_->read(now, elapsed_time_);

  // Control
  //controller_manager_->update(now, elapsed_time_);

  // Output
  this->write(now, elapsed_time_);
}

} // namespace trajectory_handler
