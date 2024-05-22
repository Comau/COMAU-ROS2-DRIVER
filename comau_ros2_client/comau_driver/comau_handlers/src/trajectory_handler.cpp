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
