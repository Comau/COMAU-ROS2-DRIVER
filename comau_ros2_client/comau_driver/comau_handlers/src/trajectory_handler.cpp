/**
 * @file trajectory_handler.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the trajectory action
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_handlers/trajectory_handler.hpp"

namespace trajectory_handler 
{

using JntTraj           = comau_msgs::action::ExecuteJointTrajectory;
using GoalHandleJntTraj = rclcpp_action::ClientGoalHandle<JntTraj>;
using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;

// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

TrajectoryHandler::TrajectoryHandler(rclcpp::Node::SharedPtr &nh) : name_("execute_trajectory_handler"), nh_(nh) 
{
  
}

bool TrajectoryHandler::init() {
  
  std::shared_ptr<rclcpp::Node> nh_priv_ = std::make_shared<rclcpp::Node>(nh_->get_name(), name_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "" << name_);
  sleep(3);
  position_controller_running_ = false;
  print_server_not_connected = true;
  
  const std::size_t NUM_JOINTS_MAX = 10;
  const std::size_t cart_pose_size = 6;

  // Get current time for use with first update
  clock_gettime(CLOCK_MONOTONIC, &last_time_);
  desired_update_period_ = (double)(1 / loop_hz_);

  // Read parameters through rclcpp parameter server  
  nh_->declare_parameter("use_state_server", true);
  use_state_server_ = nh_->get_parameter("use_state_server").as_bool();
  nh_->declare_parameter("use_robot_server", true);
  use_robot_server_ = nh_->get_parameter("use_robot_server").as_bool();
  nh_->declare_parameter("use_arm1_server", true);
  use_arm1_server_ = nh_->get_parameter("use_arm1_server").as_bool();
  nh_->declare_parameter("verbose", true);
  verbose_ = nh_->get_parameter("verbose").as_bool();
  
  joint_names_.clear();
  req_prev_open_connection = false;

  stsSelector_       = 0;
  pins_in_           = {0,0,0,0,0,0}; /* ANDY CHECK DIMENSIONS */
  pins_state_in_     = {0,0,0,0,0,0};
  pins_out_          = {0,0,0,0,0,0};
  pins_state_out_    = {0,0,0,0,0,0};
  jnt_type_.resize(NUM_JOINTS_MAX);

  // Resize vectors
  joint_position_.resize(NUM_JOINTS_MAX);
  joint_velocity_.resize(NUM_JOINTS_MAX);
  joint_position_command_.resize(NUM_JOINTS_MAX);
  joint_velocity_command_.resize(NUM_JOINTS_MAX);
  ee_position_.resize(cart_pose_size);

  initialization_ = false;
  invalidMsgCount_ = 0;
  robot_reset_ = false;
  data_timestamp_prev_ = 0;
  counter_ = 0;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_robot")," Robot driver Initialization...");
  // Initialize Robot driver
  try {
    robot_ptr_.reset(new comau_driver::ComauRobot(nh_));
    if (!robot_ptr_->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot")," Failed to initialize robot driver");
      return false;
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_robot"), "Number of joint within URDF file is: " << execute_joints_handler_ptr->urdf_number_of_joints_);
  
  result_.code = rclcpp_action::ResultCode::SUCCEEDED;
  
  num_joints_ = execute_joints_handler_ptr->urdf_number_of_joints_;
  for(size_t i = 0; i < num_joints_; i++)
  {
    std::stringstream str_temp;
    str_temp << "joint_" << i+1;
    std::string res = str_temp.str();
    joint_names_.push_back(res);
  }

  //publishers
  try {
    robot_status_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauRobotStatus>(nh_->create_publisher<comau_msgs::msg::ComauRobotStatus>(
    "robot_status", rclcpp::SystemDefaultsQoS())));
    } catch (const std::exception & e) {
      fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  try {
    server_error_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauServerError>(nh_->create_publisher<comau_msgs::msg::ComauServerError>(
        "server_error", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  try {
    server_operation_mode_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauOperationMode>(nh_->create_publisher<comau_msgs::msg::ComauOperationMode>(
        "robot_operation_mode", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  try {
    ee_pose_pub_.reset(
      new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(nh_->create_publisher<tf2_msgs::msg::TFMessage>(
      "tf", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  if(!position_controller_running_) {
  try {
    urdf_joint_states_pub_.reset(
      new realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>(nh_->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  }
  try {
    async_enable_pub_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::msg::Bool>(
      nh_->create_publisher<std_msgs::msg::Bool>(
      "async_enable", true)));
  
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }
  try {
    io_states_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::IOStates>(
        nh_->create_publisher<comau_msgs::msg::IOStates>("io_states", true)));
    io_states_pub_->msg_.digital_in_states.resize(6);
    io_states_pub_->msg_.digital_out_states.resize(6);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return false;
  }

  // Subscribers
  if(position_controller_running_)
  {
  urdf_joint_states_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,[&](const sensor_msgs::msg::JointState & msg)
                                 {  
                                    joint_position_command_.resize(num_joints_);
                                    for(size_t si_i = 0; si_i < num_joints_; si_i++)
                                    {
                                      joint_position_command_.at(si_i) = msg.position[si_i];
                                    }
                                 });
  }
  // services
  thread_service_ = nh_->create_service<comau_msgs::srv::OpenConnection>("tcpip_conn_manager", [&](const comau_msgs::srv::OpenConnection::Request::SharedPtr req, 
                                 comau_msgs::srv::OpenConnection::Response::SharedPtr resp)
                                 {
                                    if(req->open_connection != req_prev_open_connection) {
                                      robot_ptr_->state_client_ptr_->openStateThread( req->open_connection, resp->success);
                                      robot_ptr_->robot_client_ptr_->openRobotThread( req->open_connection, resp->success);
                                      robot_ptr_->arm1_client_ptr_->openHandlerThread(req->open_connection, resp->success);
                                      resp->success = true;
                                      if (req->open_connection == true)
                                      {
                                        RCLCPP_INFO_STREAM(rclcpp::get_logger("ROS2-Driver"), "Client connected to Server.");
                                      } 
                                      print_server_not_connected = !req->open_connection;
                                    }
                                    req_prev_open_connection = req->open_connection;
                                    resp->success;
                                 });
  setMoveflyParams_service_ = nh_->create_service<comau_msgs::srv::SetMoveFlyParams>("set_movefly_params", [&](const comau_msgs::srv::SetMoveFlyParams::Request::SharedPtr req, 
                                 comau_msgs::srv::SetMoveFlyParams::Response::SharedPtr resp)
                                 {
                                    if (use_robot_server_){
                                      robot_ptr_->setMoveflyParams(req->threshold, req->lin_velocity, req->fly_dist);
                                    }
                                    resp->success = true;
                                 });
  setIO_service_ = nh_->create_service<comau_msgs::srv::SetIO>("set_io", [&](const comau_msgs::srv::SetIO::Request::SharedPtr req, 
                                 comau_msgs::srv::SetIO::Response::SharedPtr resp)
                                 {  
                                    resp->success = false;
                                    if (use_robot_server_){
                                      resp->success = robot_ptr_->setIO(req->pin, req->state);
                                    }
                                    resp->success;
                                 });
  setArmState_service_ = nh_->create_service<comau_msgs::srv::SetArmState>("set_arm_state", [&](const comau_msgs::srv::SetArmState::Request::SharedPtr req, 
                                 comau_msgs::srv::SetArmState::Response::SharedPtr resp)
                                 {
                                    if (use_robot_server_)
                                      robot_ptr_->setArmState(req->arm_state);
                                    resp->success = true;
                                 });

  return true;
} // namespace comau_hardware_interface

/* comau_hw_helpers */
bool TrajectoryHandler::holdConnection() {
  // TODO add the implementation
  return true;
}

void TrajectoryHandler::publishRobotStatus() {
  if (robot_status_pub_) {
    if (robot_status_ == comau_tcp_interface::RobotStatus::TERMINATE) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::TERMINATE;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::READY) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::READY;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::MOVING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::MOVING;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::PAUSED) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::PAUSED;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::RESUMING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::RESUMING;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::SUCCEEDED) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::SUCCEEDED;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::ERROR) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::ERROR;
        robot_status_pub_->unlockAndPublish();
      }
    } else if (robot_status_ == comau_tcp_interface::RobotStatus::CANCELING) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::CANCELING;
        robot_status_pub_->unlockAndPublish();
      }
    } 
     else if (robot_status_ == comau_tcp_interface::RobotStatus::COLLISION) {
      if (robot_status_pub_->trylock()) {
        robot_status_pub_->msg_.status = comau_msgs::msg::ComauRobotStatus::COLLISION;
        robot_status_pub_->unlockAndPublish();
      }
    }else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot"), "Unknow status type.");
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot"), "" << robot_status_pub_->msg_.status);
      std::cout << "robot_status_ " << robot_status_ << std::endl;
    }
  }
}

void TrajectoryHandler::errorParser(uint32_t error_value)
{

  const std::vector<uint32_t> server_error_codes = {KI_ERR_STATE_ACCEPT,      KI_ERR_ROBOT_ACCEPT,      KI_ERR_HANDLER_ACCEPT,
                                                    KI_ERR_STATE_WRITE,       KI_ERR_ROBOT_READ_NCON,   KI_ERR_HANDLER_READ_NCON,
                                                    KI_ERR_ROBOT_READ_DCON,   KI_ERR_HANDLER_READ_DCON, KI_ERR_ROBOT_READ_CANC,    
                                                    KI_ERR_HANDLER_READ_CANC, KI_ERR_ROBOT_READ_TOUT,   KI_ERR_HANDLER_READ_TOUT,
                                                    KI_ERR_STATE_DISCONNECT,  KI_ERR_ROBOT_DISCONNECT,  KI_ERR_HANDLER_DISCONNECT, 
                                                    KI_ERR_STATE_SAFETY_GATE, KI_ERR_STATE_WRONG_MOTION,KI_ERR_ROBOT_ALARM, KI_ERR_MOTION_DRIVEOFF};
    
  error_value_clientcode_ = 0; // Initialization
  if(error_value > server_error_codes[server_error_codes.size() - 1])
  { 
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error out of scale: " << error_value);
    error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_UNDEFINED;
    return;
  }

  for ( size_t i = 0; i < server_error_codes.size(); i++ )
  {
    switch (error_value & server_error_codes.at(i))
    {
      case KI_ERR_STATE_ACCEPT      :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - State server (pdl) connection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_CONN_STATE;
        break;
      case KI_ERR_ROBOT_ACCEPT      :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Robot server (pdl) connection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ROBOT;
        break;
      case KI_ERR_HANDLER_ACCEPT    :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Arm server (pdl) connection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ARM;
        break;
      case KI_ERR_STATE_WRITE       :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - State server (pdl) TCP/IP write failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_READ;
        break;
      case KI_ERR_ROBOT_READ_NCON   :
      case KI_ERR_ROBOT_READ_DCON   :
      case KI_ERR_ROBOT_READ_CANC   :
      case KI_ERR_ROBOT_READ_TOUT   :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Robot server (pdl) TCP/IP read failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_CMD;
        break;
      case KI_ERR_HANDLER_READ_NCON :
      case KI_ERR_HANDLER_READ_DCON :
      case KI_ERR_HANDLER_READ_CANC :
      case KI_ERR_HANDLER_READ_TOUT :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Arm server (pdl) TCP/IP read failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_MOTION;
        break;
      case KI_ERR_STATE_DISCONNECT  :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - State server (pdl) disconnection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_STATE;
        break;
      case KI_ERR_ROBOT_DISCONNECT  :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Robot server (pdl) disconnection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ROBOT;
        break;
      case KI_ERR_HANDLER_DISCONNECT:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Arm server (pdl) disconnection failed.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ARM;
        break;
      case KI_ERR_STATE_SAFETY_GATE :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Safety gate / Emergency stop error.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_SAFETY_GATE;
        break;
      case KI_ERR_STATE_WRONG_MOTION :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Program Execution Errors (36864-37191).");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_WRONG_MOTION;
        break;
      case KI_ERR_ROBOT_ALARM :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Active Alarm at Restarting. Please press reset button");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_ALARM;
        break;
      case KI_ERR_MOTION_DRIVEOFF :
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Error code : " << server_error_codes.at(i) << " - Trajectory sent in not correct state (Drive-on and start necessary). Please press reset button.");
        error_value_clientcode_ |= comau_tcp_interface::ErrorValue::ERR_MOTION_DRIVEOFF;
        break;
    } 
  }

  return;
}

void TrajectoryHandler::publishErrorValue() {
  const std::vector<uint32_t> error_codes = {comau_tcp_interface::ErrorValue::ERR_TCP_UNDEFINED,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_CONN_STATE,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ROBOT,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ARM,  
                                             comau_tcp_interface::ErrorValue::ERR_TCP_READ,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_CMD,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_MOTION,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_STATE,
                                             comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ROBOT,    
                                             comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ARM,
                                             comau_tcp_interface::ErrorValue::ERR_SAFETY_GATE,
                                             comau_tcp_interface::ErrorValue::ERR_WRONG_MOTION};
  uint32_t code = error_value_clientcode_;
  if (server_error_pub_) 
  {
    if (server_error_pub_->trylock())
    {
      server_error_pub_->msg_.code = code;
      server_error_pub_->msg_.error_msg.clear();

      for ( size_t i = 0; i < error_codes.size(); i++ )
      {
        switch (code & error_codes.at(i))
        {
          case comau_tcp_interface::ErrorValue::ERR_TCP_UNDEFINED:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_UNDEFINED);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_CONN_STATE:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_CONN_STATE);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ROBOT:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_CONN_ROBOT);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_CONN_ARM:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_CONN_ARM);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_READ:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_READ);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_CMD:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_WRITE_CMD);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_WRITE_MOTION:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_WRITE_MOTION);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_STATE:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_DISCONN_STATE);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ROBOT:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_DISCONN_ROBOT);
            break;
          case comau_tcp_interface::ErrorValue::ERR_TCP_DISCONN_ARM:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_TCP_DISCONN_ARM);
            break;
          case comau_tcp_interface::ErrorValue::ERR_SAFETY_GATE:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_SAFETY_GATE);
            break;
          case comau_tcp_interface::ErrorValue::ERR_WRONG_MOTION:
            server_error_pub_->msg_.error_msg.push_back(comau_msgs::msg::ComauServerError::ERR_WRONG_MOTION);
            break;
          default:
            break;
        } 
      }
      server_error_pub_->unlockAndPublish();
    }
  }
}

void TrajectoryHandler::publishEndEffectorPose() {

  ee_transform_.header.stamp    = rclcpp::Clock{RCL_ROS_TIME}.now();
  ee_transform_.header.frame_id = "base_link";
  ee_transform_.child_frame_id  = "ee_link";//"tool_controller";
  ee_transform_.transform.translation.x = ee_position_[0] / 1000.;
  ee_transform_.transform.translation.y = ee_position_[1] / 1000.;
  ee_transform_.transform.translation.z = ee_position_[2] / 1000.;
  double roll = -1. * ee_position_[3] * M_PI / 180.;
  double pitch = -1. * ee_position_[4] * M_PI / 180.;
  double yaw = -1. * ee_position_[5] * M_PI / 180.;
  q.setRPY(roll, pitch, yaw);
  double qx, qy, qz, qw;
  qx = -1. * q.z();
  qy = -1. * q.y();
  qz = -1. * q.x();
  qw = 1. * q.w();
  ee_transform_.transform.rotation.x = qx;
  ee_transform_.transform.rotation.y = qy;
  ee_transform_.transform.rotation.z = qz;
  ee_transform_.transform.rotation.w = qw;
  if (ee_pose_pub_) {
    if (ee_pose_pub_->trylock()) {
      ee_pose_pub_->msg_.transforms.clear();
      ee_pose_pub_->msg_.transforms.push_back(ee_transform_);
      ee_pose_pub_->unlockAndPublish();
    }
  }
}

void TrajectoryHandler::publishIOPins() {
  if (io_states_pub_) {
    if (io_states_pub_->trylock()) {
      for (uint8_t i = 0; i < pins_in_.size(); i++) {
        io_states_pub_->msg_.digital_in_states[i].pin = pins_in_[i];
        io_states_pub_->msg_.digital_in_states[i].state = bool(pins_state_in_[i]);
        io_states_pub_->msg_.digital_out_states[i].pin = pins_out_[i];
        io_states_pub_->msg_.digital_out_states[i].state = bool(pins_state_out_[i]);
      }
      io_states_pub_->unlockAndPublish();
    }
  }
}

void TrajectoryHandler::publishOperationMode() {
  
  if (server_operation_mode_pub_) 
  {
    if (server_operation_mode_pub_->trylock())
    {

      if ((stsSelector_ & 0x00001) == 1)
      {
        server_operation_mode_pub_->msg_.status_selector_key = "T1";
      }else if( (stsSelector_ & 0x00002) == 2 )
      { 
        server_operation_mode_pub_->msg_.status_selector_key = "AUTO";
      }else if( (stsSelector_ & 0x00004) == 4 )
      { 
        server_operation_mode_pub_->msg_.status_selector_key = "Extern";
      } else {
        server_operation_mode_pub_->msg_.status_selector_key = "NONE";
      }
      
      if( (stsSelector_ & 0x00008) == 8 )
      { 
        server_operation_mode_pub_->msg_.drive_on = true;
      } else {
        server_operation_mode_pub_->msg_.drive_on = false;
      }

      if( (stsSelector_ & 0x00010) == 16 )
      { 
        server_operation_mode_pub_->msg_.start = true;
      } else {
        server_operation_mode_pub_->msg_.start = false;
      }

      server_operation_mode_pub_->unlockAndPublish();
    }
  }
}

void TrajectoryHandler::async_enable_routine() {
  if (async_enable_pub_) {
    if (async_enable_pub_->trylock()) {
      if (position_controller_running_ /*|| sensor_tracking_controller_running_*/) {
        async_enable_pub_->msg_.data = false;
        robot_ptr_->desableAllowAsync();
      } else {
        async_enable_pub_->msg_.data = true;
        robot_ptr_->enableAllowAsync();
      }
      async_enable_pub_->unlockAndPublish();
    }
  }
}

void TrajectoryHandler::copyVector(const std::vector<double> &src, std::vector<double> &dest) {
  for (size_t i = 0; i < src.size(); i++)
    dest.at(i) = src.at(i);
}

void TrajectoryHandler::closeComauDriver() {
  if (use_robot_server_)
    robot_ptr_->~ComauRobot();
}

void TrajectoryHandler::sendCartTraj()
{
  RCLCPP_WARN_STREAM(nh_->get_logger(),"Sending Cartesian Trajectory...");

  this->client_ptr_ = rclcpp_action::create_client<comau_msgs::action::ExecuteCartesianTrajectory>(nh_,"execute_cartesian_trajectory_handler");
  
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
  //goal_msg.trajectory.push_back(firstCartesianPose);

  secondCartesianPose.header         = header;
  secondCartesianPose.x              =  0.608;
  secondCartesianPose.y              =  0.143;
  secondCartesianPose.z              =  0.589;
  secondCartesianPose.roll           =  3.140;
  secondCartesianPose.pitch          =  0.000;
  secondCartesianPose.yaw            =  3.140;
  //goal_msg.trajectory.push_back(secondCartesianPose);

  thirdCartesianPose.header          = header;
  thirdCartesianPose.x               =  0.608;
  thirdCartesianPose.y               = -0.143;
  thirdCartesianPose.z               =  0.589;
  thirdCartesianPose.roll            =  3.140;
  thirdCartesianPose.pitch           =  0.000;
  thirdCartesianPose.yaw             =  3.140;
  //goal_msg.trajectory.push_back(thirdCartesianPose);

  fourthCartesianPose.header         = header;
  fourthCartesianPose.x              =  0.436;
  fourthCartesianPose.y              = -0.143;
  fourthCartesianPose.z              =  0.585;
  fourthCartesianPose.roll           =  3.140;
  fourthCartesianPose.pitch          =  0.000;
  fourthCartesianPose.yaw            =  3.140;
  /*goal_msg.trajectory.push_back(fourthCartesianPose);
  goal_msg.trajectory.push_back(firstCartesianPose);*/
  fifthCartesianPose.header          = header;
  fifthCartesianPose.x               =  0.436;
  fifthCartesianPose.y               =  0.143;
  fifthCartesianPose.z               =  0.400;
  fifthCartesianPose.roll            =  3.140;
  fifthCartesianPose.pitch           =  0.000;
  fifthCartesianPose.yaw             =  3.140;
  //goal_msg.trajectory.push_back(fifthCartesianPose);

  sixthCartesianPose.header          = header;
  sixthCartesianPose.x               =  0.608;
  sixthCartesianPose.y               =  0.143;
  sixthCartesianPose.z               =  0.400;
  sixthCartesianPose.roll            =  3.140;
  sixthCartesianPose.pitch           =  0.000;
  sixthCartesianPose.yaw             =  3.140;
  //goal_msg.trajectory.push_back(sixthCartesianPose);

  seventhCartesianPose.header        = header;
  seventhCartesianPose.x             =  0.608;
  seventhCartesianPose.y             = -0.143;
  seventhCartesianPose.z             =  0.400;
  seventhCartesianPose.roll          =  3.140;
  seventhCartesianPose.pitch         =  0.000;
  seventhCartesianPose.yaw           =  3.140;
  //goal_msg.trajectory.push_back(seventhCartesianPose);

  eighthCartesianPose.header         = header;
  eighthCartesianPose.x              =  0.436;
  eighthCartesianPose.y              = -0.143;
  eighthCartesianPose.z              =  0.400;
  eighthCartesianPose.roll           =  3.140;
  eighthCartesianPose.pitch          =  0.000;
  eighthCartesianPose.yaw            =  3.140;
  /*goal_msg.trajectory.push_back(eighthCartesianPose);
  goal_msg.trajectory.push_back(fifthCartesianPose);
  goal_msg.trajectory.push_back(homeCartesianPose);*/

  RCLCPP_INFO(nh_->get_logger(), "Sending goals");

  auto send_goal_options = rclcpp_action::Client<CartTraj>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TrajectoryHandler::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&TrajectoryHandler::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&TrajectoryHandler::result_callback, this, _1);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
  {
    RCLCPP_ERROR(nh_->get_logger(), "Action server not available after waiting");
    this->client_ptr_->async_cancel_all_goals();
  }
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

  jointPose.positions.resize(num_joints_);
  urdf_command_ = {0.436332, 0.0, -1.5708, 0.0, 1.57, 0.0};
  for(size_t si_i = 0; si_i < num_joints_; si_i++)
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
  this->client_jnt_ptr_->async_send_goal(goal_msg);//, send_goal_options);
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

void TrajectoryHandler::printVector(const std::vector<double> &vec) {
  RCLCPP_INFO(nh_->get_logger(), "Vector : %f %f %f %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

void TrajectoryHandler::read(double time, double perio) {
  if (!(use_state_server_ || use_robot_server_ || use_arm1_server_)) 
  {
    if (position_controller_running_)
      copyVector(joint_position_command_, joint_position_);
    return;
  }

  if (use_state_server_ && robot_ptr_->state_client_ptr_->is_connected_) 
  {
    if (robot_ptr_->readMessagePackage()) {
      robot_ptr_->getTimeStamp(data_timestamp_);
      robot_ptr_->getStatus(robot_status_);
      robot_ptr_->getEePosition(ee_position_);
      robot_ptr_->getPinsIN(pins_in_);
      robot_ptr_->getPinsStatesIN(pins_state_in_);
      robot_ptr_->getPinsOUT(pins_out_);
      robot_ptr_->getPinsStatesOUT(pins_state_out_);
      robot_ptr_->getError(error_value_);
      robot_ptr_->getNumJoints(num_robot_joints_);
      robot_ptr_->getJointType(jnt_type_);
      robot_ptr_->getJointPosition(joint_position_, num_robot_joints_, jnt_type_);
      robot_ptr_->getStsSelector(stsSelector_);
/* TO DO : Check disconnection if the timestamp from the server does not update
      if (data_timestamp_prev_ != data_timestamp_)
      {
        data_timestamp_prev_ = data_timestamp_;
        RCLCPP_WARN_STREAM("data_timestamp_ " << "[" << data_timestamp_  << "]" << "counter_ : " << counter_);
        counter_ = 0;
      }
      else
      {
        counter_ ++;
      }

      if (counter_ > 200)
      {
        RCLCPP_WARN_STREAM("Lost connection from server side");
        robot_ptr_->state_client_ptr_->openStateThread(false,  0);
        robot_ptr_->robot_client_ptr_->openRobotThread(false,  0);
        robot_ptr_->arm1_client_ptr_->openHandlerThread(false, 0);
      }
*/
      if(num_robot_joints_ > 0 && !initialization_)
      {
        if (num_robot_joints_ != num_joints_)
          RCLCPP_WARN_STREAM(nh_->get_logger(),"Warning - mismatch between real robot and urdf number of joints: " << "[" << num_robot_joints_ << ", " << num_joints_ << "]");

        robot_ptr_->jnt_cmd_type_   = jnt_type_;
        robot_ptr_->num_cmd_joints_ = num_robot_joints_;
        initialization_ = true;
      }

      if ((robot_status_ == 'T') || (robot_status_ == 'C') || (robot_status_ == 'R') || (robot_status_ == 'M') ||
          (robot_status_ == 'I') || (robot_status_ == 'P') || (robot_status_ == 'S') || (robot_status_ == 'E') || (robot_status_ == 'A') )
      {
        packet_read_ = true;
        if (error_value_ != error_value_prev_)
        {
          RCLCPP_WARN_STREAM(nh_->get_logger(),"Error from server: " << error_value_);
          errorParser(error_value_);
          publishErrorValue();

          /*  Get server reset */
          if (error_value_prev_ > 0 && error_value_prev_ < KI_ERR_STATE_MAX && error_value_ == 0)
            robot_reset_ = true;

          error_value_prev_ = error_value_;          
        }
        publishRobotStatus();
        publishEndEffectorPose();
        publishIOPins();
        publishOperationMode();
        async_enable_routine();
        execute_joints_handler_ptr->set_status(robot_status_);
        execute_joints_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());
        execute_cartesian_handler_ptr->set_status(robot_status_);
        execute_cartesian_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());

        invalidMsgCount_ = 0;
      }
      else
      {
        RCLCPP_WARN_STREAM(nh_->get_logger(),"Invalid state msg: " << robot_status_);
        //std::cout << "Server error:      " << error_value_  << std::endl;
        invalidMsgCount_ ++; // counter to restart the connection
        if (robot_ptr_->state_client_ptr_->is_connected_ && invalidMsgCount_ > 2) // VE_ADD : sistema la visibilit� di is_connected_ e state_client_ptr_ che � stata cambiata
        {
          RCLCPP_ERROR_STREAM(nh_->get_logger(),"Restart state_client");
          robot_ptr_->state_client_ptr_->is_connected_ = false;
        }
      }     
    }
  } else {
    if(print_server_not_connected)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(),"Waiting for Server Service Connection (tcpip_conn_manager)");
      print_server_not_connected = false;
    }
  }
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
  
  /*if (!(use_state_server_ || use_robot_server_ || use_arm1_server_)) {
    if (sensor_tracking_controller_running_)
      printVector(sensor_tracking_command_);
  }*/
  if (use_arm1_server_) {

    if ((robot_status_ == comau_tcp_interface::RobotStatus::READY ||
         robot_status_ == comau_tcp_interface::RobotStatus::MOVING) &&
        packet_read_) {
      /*if (sensor_tracking_controller_running_) {
        robot_ptr_->writeCommand(sensor_tracking_command_, time, period,
                                 comau_driver::ControlMode::MODE_SENSOR_TRACKING);
      } 
      else*/ if (position_controller_running_) 
      {
        robot_ptr_->writeCommand(joint_position_command_, time, period,
                                 comau_driver::ControlMode::MODE_POSITION);
      } else {
        /*if (result_.code == rclcpp_action::ResultCode::SUCCEEDED && !(robot_status_ == comau_tcp_interface::RobotStatus::MOVING))
        {
          this->sendJntTraj();
          if(robot_status_ == comau_tcp_interface::RobotStatus::SUCCEEDED)
          {
            result_.code = rclcpp_action::ResultCode::SUCCEEDED;
          }else{
            result_.code = rclcpp_action::ResultCode::UNKNOWN;
          }
        }*/
        holdConnection(); /* REAL ROBOT ASYNC COMMANDS HERE */
      }
      packet_read_ = false;
    }
  }
}

void TrajectoryHandler::update_urdf() {
    
  if (use_state_server_ && robot_ptr_->state_client_ptr_->is_connected_) 
  {
    if (packet_read_) 
    {
      urdf_joint_states_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
      urdf_joint_states_.name.resize(num_joints_);
      urdf_joint_states_.position.resize(num_joints_);
      urdf_joint_states_pub_->msg_.name.resize(num_joints_);
      urdf_joint_states_pub_->msg_.position.resize(num_joints_);

      //Joint Names
      for(size_t si_i = 0; si_i < num_joints_; si_i++)
      {
        urdf_joint_states_.name[si_i] = joint_names_.at(si_i);
        urdf_joint_states_.position[si_i] = joint_position_.at(si_i);
      }

      if (urdf_joint_states_pub_) 
      {
        if (urdf_joint_states_pub_->trylock()) 
        {
          urdf_joint_states_pub_->msg_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
          for(size_t si_i = 0; si_i < num_joints_; si_i++)
          {
            urdf_joint_states_pub_->msg_.name[si_i]     = joint_names_.at(si_i);//urdf_joint_states_.name[si_i]; 
            urdf_joint_states_pub_->msg_.position[si_i] = joint_position_.at(si_i);//urdf_joint_states_.position[si_i]; 
            //RCLCPP_INFO_STREAM(rclcpp::get_logger("urdf_joint_states_pub_ "),"" << joint_position_.at(si_i)*180/M_PI);
          }
          urdf_joint_states_pub_->unlockAndPublish();
        }
      }
    }
  }
  
}

void TrajectoryHandler::update() {

  
  double now = rclcpp::Clock{RCL_ROS_TIME}.now().seconds();

  // Get change in time
  
  elapsed_time_ = (current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);

  // Error check cycle time
  const double cycle_time_error = (elapsed_time_ - desired_update_period_);
  if (cycle_time_error > cycle_time_error_threshold_) 
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger(name_), "Cycle time exceeded error threshold by: "
                                     << cycle_time_error << ", cycle time: " << elapsed_time_
                                     << ", threshold: " << cycle_time_error_threshold_);
  }

  // Input
  this->read(now, elapsed_time_);
  clock_gettime(CLOCK_MONOTONIC, &last_time_);
  //URDF Update
  if(!position_controller_running_)
  {
    this->update_urdf();
  }
  // Output
  this->write(now, elapsed_time_);
  clock_gettime(CLOCK_MONOTONIC, &current_time_);
}

} // namespace trajectory_handler
