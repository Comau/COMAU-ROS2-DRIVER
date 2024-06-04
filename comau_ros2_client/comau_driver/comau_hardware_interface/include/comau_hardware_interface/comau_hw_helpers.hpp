/**
 * @file comau_hw_helpers.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#pragma once
#include <comau_hardware_interface/comau_hw_interface.hpp>
#include <sstream>

namespace comau_hardware_interface {

  bool ComauHardwareInterface::holdConnection() {
  // TODO add the implementation
    return true;
  }
void ComauHardwareInterface::publishRobotStatus() {
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
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot"), "Unknow status type.");
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_robot"), "" << robot_status_pub_->msg_.status);
      std::cout << "robot_status_ " << robot_status_ << std::endl;
    }
  }
}

void ComauHardwareInterface::errorParser(uint32_t error_value)
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

void ComauHardwareInterface::publishErrorValue() {
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

void ComauHardwareInterface::publishEndEffectorPose() {

  ee_transform_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  ee_transform_.header.frame_id = "base_link";
  ee_transform_.child_frame_id = "tool_controller";
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

void ComauHardwareInterface::publishIOPins() {
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

void ComauHardwareInterface::publishOperationMode() {
  
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

void ComauHardwareInterface::copyVector(const std::vector<double> &src, std::vector<double> &dest) {
  for (size_t i = 0; i < src.size(); i++)
    dest.at(i) = src.at(i);
}
void ComauHardwareInterface::closeComauDriver() {
  if (use_robot_server_)
    robot_ptr_->~ComauRobot();
}
/*
//void ComauHardwareInterface::publishSnsTrkType() {
//  if (sns_trk_type_pub_) {
//    if (sns_trk_type_pub_->trylock()) {
//      sns_trk_type_pub_->msg_.data = sns_trk_type_;
//      sns_trk_type_pub_->unlockAndPublish();
//    }
//  }
//}

bool ComauHardwareInterface::shouldResetControllers() {
  return false;
}



bool ComauHardwareInterface::ifZero(const std::vector<double> &vec) {
  for (double val : vec)
    if (val != 0.0)
      return false;
  return true;
}

void ComauHardwareInterface::printVector(const std::vector<double> &vec) {
  RCLCPP_INFO(rclcpp::get_logger("comau_hw_interface"),"Vector : %f %f %f %f %f %f", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

bool ComauHardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                           const std::list<hardware_interface::ControllerInfo> &stop_list) {
  bool ret_val = true;
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty()) {
    for (auto &controller : start_list) {
      if (!controller.claimed_resources.empty()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface"),
                            "Robot control is currently inactive. Starting controllers that claim resources is currently "
                            "not possible. Not starting controller '" << controller.name << "'");
        ret_val = false;
      }
    }
  }

  controllers_initialized_ = true;
  return ret_val;
}

void ComauHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                      const std::list<hardware_interface::ControllerInfo> &stop_list) {
  for (auto &controller_it : stop_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface") {
          sensor_tracking_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingRelativeController") {
          sensor_tracking_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingAbsoluteController") {
          sensor_tracking_controller_running_ = false;
        }
      }
    }
  }
  for (auto &controller_it : start_list) {
    for (auto &resource_it : controller_it.claimed_resources) {
      if (checkControllerClaims(resource_it.resources)) {
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface") {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::EffortJointInterface") {
          sensor_tracking_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingRelativeController") {
          sensor_tracking_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "comau_controllers::SensorTrackingAbsoluteController") {
          sensor_tracking_controller_running_ = true;
        }
      }
    }
  }
  if (async_enable_pub_) {
    if (async_enable_pub_->trylock()) {
      if (position_controller_running_ || sensor_tracking_controller_running_) {
        async_enable_pub_->msg_.data = false;
        robot_ptr_->desableAllowAsync();
      } else {
        async_enable_pub_->msg_.data = true;
        robot_ptr_->enableAllowAsync();
      }
      async_enable_pub_->unlockAndPublish();
    }
  }
  if (use_robot_server_ && robot_ptr_->robot_client_ptr_->isConnected() && robot_ptr_->isCommunicationInit())
    robot_ptr_->cancelMotionPDL();
}

bool ComauHardwareInterface::checkControllerClaims(const std::set<std::string> &claimed_resources) {
  for (const std::string &it : joint_names_) {
    for (const std::string &jt : claimed_resources) {
      if (it == jt) {
        return true;
      }
    }
  }
  return false;
}

bool ComauHardwareInterface::isRobotProgramRunning() const {
  return robot_program_running_;
}

bool ComauHardwareInterface::setSnsTrkParams_routine(comau_msgs::srv::SetSnsTrkParams::Request &req,
                                                     comau_msgs::srv::SetSnsTrkParams::Response &resp) {
  if (use_robot_server_)
    robot_ptr_->setSensorParams(req.sensor_type, req.sensor_cnvrsn, req.sensor_gain, req.sensor_time,
                                req.sensor_ofst_lim_trans, req.sensor_ofst_lim_rot);
  resp.success = true;
  return true;
}
void ComauHardwareInterface::setMoveflyParams_routine(const std::shared_ptr<comau_msgs::srv::SetMoveFlyParams::Request> req,
                                                            std::shared_ptr<comau_msgs::srv::SetMoveFlyParams::Response> resp) {
  if (use_robot_server_){
    robot_ptr_->setMoveflyParams(req->threshold, req->lin_velocity, req->fly_dist);
  }
  resp->success = true;
}
void ComauHardwareInterface::setIO_routine(const std::shared_ptr<comau_msgs::srv::SetIO::Request> req, 
                                                 std::shared_ptr<comau_msgs::srv::SetIO::Response> resp) {
  if (use_robot_server_){
    resp->success = robot_ptr_->setIO(req->pin, req->state);
  }
    resp->success;
}
void ComauHardwareInterface::threadConnections(const std::shared_ptr<comau_msgs::srv::OpenConnection::Request> req, 
                                                     std::shared_ptr<comau_msgs::srv::OpenConnection::Response> resp) {
  if(req->open_connection != req_prev->open_connection) {
    robot_ptr_->state_client_ptr_->openStateThread( req->open_connection, resp->success);
    robot_ptr_->robot_client_ptr_->openRobotThread( req->open_connection, resp->success);
    robot_ptr_->arm1_client_ptr_->openHandlerThread(req->open_connection, resp->success);
    resp->success = true;
  }
  req_prev->open_connection = req->open_connection;
  resp->success;
}

void ComauHardwareInterface::setArmState_routine(const std::shared_ptr<comau_msgs::srv::SetArmState::Request> req, 
                                                       std::shared_ptr<comau_msgs::srv::SetArmState::Response> resp) {
  if (use_robot_server_)
    robot_ptr_->setArmState(req->arm_state);
  resp->success = true;
}
*/
}