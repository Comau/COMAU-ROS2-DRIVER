/**
 * @file comau_driver.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "comau_driver/comau_driver.hpp"
#include "pluginlib/class_loader.hpp"


//#include <vector>
using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {

  ComauRobot::ComauRobot(rclcpp::NodeOptions options) : rclcpp:: Node("comau_robot", options){
 
   std::string server_ip_address;
   std::string server_port;

   bool parallel_link_fix_;

   robot_params_.server_ip_address = state_params_.server_ip_address;
   arm1_params_.server_ip_address = state_params_.server_ip_address;
   state_params_.log_tag = "state_tcp_client";
   robot_params_.log_tag = "motion_tcp_client";
   arm1_params_.log_tag = "arm1_tcp_client";

   declare_parameter("robot_ip", "default value"); 
   declare_parameter("state_server_port", "default value"); 
   declare_parameter("robot_server_port", "default value"); 
   declare_parameter("arm1_server_port", "default value"); 
   declare_parameter("default_linear_velocity", default_linear_velocity_); 
   declare_parameter("threshold", threshold_); 
   declare_parameter("fly_lin_velocity", fly_lin_velocity_); 
   declare_parameter("fly_dist", fly_dist_); 
   declare_parameter("verbose", verbose_); 
   declare_parameter("sensor_type", sensor_type_); 
   declare_parameter("sensor_cnvrsn", sensor_cnvrsn_);
   declare_parameter("sensor_gain", sensor_gain_);
   declare_parameter("sensor_time", sensor_time_);
   declare_parameter("sensor_ofst_lim_trans", sensor_ofst_lim_trans_);
   declare_parameter("sensor_ofst_lim_rot", sensor_ofst_lim_rot_);
   declare_parameter("din_pins", din_pins_);
   declare_parameter("parallel_joint_fix", false);

   get_parameter("robot_ip",state_params_.server_ip_address);
   get_parameter("state_server_port",state_params_.server_port);
   get_parameter("robot_server_port",robot_params_.server_port);
   get_parameter("arm1_server_port",arm1_params_.server_port);   
   get_parameter("default_linear_velocity",default_linear_velocity_);
   get_parameter("threshold",threshold_);
   get_parameter("fly_lin_velocity", fly_lin_velocity_);
   get_parameter("fly_dist", fly_dist_); 
   get_parameter("verbose", verbose_); 
   get_parameter("sensor_type", sensor_type_); 
   get_parameter("sensor_cnvrsn", sensor_cnvrsn_);
   get_parameter("sensor_gain", sensor_gain_);
   get_parameter("sensor_time", sensor_time_);
   get_parameter("sensor_ofst_lim_trans", sensor_ofst_lim_trans_);
   get_parameter("sensor_ofst_lim_rot", sensor_ofst_lim_rot_);
   get_parameter({"din_pins"}, din_pins_);
   get_parameter("dout_pins", dout_pins_);
   get_parameter("parallel_joint_fix", parallel_link_fix_);

if (din_pins_.size() != 6 || dout_pins_.size() != 6) {
    RCLCPP_ERROR(this->get_logger(),"[comau_driver] The size of the dout and din parameters must be 6");
    rclcpp::shutdown();
}
  } //class 
  

ComauRobot::~ComauRobot() {
  if (use_robot_server_) {
    RCLCPP_ERROR(this->get_logger(),"Terminating PDL Programs");
    this->terminatePDL();
  }
  if (use_state_server_) {
    state_client_ptr_->close();
    state_client_ptr_.reset();
  }
  if (use_robot_server_) {
    robot_client_ptr_->close();
    robot_client_ptr_.reset();
  }
  if (use_arm1_server_) {
    arm1_client_ptr_->close();
    arm1_client_ptr_.reset();
  }
}


bool ComauRobot::initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server) {
  use_state_server_ = use_state_server;
  use_robot_server_ = use_robot_server;
  use_arm1_server_ = use_arm1_server;

  pluginlib::ClassLoader<comau_tcp_interface::ComauClientBase> client_loader("comau_tcp_interface",
                                                                             "comau_tcp_interface::ComauClientBase");
if (use_state_server) { 
  // tcp interface state client ptr 
  try { 
      state_client_ptr_.reset(new comau_tcp_interface::StateClient());
  
    if (!state_client_ptr_->initialize(state_params_)) { 
       RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"), "[comau_driver] State Client could not initialized"); 
      return false; 
      }  
  } catch (pluginlib::PluginlibException &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"), e.what());
     return false;
  }
}

  if (use_robot_server) {

    // tcp interface motion client ptr
    try {
      robot_client_ptr_.reset(new comau_tcp_interface::RobotClient());
      if (!robot_client_ptr_->initialize(robot_params_)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"),"[comau_driver] Robot Command Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"), e.what());
      return false;
    }
  }
  if (use_arm1_server) {

    try {
      arm1_client_ptr_.reset(new comau_tcp_interface::RobotClient());
      if (!arm1_client_ptr_->initialize(arm1_params_)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"),"[comau_driver] Arm 1 Motion Command Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"), e.what());
      return false;
    }
  }

  return true;
} //END BOOL

 
 float ComauRobot::getDefaultLinVel() {
   return static_cast<float>(default_linear_velocity_);
 }
 
 bool ComauRobot::setMoveflyParams(double &threshold, double &lin_velocity, double &fly_dist) {
   threshold_ = threshold;
   fly_lin_velocity_ = lin_velocity;
   fly_dist_ = fly_dist;
   return true;
 }
 
 void ComauRobot::getSensorType(int32_t &sensor_type) {
   msg->getData("sensor_type_feedback", sensor_type);
 }
 
 
 void ComauRobot::getPinsIN(std::vector<long int> &pins_in) {
   pins_in.assign(din_pins_.begin(), din_pins_.end());
 }

 void ComauRobot::getPinsStatesIN(std::vector<long int> &pins_state_in) {
  msg->getData("pins_state_in", pins_state_in_);
  pins_state_in.assign(pins_state_in_.begin(), pins_state_in_.end());
  }

 void ComauRobot::getPinsOUT(std::vector<long int> &pins_out) {
   pins_out.assign(dout_pins_.begin(), dout_pins_.end());
  }

 void ComauRobot::getPinsStatesOUT(std::vector<long int> &pins_state_out) {
  msg->getData("pins_state_out", pins_state_out_);
  pins_state_out.assign(pins_state_out_.begin(), pins_state_out_.end());
  }


 void ComauRobot::getEePosition(std::vector<double> &ee_position) {
   msg->getData("ee_position", ee_float_);
   ee_position.assign(ee_float_.begin(), ee_float_.end());
  }

 void ComauRobot::getTimeStamp(int32_t &timestamp) {
   msg->getData("timestamp", timestamp);
  }
 
 void ComauRobot::getStatus(char &status) {
   msg->getData("robot_status", status);
  }
 
 void ComauRobot::getError(uint32_t &error) {
   msg->getData("error_value", error);
  }
 
 void ComauRobot::getNumJoints(uint32_t &num_joints) {
   msg->getData("num_joints", num_joints);
  }

 void ComauRobot::getJointType(std::vector<int> &jnt_type) {
   msg->getData("jnt_type", jnt_type_);
   jnt_type.assign(jnt_type_.begin(), jnt_type_.end());
  }
 
 void ComauRobot::getStsSelector(uint32_t &stsSelector) {
   msg->getData("stsSelector", stsSelector);
  }

 bool ComauRobot::readMessagePackage() {
  msg = dynamic_cast<comau_tcp_interface::utils::MessagePackage *>(
      new comau_tcp_interface::utils::MessagePackage(state_client_ptr_->getRecvRecipe()));
    if (state_client_ptr_->getLastMessage(*msg)) {
     return true;
    }
    RCLCPP_ERROR(this->get_logger(),"[comau_robot] Could not get Last Message Package");
    return false;
 }
 
 bool ComauRobot::writeJointTrajectoryCommand(joint_trajectoryf_t &trajectory, ControlMode ControlMode) {
   if (ControlMode == ControlMode::MODE_JOINT_TRAJECTORY) {
 
     if (parallel_link_fix_)
       for (size_t i = 0; i < trajectory.size(); i++)
         trajectory[i].pose[2] -= trajectory[i].pose[1]; // trajectory.at(i).pose[2] -= trajectory.at(i).pose[1];
 
     return arm1_client_ptr_->sendJointTrajectoryMessage(trajectory);
   }
   /*
   if (ControlMode == ControlMode::MODE_CARTESIAN_TRAJECTORY) {
     return arm1_client_ptr_->sendCartTrajectoryMessage(trajectory);
   }
   */
   return true;
 }
 
 
 bool ComauRobot::writeTrajectoryCommand(trajectoryf_t &trajectory, ControlMode ControlMode) {
   /*
   if (ControlMode == ControlMode::MODE_JOINT_TRAJECTORY) {
 
     if (parallel_link_fix_)
       for (size_t i = 0; i < trajectory.size(); i++)
         trajectory.at(i).pose[2] -= trajectory.at(i).pose[1];
 
     //return arm1_client_ptr_->sendJointTrajectoryMessage(trajectory);
   }
   */
   if (ControlMode == ControlMode::MODE_CARTESIAN_TRAJECTORY) {
     return arm1_client_ptr_->sendCartTrajectoryMessage(trajectory);
   }
   return true;
 }
 
 bool ComauRobot::writeCommand(const std::vector<double> &joint_command, double curr_time, double curr_period,
                               ControlMode ControlMode) {
   if (ControlMode == ControlMode::MODE_SENSOR_TRACKING) {
     for (uint8_t i = 0; i < 6; i++)
       sns_trk_cmd_.at(i) = joint_command.at(i);
     return arm1_client_ptr_->sendSensorTrackingMessage(sns_trk_cmd_);
   } else if (ControlMode == ControlMode::MODE_POSITION) {
     bool allow = false;
     if (fabs(curr_time - prev_time_) > threshold_) {
       allow = true;
     }
     if (allow) {
       for (uint8_t i = 0; i < joint_command.size(); i++)
         joint_cmd_[i] = joint_command[i] * 57.2957795;
       prev_time_ = curr_time;
       return arm1_client_ptr_->sendJointMoveFlyMessage(joint_cmd_, float(fly_lin_velocity_), float(fly_dist_));
     }
   }
   return true;
 }
 
 bool ComauRobot::setIO(int &pin, bool state) {
   return robot_client_ptr_->sendIOMessage(pin, state);
 }
 bool ComauRobot::setSensorParams(int &sensor_type, int &sensor_cnvrsn, int &sensor_gain, int &sensor_time,
                                  int &sensor_ofst_lim_trans, int &sensor_ofst_lim_rot) {
   return robot_client_ptr_->sendSensorConfigurationMessage(sensor_type, sensor_cnvrsn, sensor_gain, sensor_time,
                                                            sensor_ofst_lim_trans, sensor_ofst_lim_rot);
 }
 
 bool ComauRobot::setArmState(int &state) {
   return robot_client_ptr_->sendArmStateMessage(state);
 }

}; //end namespace
/**
if (!this->get_parameter("robot_ip",state_params_.server_ip_address)){
  RCLCPP_ERROR(this->get_logger(),"Impossibile ottenere il parametro 'robot ip'");
return;
}

if (!this->get_parameter("state_server_port",state_params_.server_port)){
  RCLCPP_ERROR(this->get_logger(),"Impossibile ottenere il parametro 'state_server_port'");
return;
}

if (!this->get_parameter("robot_server_port",robot_params_.server_port)){
  RCLCPP_ERROR(this->get_logger(),"Impossibile ottenere il parametro 'robot_server_port'");
return;
}
*/



