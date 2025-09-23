/**
 * @file comau_driver.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the robot information
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include "comau_driver/comau_driver.hpp"
// ROS parameter loading
#include "rclcpp/rclcpp.hpp"
// pluginlib
#include <pluginlib/class_loader.hpp>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
  
  ComauRobot::ComauRobot(rclcpp::Node::SharedPtr &nh) : name_("comau_driver"), nh_(nh) 
  {
    // Load rosparams
    //auto nh_priv_ = std::make_shared<rclcpp::Node>(nh->get_name(), name_);

    // Read parameters through ros parameter server
    //std::size_t error = 0;
    state_params_.server_ip_address = nh_->declare_parameter<std::string>("robot_ip", "192.168.56.2");
    state_params_.server_ip_address = nh_->get_parameter("robot_ip").as_string();
    state_params_.server_port = nh_->declare_parameter<std::string>("state_server_port", "1104");
    state_params_.server_port = nh_->get_parameter("state_server_port").as_string();
    robot_params_.server_port = nh_->declare_parameter("robot_server_port", "1105");
    robot_params_.server_port = nh_->get_parameter("robot_server_port").as_string();
    arm1_params_.server_port = nh_->declare_parameter<std::string>("arm1_server_port", "1101");
    arm1_params_.server_port = nh_->get_parameter("arm1_server_port").as_string();
    default_linear_velocity_ = nh_->declare_parameter<double>("default_linear_velocity", 1.0);
    default_linear_velocity_ = nh_->get_parameter("default_linear_velocity").as_double();
    threshold_ = nh_->declare_parameter<double>("threshold", 0.2);
    threshold_ = nh_->get_parameter("threshold").as_double();
    fly_lin_velocity_ = nh_->declare_parameter<double>("fly_lin_velocity", 0.2);
    fly_lin_velocity_ = nh_->get_parameter("fly_lin_velocity").as_double();
    fly_dist_ = nh_->declare_parameter<double>("fly_dist", 5.0);
    fly_dist_ = nh_->get_parameter("fly_dist").as_double();
    //verbose_ = nh_->declare_parameter<bool>("verbose", true);
    verbose_ = nh_->get_parameter("verbose").as_bool();
    sensor_type_ = nh_->declare_parameter<int>("sensor_type", 10);
    sensor_type_ = nh_->get_parameter("sensor_type").as_int();
    sensor_cnvrsn_ = nh_->declare_parameter<int>("sensor_cnvrsn", 1);
    sensor_cnvrsn_ = nh_->get_parameter("sensor_cnvrsn").as_int();
    sensor_gain_ = nh_->declare_parameter<int>("sensor_gain", 100);
    sensor_gain_ = nh_->get_parameter("sensor_gain").as_int();
    sensor_time_ = nh_->declare_parameter<int>("sensor_time", 0);
    sensor_time_ = nh_->get_parameter("sensor_time").as_int();
    sensor_ofst_lim_trans_ = nh_->declare_parameter<int>("sensor_ofst_lim_trans", 500);
    sensor_ofst_lim_trans_ = nh_->get_parameter("sensor_ofst_lim_trans").as_int();
    sensor_ofst_lim_rot_ = nh_->declare_parameter<int>("sensor_ofst_lim_rot", 800);
    sensor_ofst_lim_rot_ = nh_->get_parameter("sensor_ofst_lim_rot").as_int();

    din_pins_ = {-1, -1, -1, -1, -1, -1};
    dout_pins_ = {-1, -1, -1, -1, -1, -1};
    nh_->declare_parameter("din_pins", rclcpp::PARAMETER_INTEGER_ARRAY);
    nh_->declare_parameter("dout_pins", rclcpp::PARAMETER_INTEGER_ARRAY);
    din_pins_  = nh_->get_parameter("din_pins").as_integer_array();
    dout_pins_ = nh_->get_parameter("dout_pins").as_integer_array();
    if (din_pins_.size() != 6 || dout_pins_.size() != 6) {
      RCLCPP_ERROR(rclcpp::get_logger("comau_driver")," The size of the dout and din parameters must be 6");
      rclcpp::shutdown();
    }
    
    robot_params_.server_ip_address = state_params_.server_ip_address;
    arm1_params_.server_ip_address = state_params_.server_ip_address;
    state_params_.log_tag = "state_tcp_client";
    robot_params_.log_tag = "motion_tcp_client";
    arm1_params_.log_tag = "arm1_tcp_client";

    // get robot specific transforms
    parallel_link_fix_ = nh_->declare_parameter<bool>("parallel_joint_fix", false);
    parallel_link_fix_ = nh_->get_parameter("parallel_joint_fix").as_bool();
}

ComauRobot::~ComauRobot() {
  if (use_robot_server_) {
    RCLCPP_ERROR(rclcpp::get_logger("comau_driver")," Terminating PDL Programs");
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
  pluginlib::ClassLoader<comau_tcp_interface::ComauClientBase> client_loader("comau_tcp_interface", "comau_tcp_interface::ComauClientBase");
  
  if (use_state_server) {
    // tcp interface state client ptr
    try {
      state_client_ptr_.reset(new comau_tcp_interface::StateClient());
      if (!state_client_ptr_->initialize(state_params_)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver")," State Client could not initialized ");
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver")," Robot Command Client could not initialized ");
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
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver")," Arm 1 Motion Command Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver"), e.what());
      return false;
    }
  }

  return true;
}

float ComauRobot::getDefaultLinVel()
{
  return (float)default_linear_velocity_;
}
bool ComauRobot::setMoveflyParams(double &threshold, double &lin_velocity, double &fly_dist) {
  threshold_ = threshold;
  fly_lin_velocity_ = lin_velocity;
  fly_dist_ = fly_dist;
  return true;
}
void ComauRobot::getJointPosition(std::vector<double> &joint_position, uint32_t &num_robot_joints, std::vector<int> &jnt_type) { /*ANDY vedere se fare un metodo per scegliere il tipo di giunto*/
  msg->getData("joint_position", joints_float_);
  if (parallel_link_fix_)
    joints_float_[2] += joints_float_[1];
  //joints_float_ = comau_tcp_interface::utils::toRad(joints_float_);
  for(size_t si_i = 0; si_i < num_robot_joints; /*jnt_type_.size()*/ si_i++)
  {
    switch (jnt_type_.at(si_i))
    {
      case 0:
        joints_float_.at(si_i) =  comau_tcp_interface::utils::toRad(joints_float_.at(si_i));
        //joint_position.push_back(joints_float_.at(si_i));
        joint_position.at(si_i) = joints_float_.at(si_i);
        break;
      case 1:
        joints_float_.at(si_i) = comau_tcp_interface::utils::toMeter(joints_float_.at(si_i));
        //joint_position.push_back(joints_float_.at(si_i));
        joint_position.at(si_i) = joints_float_.at(si_i);
        break;
      default: /* No error */
        joints_float_.at(si_i) =  comau_tcp_interface::utils::toRad(joints_float_.at(si_i));
        //joint_position.push_back(joints_float_.at(si_i));
        joint_position.at(si_i) = joints_float_.at(si_i);
        break;
    }
  }
  
  //joint_position.assign(joints_float_.begin(), joints_float_.end());
}


void ComauRobot::getPinsIN(std::vector<int> &pins_in) {
  pins_in.assign(din_pins_.begin(), din_pins_.end());
}
void ComauRobot::getPinsStatesIN(std::vector<int64_t> &pins_state_in) {

  msg->getData("pins_state_in", pins_state_in_);
  pins_state_in.assign(pins_state_in_.begin(), pins_state_in_.end());
}
void ComauRobot::getPinsOUT(std::vector<int> &pins_out) {
  pins_out.assign(dout_pins_.begin(), dout_pins_.end());
}
void ComauRobot::getPinsStatesOUT(std::vector<int64_t> &pins_state_out) {
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
void ComauRobot::getSensorType(int32_t &sensor_type) {
  msg->getData("sensor_type_feedback", sensor_type);
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
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_driver")," Could not get Last Message Package");
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

bool ComauRobot::setIO(int &pin, bool &state) {
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

} // namespace comau_driver
