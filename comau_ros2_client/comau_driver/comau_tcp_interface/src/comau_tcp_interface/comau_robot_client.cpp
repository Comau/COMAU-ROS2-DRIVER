/**
 * @file comau_robot_client.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node send the robot data
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "comau_tcp_interface/comau_robot_client.hpp"

using namespace comau_tcp_interface::utils;

namespace comau_tcp_interface {

RobotClient::~RobotClient() {
  close();
  robot_receiving_thread_->detach();
}

bool RobotClient::initialize(const ComauTcpInterfaceParameters &params) {
  is_connected_ = false;
  // Assign parameters
  params_ptr_.reset(new ComauTcpInterfaceParameters(params));

  // Describe the incoming message
  incoming_msg_motion_descr_.push_back("timestamp");
  incoming_msg_motion_descr_.push_back("robot_status");
  last_recv_msg_.reset(new MessagePackage(incoming_msg_motion_descr_));

  // Describe the outgoing messages

  // terminate msg
  terminate_msg_descr_.push_back("robot_command");
  // reset msg
  reset_msg_descr_.push_back("robot_command");
  // disconnect msg
  disco_msg_descr_.push_back("robot_command");
  // initialize msg
  initialize_msg_descr_.push_back("robot_command");
  initialize_msg_descr_.push_back("verbose");
  initialize_msg_descr_.push_back("sensor_type");
  initialize_msg_descr_.push_back("sensor_cnvrsn");
  initialize_msg_descr_.push_back("sensor_gain");
  initialize_msg_descr_.push_back("sensor_time");
  initialize_msg_descr_.push_back("sensor_ofst_lim_trans");
  initialize_msg_descr_.push_back("sensor_ofst_lim_rot");
  initialize_msg_descr_.push_back("din_pins");
  initialize_msg_descr_.push_back("dout_pins");
  // start msg
  start_msg_descr_.push_back("robot_command");
  // configuration msg
  configuration_msg_descr_.push_back("robot_command");
  configuration_msg_descr_.push_back("sensor_type");
  configuration_msg_descr_.push_back("sensor_cnvrsn");
  configuration_msg_descr_.push_back("sensor_gain");
  configuration_msg_descr_.push_back("sensor_time");
  configuration_msg_descr_.push_back("sensor_ofst_lim_trans");
  configuration_msg_descr_.push_back("sensor_ofst_lim_rot");
  // io msg
  io_msg_descr_.push_back("robot_command");
  io_msg_descr_.push_back("set_pin");
  io_msg_descr_.push_back("set_pin_state");
  // traj joint msg
  traj_joint_msg_descr_.push_back("motion_type");
  traj_joint_msg_descr_.push_back("trajectory_size");
  traj_joint_msg_descr_.push_back("joint_trajectory");
  // traj cart msg
  traj_cart_msg_descr_.push_back("motion_type");
  traj_cart_msg_descr_.push_back("trajectory_size");
  traj_cart_msg_descr_.push_back("trajectory");
  // sensor tracking msg
  sns_trk_msg_descr_.push_back("motion_type");
  sns_trk_msg_descr_.push_back("sensor_tracking_command");
  // movefly msg
  movefly_msg_descr_.push_back("motion_type");
  movefly_msg_descr_.push_back("linear_velocity");
  movefly_msg_descr_.push_back("fly_dist");
  movefly_msg_descr_.push_back("joint_position_command");
  // position msg
  pos_msg_descr_.push_back("motion_type");
  pos_msg_descr_.push_back("joint_position_command");
  // arm state msg
  arm_state_msg_descr_.push_back("robot_command");

  // Fetch std::future object associated with promise
  /*robot_future_obj_for_exit_ = robot_exit_promise_signal_.get_future();*/
  // Starting Thread & move the future object in callback function by reference
  /*try {
    robot_receiving_thread_.reset(new std::thread(&RobotClient::callback, this, std::move(robot_future_obj_for_exit_))); //tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
  }catch (const std::bad_alloc &e) {
    RCLCPP_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread allocation failed: " << e.what());
    return false;
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread instantion throws : " << e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread instantion throws unexpected error ");
    return false;
  }*/ /*catch (const boost::system::system_error &e) {
    RCLCPP_ERROR_STREAM("[" << params_ptr_->log_tag << "] " << e.what());
    return false;
  }*/

  //std::this_thread::sleep_for(std::chrono::milliseconds(50));

  return true;
}

bool RobotClient::openRobotThread(bool openRobot, bool success)
{

  if(openRobot == true)
  { 
    robot_exit_promise_signal_ = std::promise<void>();
    robot_future_obj_for_exit_ = robot_exit_promise_signal_.get_future();
    try {
      robot_receiving_thread_.reset(new std::thread(&RobotClient::callback, this, std::move(robot_future_obj_for_exit_))); //tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
    }catch (const std::bad_alloc &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread allocation failed: " << e.what());
                        success = false;
      return false;
    } catch (const boost::system::system_error &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread instantion throws : " << e.what());
                        success = false;
      return false;
    } catch (...) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread instantion throws unexpected error ");
                        success = false;
      return false;
    }
    notRobotState = true;
  } else if (openRobot == false && notRobotState == true)
  {
    this->sendDisconnectMessage();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_exit_promise_signal_.set_value();
    is_connected_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_receiving_thread_->detach();
    notRobotState = false;
    success = true;
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  success = true;
  return true;
} /* thread*/

bool RobotClient::openHandlerThread(bool openHandler, bool success)
{
  if(openHandler == true)
  {
    robot_exit_promise_signal_ = std::promise<void>();
    robot_future_obj_for_exit_ = robot_exit_promise_signal_.get_future();
    try {
      robot_receiving_thread_.reset(new std::thread(&RobotClient::callback, this, std::move(robot_future_obj_for_exit_))); //tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
    }catch (const std::bad_alloc &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread allocation failed: " << e.what());
                        success = false;
      return false;
    } catch (const boost::system::system_error &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread instantion throws : " << e.what());
                        success = false;
      return false;
    } catch (...) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), "Callback thread instantion throws unexpected error ");
                        success = false;
      return false;
    }
    notHandlerState = true;
  } else if (openHandler == false && notHandlerState == true)
  {
    /*this->sendResetMessage();*/
    std::this_thread::sleep_for(std::chrono::seconds(5));
    robot_exit_promise_signal_.set_value();
    is_connected_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    robot_receiving_thread_->detach();
    notHandlerState = false;
    success = true;
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  success = true;
  return true;
} /* thread*/
bool RobotClient::openStateThread(bool openState, bool success) /*TEMP*/
{
  return true;
}
void RobotClient::close() {
  this->sendTerminateMessage();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::seconds(3));
}

bool RobotClient::isConnected() {
  return is_connected_;
}

bool RobotClient::getLastMessage(MessagePackage &msg) {
  std::cout << "getLastMessage" << std::endl;
  MessagePackage *last = dynamic_cast<MessagePackage *>(last_recv_msg_.get());
  std::cout << "last" << std::endl;
  if (last != nullptr) {
    msg = *last;
    return true;
  }
  return false;
}

void RobotClient::receive(std::chrono::milliseconds timeout) {

  std::vector<uint8_t> recv_raw_data(last_recv_msg_->getCapacity());
  size_t read_len = 0;

  tcp_interface_ptr_->read(recv_raw_data, sizeof(recv_raw_data), read_len);
  if (read_len == 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Received zero bytes message");
    return;
  }

  MessageParser mp(recv_raw_data.data(), read_len);
  if (!last_recv_msg_->parseWith(mp)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Received message could not parsed");
    return;
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Received message : " << last_recv_msg_->toString());
}

bool RobotClient::send(MessagePackage &msg) {
  size_t written_size = 0;
  std::vector<uint8_t> send_buffer(msg.getSize());
  size_t serielized_buffer_size = msg.serializePackage(send_buffer.data());

  if (serielized_buffer_size > msg.getSize()) {
    send_buffer.resize(serielized_buffer_size);
  }

  tcp_interface_ptr_->write(send_buffer, serielized_buffer_size, written_size);

  if (written_size != serielized_buffer_size) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed buffer size: " << serielized_buffer_size
                         << "  written: " << written_size);
    return false;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger(params_ptr_->log_tag), msg.toString());
  return true;
}

bool RobotClient::sendJointPositionMessage(utils::vector6f_t joint_position_command) {
  utils::MessagePackage msg(getSendPosRecipe());
  bool com1 = msg.setData("motion_type", MotionType::JOINT_POSITION);
  bool com2 = msg.setData("joint_position_command", joint_position_command);
  if (com1 && com2) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendJointMoveFlyMessage(utils::vector6f_t joint_position_command, float lin_velocity,
                                          float fly_dist) {
  utils::MessagePackage msg(getSendMoveFlyRecipe());
  bool com1 = msg.setData("motion_type", MotionType::JOINT_MOVEFLY);
  bool com2 = msg.setData("linear_velocity", lin_velocity);
  bool com3 = msg.setData("fly_dist", fly_dist);
  bool com4 = msg.setData("joint_position_command", joint_position_command);
  if (com1 && com2 && com3 && com4) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendSensorTrackingMessage(utils::vector6f_t sensor_tracking_command) {
  utils::MessagePackage msg(getSendSnsTrkRecipe());
  bool com1 = msg.setData("motion_type", MotionType::SENSOR_TRACKING);
  bool com2 = msg.setData("sensor_tracking_command", sensor_tracking_command);
  if (com1 && com2) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendJointTrajectoryMessage(utils::joint_trajectoryf_t trajectory) {
  utils::MessagePackage msg(getSendTrajJointRecipe());
  uint32_t traj_size = static_cast<uint32_t>(trajectory.size());
  // float default_linear_velocity = 0.1f;
  bool com1 = msg.setData("motion_type", MotionType::JOINT_TRAJECTORY);
  // bool com2 = msg.setData("linear_velocity", default_linear_velocity);
  bool com3 = msg.setData("trajectory_size", traj_size);
  bool com4 = msg.setData("joint_trajectory", trajectory);
  if (com1 /*&& com2*/ && com3 && com4) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendCartTrajectoryMessage(utils::trajectoryf_t trajectory) {
  utils::MessagePackage msg(getSendTrajCartRecipe());
  uint32_t traj_size = static_cast<uint32_t>(trajectory.size());
  //float default_linear_velocity = 0.1f;
  bool com1 = msg.setData("motion_type", MotionType::CARTESIAN_TRAJECTORY);
  //bool com2 = msg.setData("linear_velocity", default_linear_velocity);
  bool com3 = msg.setData("trajectory_size", traj_size);
  bool com4 = msg.setData("trajectory", trajectory);
  if (com1 /* && com2 */ && com3 && com4) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendSensorConfigurationMessage(int &sensor_type, int &sensor_cnvrsn, int &sensor_gain,
                                                 int &sensor_time, int &sensor_ofst_lim_trans,
                                                 int &sensor_ofst_lim_rot) {
  utils::MessagePackage msg(getSendConfigurationRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::CONFIGURATION);
  bool com2 = msg.setData("sensor_type", sensor_type);
  bool com3 = msg.setData("sensor_cnvrsn", sensor_cnvrsn);
  bool com4 = msg.setData("sensor_gain", sensor_gain);
  bool com5 = msg.setData("sensor_time", sensor_time);
  bool com6 = msg.setData("sensor_ofst_lim_trans", sensor_ofst_lim_trans);
  bool com7 = msg.setData("sensor_ofst_lim_rot", sensor_ofst_lim_rot);
  if (com1 && com2 && com3 && com4 && com5 && com6 && com7) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}
bool RobotClient::sendIOMessage(int pin, bool state) {
  int32_t state_on = 1;
  int32_t state_off = 0;
  utils::MessagePackage msg(getSendIORecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::IO);
  bool com2 = msg.setData("set_pin", pin);
  bool com3;
  if (state)
    com3 = msg.setData("set_pin_state", state_on);
  else
    com3 = msg.setData("set_pin_state", state_off);
  if (com1 && com2 && com3) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendTerminateMessage() {
  utils::MessagePackage msg(getSendTerminateRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::TERMINATE);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendResetMessage() {
  utils::MessagePackage msg(getSendResetRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::RESET);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendCancelMessage() {
  utils::MessagePackage msg(getSendResetRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::CANCEL);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendDisconnectMessage() {
  utils::MessagePackage msg(getSendDisconnectRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::DISCONNECT);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendInitializeMessage(bool with_verbose, int &sensor_type, int &sensor_cnvrsn, int &sensor_gain,
                                        int &sensor_time, int &sensor_ofst_lim_trans, int &sensor_ofst_lim_rot,
                                        std::vector<int64_t> &din_pins, std::vector<int64_t> &dout_pins) {
  utils::MessagePackage msg(getSendInitializeRecipe());
  uint32_t verb = 1;
  uint32_t no_verb = 0;
  vector6i_t din_pins_, dout_pins_;
  for (uint8_t i = 0; i < din_pins.size(); i++) {
    din_pins_[i] = din_pins[i];
    dout_pins_[i] = dout_pins[i];
  }
  bool com2;
  bool com1 = msg.setData("robot_command", RobotCommand::INITIALIZE);
  if (with_verbose)
    com2 = msg.setData("verbose", verb);
  else
    com2 = msg.setData("verbose", no_verb);

  bool com3 = msg.setData("sensor_type", sensor_type);
  bool com4 = msg.setData("sensor_cnvrsn", sensor_cnvrsn);
  bool com5 = msg.setData("sensor_gain", sensor_gain);
  bool com6 = msg.setData("sensor_time", sensor_time);
  bool com7 = msg.setData("sensor_ofst_lim_trans", sensor_ofst_lim_trans);
  bool com8 = msg.setData("sensor_ofst_lim_rot", sensor_ofst_lim_rot);
  bool com9 = msg.setData("din_pins", din_pins_);
  bool com10 = msg.setData("dout_pins", dout_pins_);
  if (com1 && com2 && com3 && com4 && com5 && com6 && com7 && com8 && com9 && com10) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendArmStateMessage(int state) {
  utils::MessagePackage msg(getSendArmStateRecipe());
  bool com1;
  if (state == 0)
    com1 = msg.setData("robot_command", RobotCommand::TERMINATE);
  if (state == 1)
    com1 = msg.setData("robot_command", RobotCommand::LOCK);
  if (state == 2)
    com1 = msg.setData("robot_command", RobotCommand::RESET);
  if (state == 3)
    com1 = msg.setData("robot_command", RobotCommand::UNLOCK);
  if (state == 4)
    com1 = msg.setData("robot_command", RobotCommand::CANCEL);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::sendStartMessage() {
  utils::MessagePackage msg(getSendStartRecipe());
  bool com1 = msg.setData("robot_command", RobotCommand::START);
  if (com1) {
    return send(msg);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Send message failed ");
    return false;
  }
}

bool RobotClient::validate() {

  const int UTC_diff = 7199; // The time difference between the two timestamps (1h59m59s) constant due to SNTP sync.
  const int validation_threshold = 1; // The threshold (in seconds) between send and receive.

  /*std::int32_t time;
  if (last_recv_msg_->getData("timestamp", time)) {
    if (time - ros::Time::now().toSec() - UTC_diff > validation_threshold ||
        time - ros::Time::now().toSec() - UTC_diff < -validation_threshold) {
      return false;
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Message validation error: Time difference between COMAU CONTROLLER and ROS MASTER is over "
              << validation_threshold);
    } else {
      return true;
    }
  } else {
    return false;
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Message validation error: COMAU CONTROLLER timestamp could not parsed correctly");
  }*/
  return true;
}

void RobotClient::callback(std::future<void> exit_signal) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Receiving callback starting for host ip : " << params_ptr_->server_ip_address << ":"
                      << params_ptr_->server_port);

  // TODO remove try to connect it is not working with the current server
  while (exit_signal.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
    if (!is_connected_) {
      try {
        tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
        is_connected_ = true;
      } catch (const boost::system::system_error &e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), e.what());
        is_connected_ = false;
        size_t timeout = 10;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Trying again in " << timeout << " seconds");
        std::this_thread::sleep_for(std::chrono::seconds(timeout));
        continue;
      }
    } else {
      try {
        receive(std::chrono::seconds(1));
        if (!validate()) {
          // TODO uncomment the line below when validate is fully tested
          // is_connected_ = false;
        }
      } catch (const boost::system::system_error &e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), e.what());
        is_connected_ = false;
        continue;
      }
    }
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Receiving callback ending ");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Connection is Closed.");
}

} // namespace comau_tcp_interface

PLUGINLIB_EXPORT_CLASS(comau_tcp_interface::RobotClient, comau_tcp_interface::ComauClientBase)
