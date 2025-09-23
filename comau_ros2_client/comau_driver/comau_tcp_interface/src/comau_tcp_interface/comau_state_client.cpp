/**
 * @file comau_state_client.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node recives the robot data
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "comau_tcp_interface/comau_state_client.hpp"

using namespace comau_tcp_interface::utils;

namespace comau_tcp_interface {

StateClient::~StateClient() {
  close();
  receiving_thread_->detach();
}

bool StateClient::initialize(const ComauTcpInterfaceParameters &params) {
  is_connected_ = false;
  // Assign parameters
  params_ptr_.reset(new ComauTcpInterfaceParameters(params));

  // Describe the incoming message
  incoming_msg_descr_.push_back("timestamp");
  incoming_msg_descr_.push_back("robot_status");
  incoming_msg_descr_.push_back("stsSelector");
  incoming_msg_descr_.push_back("sensor_type_feedback");
  incoming_msg_descr_.push_back("joint_position");
  incoming_msg_descr_.push_back("ee_position");
  incoming_msg_descr_.push_back("pins_state_in");
  incoming_msg_descr_.push_back("pins_state_out");
  incoming_msg_descr_.push_back("error_value");
  incoming_msg_descr_.push_back("num_joints");
  incoming_msg_descr_.push_back("jnt_type");
  last_recv_msg_.reset(new MessagePackage(incoming_msg_descr_));

  // Fetch std::future object associated with promise
  /*future_obj_for_exit_ = exit_promise_signal_.get_future();*/

  // Starting Thread & move the future object in callback function by reference
  /*try {
    receiving_thread_.reset(new std::thread(&StateClient::callback, this, std::move(future_obj_for_exit_)));
  } catch (const std::bad_alloc &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread allocation failed: " << e.what());
    return false;
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread instantion throws : " << e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread instantion throws unexpected error ");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));*/

  return true;
}

bool StateClient::openStateThread(bool openState, bool success)
{
  
  if(openState == true)
  {
    exit_promise_signal_ = std::promise<void>();
    future_obj_for_exit_ = exit_promise_signal_.get_future();
    try {
      receiving_thread_.reset(new std::thread(&StateClient::callback, this, std::move(future_obj_for_exit_)));
    } catch (const std::bad_alloc &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread allocation failed: " << e.what());
                         success = false;
      return false;
    } catch (const boost::system::system_error &e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread instantion throws : " << e.what());
                         success = false;
      return false;
    } catch (...) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Callback thread instantion throws unexpected error ");
                         success = false;
      return false;
    }
    notCloseState = true;
  } else if (openState == false && notCloseState == true)
  {
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    close();
    receiving_thread_->detach();
    notCloseState = false;
    success = true;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  success = true;
  return true;
} /* thread*/

void StateClient::close() {
  exit_promise_signal_.set_value();
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::seconds(5));
}

bool StateClient::isConnected() 
{
  return is_connected_;
}

bool StateClient::getLastMessage(MessagePackage &msg) {
  MessagePackage *last = dynamic_cast<MessagePackage *>(last_recv_msg_.get());
  if (last != nullptr) {
    msg = *last;
    return true;
  }
  return false;
}

void StateClient::receive(std::chrono::milliseconds timeout) {

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

bool StateClient::send(MessagePackage &msg) {
  return true;
  // State client for now does not send something
}

bool StateClient::validate() {

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

void StateClient::callback(std::future<void> exit_signal) {
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
        //if (!validate()) 
        {
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

  RCLCPP_INFO_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " Receiving callback ending...");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(params_ptr_->log_tag), " State Client Connection is Closed.");
}

} // namespace comau_tcp_interface

PLUGINLIB_EXPORT_CLASS(comau_tcp_interface::StateClient, comau_tcp_interface::ComauClientBase)
